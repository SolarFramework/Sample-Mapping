/**
 * @copyright Copyright (c) 2020 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include "PipelineMappingMonoProcessing.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace PIPELINES {
namespace MAPPING {

#define NB_LOCALKEYFRAMES 10
#define NB_NEWKEYFRAMES_LOOP 20


// Public methods

    PipelineMappingMonoProcessing::PipelineMappingMonoProcessing():ConfigurableBase(xpcf::toUUID<PipelineMappingMonoProcessing>())
    {
        LOG_DEBUG("PipelineMappingMonoProcessing constructor");

        try {
            declareInterface<api::pipeline::IMappingPipeline>(this);

            LOG_DEBUG("Components injection declaration");
            declareInjectable<api::slam::IBootstrapper>(m_bootstrapper);
            declareInjectable<api::solver::map::IBundler>(m_bundler, "BundleFixedKeyframes");
            declareInjectable<api::solver::map::IBundler>(m_globalBundler);
            declareInjectable<api::geom::IUndistortPoints>(m_undistortKeypoints);            
            declareInjectable<api::slam::ITracking>(m_tracking);
            declareInjectable<api::slam::IMapping>(m_mapping);
            declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
            declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
			declareInjectable<api::storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
			declareInjectable<api::storage::IMapManager>(m_mapManager);
            declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
            declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
            declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
            declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
            declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
            declareInjectable<api::geom::IProject>(m_projector);            
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);

            LOG_DEBUG("All component injections declared");

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members
            m_cameraParams.resolution.width = 0;
            m_cameraParams.resolution.height = 0;
            m_countNewKeyframes = 0;

            m_T_M_W = Transform3Df::Identity();
            m_minWeightNeighbor = 0;

            // Initial bootstrap status
            m_isBootstrapFinished = false;

            LOG_DEBUG("Set the mapping function for asynchronous task");
            // Mapping processing function
            if (m_mappingTask == nullptr) {
                auto fnMappingProcessing = [&]() {
                    processMapping();
                };

                m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
            }
        }
        catch (xpcf::Exception & e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }

    void PipelineMappingMonoProcessing::onInjected() {

        LOG_DEBUG("PipelineMappingMonoProcessing::onInjected");

        // Get properties
        m_reprojErrorThreshold = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
    }

    PipelineMappingMonoProcessing::~PipelineMappingMonoProcessing() {

        LOG_DEBUG("PipelineMappingMonoProcessing destructor");

        delete m_mappingTask;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::init() {

        LOG_DEBUG("PipelineMappingMonoProcessing::setCameraParameters");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::setCameraParameters(const CameraParameters & cameraParams) {

        LOG_DEBUG("PipelineMappingMonoProcessing::setCameraParameters");

        m_cameraParams = cameraParams;

        m_bootstrapper->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_mapping->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_projector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopDetector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopCorrector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
		m_undistortKeypoints->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
		m_tracking->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);

        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
                  m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::start() {

        LOG_DEBUG("PipelineMappingMonoProcessing::start");

        // Check members initialization
        if ((m_cameraParams.resolution.width > 0) && (m_cameraParams.resolution.height > 0)) {

            LOG_DEBUG("Start mapping processing task");
            m_mappingTask->start();
        }
        else {
            LOG_DEBUG("Camera parameters and/or fiducial marker description not set");
            return FrameworkReturnCode::_ERROR_;
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::stop() {

        LOG_DEBUG("PipelineMappingMonoProcessing::stop");
        if (isBootstrapFinished()){
            globalBundleAdjustment();
        }
        LOG_DEBUG("Stop mapping processing task");
        m_mappingTask->stop();

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::mappingProcessRequest(const SRef<Image> image, const Transform3Df & pose) {

        LOG_DEBUG("PipelineMappingMonoProcessing::mappingProcessRequest");

        // Correct pose after loop detection
        Transform3Df poseCorrected = m_T_M_W * pose;

        // Add pair (image, pose) to input drop buffer for mapping
        m_inputImagePoseBuffer.push(std::make_pair(image, poseCorrected));

        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::getDataForVisualization(std::vector<SRef<CloudPoint>> & outputPointClouds,
                                                std::vector<Transform3Df> & keyframePoses) const {

        LOG_DEBUG("PipelineMappingMonoProcessing::getDataForVisualization");

        if (isBootstrapFinished()) {

            std::vector<SRef<Keyframe>> allKeyframes;
            keyframePoses.clear();

            if (m_keyframesManager->getAllKeyframes(allKeyframes) == FrameworkReturnCode::_SUCCESS)
            {
                for (auto const &it : allKeyframes)
                    keyframePoses.push_back(it->getPose());

                return m_pointCloudManager->getAllPoints(outputPointClouds);
            }
            else {
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            return FrameworkReturnCode::_ERROR_;
        }

    }

// Private methods

    bool PipelineMappingMonoProcessing::correctPoseAndBootstrap
                                            (const SRef<Image> & image, const Transform3Df & pose) {

        LOG_DEBUG("PipelineMappingMonoProcessing::correctPoseAndBootstrap");

        SRef<Image> view;
        if (m_bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Bootstrap finished: apply bundle adjustement");

            m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);

            // Prepare mapping process
			SRef<Keyframe> keyframe2;
			m_keyframesManager->getKeyframe(1, keyframe2);
			m_tracking->updateReferenceKeyframe(keyframe2);

            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());

            setBootstrapSatus(true);
        }
        else {
            LOG_DEBUG("Boostrap not finished");

            return false;
        }

        return true;
    }

    void PipelineMappingMonoProcessing::globalBundleAdjustment() {
        LOG_DEBUG("PipelineMappingMonoProcessing::globalBundleAdjustment");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
		// map pruning
		m_mapManager->pointCloudPruning();
		m_mapManager->keyframePruning();
        LOG_DEBUG("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        LOG_DEBUG("Update global map");
        m_mapManager->saveToFile();
    }

    void PipelineMappingMonoProcessing::processMapping() {

        LOG_DEBUG("PipelineMappingMonoProcessing::processMapping");

        std::pair<SRef<Image>, Transform3Df> image_pose_pair;

        if (!isBootstrapFinished()) {

            // Process bootstrap

            // Get (image, pose) from input buffer
            if (!m_inputImagePoseBuffer.empty()) {

                m_inputImagePoseBuffer.pop(image_pose_pair);

                SRef<Image> image = image_pose_pair.first;
                Transform3Df pose = image_pose_pair.second;

                // Ask for pose correction and bootstrap
                correctPoseAndBootstrap(image, pose);
            }
        }
        else {
            // Process mapping;
            if (!m_inputImagePoseBuffer.empty()) {

                m_inputImagePoseBuffer.pop(image_pose_pair);

                SRef<Image> image = image_pose_pair.first;
                Transform3Df pose = image_pose_pair.second;

                // feature extraction image
                std::vector<Keypoint> keypoints, undistortedKeypoints;
                m_keypointsDetector->detect(image, keypoints);
				m_undistortKeypoints->undistort(keypoints, undistortedKeypoints);
                LOG_DEBUG("Keypoints size = {}", keypoints.size());

                SRef<DescriptorBuffer> descriptors;
                m_descriptorExtractor->extract(image, keypoints, descriptors);
                SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, pose);

				// update visibility for the current frame
				SRef<Image> displayImage;
				m_tracking->process(frame, displayImage);
                LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());
				if (frame->getVisibility().size() < m_minWeightNeighbor) {
					xpcf::DelegateTask::yield();
					return;
				}

                // mapping
                SRef<Keyframe> keyframe;
                if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
                    LOG_DEBUG("New keyframe id: {}", keyframe->getId());
                    // Local bundle adjustment
                    std::vector<uint32_t> bestIdx, bestIdxToOptimize;
                    m_covisibilityGraphManager->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx);
					if (bestIdx.size() < NB_LOCALKEYFRAMES)
						bestIdxToOptimize = bestIdx;
					else
						bestIdxToOptimize.insert(bestIdxToOptimize.begin(), bestIdx.begin(), bestIdx.begin() + NB_LOCALKEYFRAMES);
					bestIdxToOptimize.push_back(keyframe->getId());
					LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdxToOptimize.size());
					double bundleReprojError = m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion, bestIdx);
                    // map pruning
					std::vector<SRef<CloudPoint>> localMap;
					m_mapManager->getLocalPointCloud(keyframe, m_minWeightNeighbor, localMap);
                    int nbRemovedCP = m_mapManager->pointCloudPruning(localMap);
					std::vector<SRef<Keyframe>> localKeyframes;
					m_keyframesManager->getKeyframes(bestIdx, localKeyframes);
					int nbRemovedKf = m_mapManager->keyframePruning(localKeyframes);
					LOG_DEBUG("Nb of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
                    // try to loop detection
                    m_countNewKeyframes++;
                    // loop closure
                    if (m_countNewKeyframes >= NB_NEWKEYFRAMES_LOOP) {
                        SRef<Keyframe> detectedLoopKeyframe;
                        Transform3Df sim3Transform;
                        std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
                        if (m_loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
                            // detected loop keyframe
                            LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
                            LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
                            LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
                            // performs loop correction
                            Transform3Df keyframeOldPose = keyframe->getPose();
                            m_loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
                            // loop optimization
                            m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
							// map pruning
							m_mapManager->pointCloudPruning();
							m_mapManager->keyframePruning();
                            m_countNewKeyframes = 0;
                            // update pose correction
                            Transform3Df transform = keyframe->getPose() * keyframeOldPose.inverse();
                            m_T_M_W = transform * m_T_M_W;
                        }
                    }
                }

                // update reference keyframe
                if (keyframe) {
					m_tracking->updateReferenceKeyframe(keyframe);
                }
            }
        }
    }

    bool PipelineMappingMonoProcessing::isBootstrapFinished() const {

        // Protect variable access with mutex
        std::shared_lock lock(m_bootstrap_mutex);

        return m_isBootstrapFinished;
    }

    void PipelineMappingMonoProcessing::setBootstrapSatus(const bool status) {

        LOG_DEBUG("Set bootstrap status to: {}", status);

        // Protect variable access with mutex
        std::unique_lock lock(m_bootstrap_mutex);

        m_isBootstrapFinished = status;
    }

}
}
} // SolAR::PIPELINES::MAPPING
