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

#include "PipelineMappingMultiNoDropProcessing.h"
#include <boost/log/core.hpp>
#include <boost/timer.hpp>
#include <boost/thread/thread.hpp>
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace MAPPING {

#define NB_LOCALKEYFRAMES 10
#define NB_NEWKEYFRAMES_LOOP 20

// Public methods

    PipelineMappingMultiNoDropProcessing::PipelineMappingMultiNoDropProcessing():ConfigurableBase(xpcf::toUUID<PipelineMappingMultiNoDropProcessing>())
    {
        LOG_DEBUG("PipelineMappingMultiNoDropProcessing constructor");

        try {
            declareInterface<api::pipeline::IMappingPipeline>(this);

            LOG_DEBUG("Components injection declaration");

            declareInjectable<api::slam::IBootstrapper>(m_bootstrapper);
            declareInjectable<api::solver::map::IBundler>(m_bundler, "BundleFixedKeyframes");
            declareInjectable<api::solver::map::IBundler>(m_globalBundler);
			declareInjectable<api::geom::IUndistortPoints>(m_undistortKeypoints);            
            declareInjectable<api::slam::ITracking>(m_tracking);
            declareInjectable<api::slam::IMapping>(m_mapping);
            declareInjectable<api::pipeline::IMapUpdatePipeline>(m_mapUpdatePipeline, true);
            declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
            declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
			declareInjectable<api::storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
			declareInjectable<api::storage::IMapManager>(m_mapManager);
            declareInjectable<api::features::IDescriptorsExtractorFromImage>(m_descriptorExtractor);
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);

            LOG_DEBUG("All component injections declared");

            LOG_DEBUG("Set the mapping function for asynchronous task");

            // Bootstrap processing function
            if (m_bootstrapTask == nullptr) {
                auto fnBootstrapProcessing = [&]() {
                    correctPoseAndBootstrap();
                };

                m_bootstrapTask = new xpcf::DelegateTask(fnBootstrapProcessing, true);
            }

            // Feature extraction processing function
            if (m_featureExtractionTask == nullptr) {
                auto fnFeatureExtractionProcessing = [&]() {
                    featureExtraction();
                };

                m_featureExtractionTask = new xpcf::DelegateTask(fnFeatureExtractionProcessing, true);
            }

            // Update visibility processing function
            if (m_updateVisibilityTask == nullptr) {
                auto fnUpdateVisibilityProcessing = [&]() {
                    updateVisibility();
                };

                m_updateVisibilityTask = new xpcf::DelegateTask(fnUpdateVisibilityProcessing, true);
            }

            // Mapping processing function
            if (m_mappingTask == nullptr) {
                auto fnMappingProcessing = [&]() {
                    mapping();
                };

                m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing, true);
            }

            // Loop closure processing function
            if (m_loopClosureTask == nullptr) {
                auto fnLoopClosureProcessing = [&]() {
                    loopClosure();
                };

                m_loopClosureTask = new xpcf::DelegateTask(fnLoopClosureProcessing, true);
            }
        }
        catch (xpcf::Exception & e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }

    void PipelineMappingMultiNoDropProcessing::onInjected() {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::onInjected");

        // Get properties
        m_reprojErrorThreshold = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
    }

    PipelineMappingMultiNoDropProcessing::~PipelineMappingMultiNoDropProcessing() {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing destructor");

        delete m_bootstrapTask;
        delete m_featureExtractionTask;
        delete m_updateVisibilityTask;
        delete m_mappingTask;
        delete m_loopClosureTask;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::init() {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing init");

        LOG_DEBUG("Initialize instance attributes");

        // Initialize private members
        m_countNewKeyframes = 0;

        if (m_mapManager != nullptr) {
            m_mapManager->setMap(xpcf::utils::make_shared<Map>());
        }

        m_T_M_W = Transform3Df::Identity();

        // Initial bootstrap status
        m_isBootstrapFinished = false;

        LOG_DEBUG("Empty buffers");

        m_sharedBufferCamImagePoseCapture.clear();
        m_sharedBufferFrame.clear();
        m_sharedBufferFrameBootstrap.clear();
        m_sharedBufferAddKeyframe.clear();
        m_dropBufferNewKeyframe.clear();
        m_dropBufferNewKeyframeLoop.clear();

        if (m_mapUpdatePipeline != nullptr){

            LOG_DEBUG("Map Update pipeline URL = {}",
                     m_mapUpdatePipeline->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the remote map update pipeline");

            try {
                if (m_mapUpdatePipeline->init() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while initializing the remote map update pipeline");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the map update pipeline: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::setCameraParameters(const CameraParameters & cameraParams) {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::setCameraParameters");

        m_cameraParams = cameraParams;

        m_bootstrapper->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
		m_tracking->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_mapping->setCameraParameters(m_cameraParams);
        m_loopDetector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopCorrector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
		m_undistortKeypoints->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);

        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
                  m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::start() {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::start");

        // Check members initialization
        if ((m_cameraParams.resolution.width > 0) && (m_cameraParams.resolution.height > 0)) {

            if (!m_tasksStarted) {
                LOG_DEBUG("Start processing tasks");

                m_bootstrapTask->start();
                m_featureExtractionTask->start();
                m_updateVisibilityTask->start();
                m_mappingTask->start();
                m_loopClosureTask->start();

                m_tasksStarted = true;
            }
        }
        else {
            LOG_DEBUG("Camera parameters and/or fiducial marker description not set");
            return FrameworkReturnCode::_ERROR_;
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::stop() {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::stop");

        if (isBootstrapFinished()){
            LOG_DEBUG("Wait until all images have been processed...");
            while (!m_sharedBufferCamImagePoseCapture.empty()){
               boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
            }

            LOG_DEBUG("Bundle adjustment, map pruning and global map udate");
            globalBundleAdjustment();
        }

        if (m_tasksStarted) {
            LOG_DEBUG("Stop processing tasks");

            m_loopClosureTask->stop();
            m_mappingTask->stop();
            m_updateVisibilityTask->stop();
            m_featureExtractionTask->stop();
            m_bootstrapTask->stop();

            m_tasksStarted = false;
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::mappingProcessRequest(const SRef<Image> image, const Transform3Df & pose) {

        LOG_DEBUG("PipelineMappingMultSolARImageConvertorOpencviProcessing::mappingProcessRequest");
        // Correct pose 
        Transform3Df poseCorrected = m_T_M_W * pose;
		// Send image and corrected pose to process
        m_sharedBufferCamImagePoseCapture.push(std::make_pair(image, poseCorrected));
        LOG_INFO("Nb images in buffer = {}", m_sharedBufferCamImagePoseCapture.size());
        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiNoDropProcessing::getDataForVisualization(std::vector<SRef<CloudPoint>> & outputPointClouds,
                                                std::vector<Transform3Df> & keyframePoses) const {

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::getDataForVisualization");

        if (isBootstrapFinished()) {

            std::vector<SRef<Keyframe>> allKeyframes;
            keyframePoses.clear();

//            LOG_DEBUG("Bootstrap finished");

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

    void PipelineMappingMultiNoDropProcessing::featureExtraction() {

        boost::timer processing_timer;
        processing_timer.restart();

		std::pair<SRef<Image>, Transform3Df> imagePose;

		if (!m_sharedBufferCamImagePoseCapture.tryPop(imagePose)) {
			xpcf::DelegateTask::yield();
			return;
		}
		SRef<Image> image = imagePose.first;
		Transform3Df pose = imagePose.second;
		std::vector<Keypoint> keypoints, undistortedKeypoints;
		SRef<DescriptorBuffer> descriptors;
		if (m_descriptorExtractor->extract(image, keypoints, descriptors) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("PipelineMappingMultiNoDropProcessing::featureExtraction: nb keypoints = {} / nb descriptors = {}",
                     keypoints.size(), descriptors->getNbDescriptors());
			processing_timer.restart();
			m_undistortKeypoints->undistort(keypoints, undistortedKeypoints);
            LOG_DEBUG("PipelineMappingMultiNoDropProcessing::featureExtraction: nb undistortedKeypoints = {}",
                     undistortedKeypoints.size());
            SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, pose);
			if (isBootstrapFinished())
				m_sharedBufferFrame.push(frame);
			else
				m_sharedBufferFrameBootstrap.push(frame);
		}
        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::featureExtraction elapsed time = {} ms", processing_timer.elapsed() * 1000);
    }

    void PipelineMappingMultiNoDropProcessing::correctPoseAndBootstrap() {

		boost::timer processing_timer;
		processing_timer.restart();

//        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::correctPoseAndBootstrap = {}", isBootstrapFinished());

		SRef<Frame> frame;

		// Try to get next (image, pose) if bootstrap is not finished
		if (m_isBootstrapFinished || !m_sharedBufferFrameBootstrap.tryPop(frame)) {
			xpcf::DelegateTask::yield();
			return;
		}
        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::correctPoseAndBootstrap: new image to process");

		// do bootstrap
		SRef<Image> view;
		if (m_bootstrapper->process(frame, view) == FrameworkReturnCode::_SUCCESS) {

			LOG_DEBUG("Bootstrap finished: apply bundle adjustement");
			m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
			SRef<Keyframe> keyframe2;
			m_keyframesManager->getKeyframe(1, keyframe2);
			m_tracking->updateReferenceKeyframe(keyframe2);

			LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());

			setBootstrapSatus(true);
		}

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::correctPoseAndBootstrap elapsed time = {} ms", processing_timer.elapsed() * 1000);
	}

    void PipelineMappingMultiNoDropProcessing::updateVisibility() {

        boost::timer processing_timer;
        processing_timer.restart();

//        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::updateVisibility");

        SRef<Frame> frame;

        // Frame to process
        if (!m_sharedBufferFrame.tryPop(frame)) {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<Keyframe> newKeyframe;

        if (m_dropBufferNewKeyframe.tryPop(newKeyframe))
        {
            LOG_DEBUG("Update new keyframe in update task");
			m_tracking->updateReferenceKeyframe(newKeyframe);
        }

		// update visibility for the current frame
		SRef<Image> displayImage;
		m_tracking->process(frame, displayImage);
		LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());
        if (frame->getVisibility().size() < m_minWeightNeighbor) {
            LOG_DEBUG("PipelineMappingMultiNoDropProcessing::updateVisibility tracking lost");
            LOG_DEBUG("PipelineMappingMultiNoDropProcessing::updateVisibility elapsed time = {} ms", processing_timer.elapsed() * 1000);
            stop();
            return;
        }

        // send frame to mapping task
        m_sharedBufferAddKeyframe.push(frame);

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::updateVisibility elapsed time = {} ms", processing_timer.elapsed() * 1000);
    }

    void PipelineMappingMultiNoDropProcessing::mapping() {

        boost::timer processing_timer;
        processing_timer.restart();

//        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::mapping");

        SRef<Frame> frame;

        // Images to process ?
        if (!m_sharedBufferAddKeyframe.tryPop(frame)) {
            xpcf::DelegateTask::yield();
            return;
        }

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
            m_countNewKeyframes++;
            m_dropBufferNewKeyframeLoop.push(keyframe);
        }

        if (keyframe)
            m_dropBufferNewKeyframe.push(keyframe);

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::mapping elapsed time = {} ms", processing_timer.elapsed() * 1000);
    }

    void PipelineMappingMultiNoDropProcessing::loopClosure() {

        boost::timer processing_timer;
        processing_timer.restart();

//        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::loopClosure");

        SRef<Keyframe> lastKeyframe;

        if ((m_countNewKeyframes < NB_NEWKEYFRAMES_LOOP) ||
            !m_dropBufferNewKeyframeLoop.tryPop(lastKeyframe))
        {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<Keyframe> detectedLoopKeyframe;
        Transform3Df sim3Transform;
        std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
        if (m_loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
            // detected loop keyframe
            LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
            LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
            LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
            // performs loop correction
            {
                m_loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
                m_countNewKeyframes = 0;
                Eigen::Matrix3f scale;
                Eigen::Matrix3f rot;
                sim3Transform.computeScalingRotation(&scale, &rot);
                sim3Transform.linear() = rot;
                sim3Transform.translation() = sim3Transform.translation() / scale(0, 0);
                m_T_M_W = sim3Transform * m_T_M_W;
            }
            // loop optimization
            Transform3Df keyframeOldPose = lastKeyframe->getPose();
            m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
            // map pruning
            m_mapManager->pointCloudPruning();
            m_mapManager->keyframePruning();
            // update pose correction
            Transform3Df transform = lastKeyframe->getPose() * keyframeOldPose.inverse();
            m_T_M_W = transform * m_T_M_W;
        }

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::loopClosure elapsed time = {} ms", processing_timer.elapsed() * 1000);
    }

    void PipelineMappingMultiNoDropProcessing::globalBundleAdjustment() {

        boost::timer processing_timer;
        processing_timer.restart();

//        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::globalBundleAdjustment");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
        // Map pruning
		m_mapManager->pointCloudPruning();
		m_mapManager->keyframePruning();
        LOG_INFO("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        if (m_mapUpdatePipeline != nullptr){
            try {
                LOG_DEBUG("Start the remote map update pipeline");

                if (m_mapUpdatePipeline->start() == FrameworkReturnCode::_SUCCESS) {

                    LOG_DEBUG("Send local map to the remote map update pipeline");

                    SRef<Map> localMap;
                    m_mapManager->getMap(localMap);

                    if (m_mapUpdatePipeline->mapUpdateRequest(localMap) == FrameworkReturnCode::_SUCCESS) {
                        LOG_DEBUG("Request to the remote map update pipeline has succeeded");
                    }
                    else {
                        LOG_DEBUG("Request to the remote map update pipeline has failed");
                    }

                    LOG_DEBUG("Stop the remote map update pipeline");

                    if (m_mapUpdatePipeline->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the remote map update pipeline");
                    }
                }
                else {
                    LOG_ERROR("Error while starting the remote map update pipeline");
                }

            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the map update pipeline: {}", e.what());
            }
        }
        else {
            LOG_DEBUG("Update global map (save to file)");
            m_mapManager->saveToFile();
        }

        LOG_DEBUG("PipelineMappingMultiNoDropProcessing::globalBundleAdjustment elapsed time = {} ms", processing_timer.elapsed() * 1000);
    }

    bool PipelineMappingMultiNoDropProcessing::isBootstrapFinished() const {

        return m_isBootstrapFinished;
    }

    void PipelineMappingMultiNoDropProcessing::setBootstrapSatus(const bool status) {

        LOG_DEBUG("Set bootstrap status to: {}", status);

        m_isBootstrapFinished = status;
    }

}
}
} // SolAR::PIPELINES::MAPPING
