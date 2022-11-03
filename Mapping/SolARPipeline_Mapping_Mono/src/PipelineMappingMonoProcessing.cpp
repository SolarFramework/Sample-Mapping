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

    PipelineMappingMonoProcessing::PipelineMappingMonoProcessing(): base::pipeline::AMappingPipeline(xpcf::toMap<PipelineMappingMonoProcessing>())
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
            declareInjectable<api::storage::ICameraParametersManager>(m_cameraParametersManager);
            declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
			declareInjectable<api::storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
			declareInjectable<api::storage::IMapManager>(m_mapManager);
            declareInjectable<api::features::IDescriptorsExtractorFromImage>(m_descriptorExtractor);
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);
            declareInjectable<api::geom::I3DTransform>(m_transform3D);
            LOG_DEBUG("All component injections declared");
        }
        catch (xpcf::Exception & e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }

    void PipelineMappingMonoProcessing::onInjected()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::onInjected");
        // Get properties
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
    }

    PipelineMappingMonoProcessing::~PipelineMappingMonoProcessing()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing destructor");
        delete m_mappingTask;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::init()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::init");
        // Initialize private members
        m_countNewKeyframes = 0;        
        m_lastTransform = Transform3Df(Maths::Matrix4f::Zero());
        // Initial bootstrap status
        m_status = MappingStatus::BOOTSTRAP;
        // loop
        m_isDetectedLoop = false;
        m_loopTransform = Transform3Df::Identity();
        // init task
        // Mapping processing function
        if (m_mappingTask == nullptr) {
            auto fnMappingProcessing = [&]() {
                processMapping();
            };
            m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
        }
        // init done
        m_init = true;
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::setCameraParameters(const CameraParameters & cameraParams)
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::setCameraParameters");
        m_cameraParams = cameraParams;
        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
                  m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        m_cameraParametersManager->addCameraParameters(m_cameraParams);
        m_cameraParamsID = m_cameraParams.id;

        m_cameraOK = true;

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::setRectificationParameters(const SolAR::datastructure::RectificationParameters & rectCam1,
                                                                                  const SolAR::datastructure::RectificationParameters & rectCam2)
    {
        LOG_ERROR("Stereo camera is not supported for this pipeline");
        return FrameworkReturnCode::_ERROR_;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::start() {

        LOG_DEBUG("PipelineMappingMonoProcessing::start");

        // Check members initialization
        if (m_init && m_cameraOK) {

            LOG_DEBUG("Start mapping processing task");
            m_mappingTask->start();
        }
        else {
            LOG_DEBUG("Check init or set camera parameters");
            return FrameworkReturnCode::_ERROR_;
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::stop()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::stop");
        LOG_DEBUG("Stop mapping processing task");
        m_mappingTask->stop();
        if (m_status != MappingStatus::BOOTSTRAP){
            globalBundleAdjustment();
        }        
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::mappingProcessRequest(const std::vector<SRef<SolAR::datastructure::Image>> & images,
                                                                             const std::vector<SolAR::datastructure::Transform3Df> & poses,
                                                                             const SolAR::datastructure::Transform3Df & transform,
                                                                             SolAR::datastructure::Transform3Df & updatedTransform,
                                                                             MappingStatus & status)
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::mappingProcessRequest");        
        // init last transform
        if (m_lastTransform.matrix().isZero())
            m_lastTransform = transform;        

        updatedTransform = transform;

        // refine transformation matrix by loop closure detection
        if (m_isDetectedLoop) {
            updatedTransform = m_loopTransform * transform;
            LOG_INFO("New transform matrix after loop detection:\n{}", updatedTransform.matrix());
            m_isDetectedLoop = false;
        }
        // drift correction
        else {
            Transform3Df driftTransform = transform * m_lastTransform.inverse();
            if (!driftTransform.isApprox(Transform3Df::Identity()) && (m_status != TRACKING_LOST)){
                driftCorrection(driftTransform);
            }
        }

        // Correct pose to the world coordinate system
        Transform3Df poseCorrected = updatedTransform * poses[0];

        // update status
        status = m_status;

        // update last transform
        m_lastTransform = updatedTransform;

        // Add pair (image, pose) to input drop buffer for mapping
        m_inputImagePoseBuffer.push(std::make_pair(images[0], poseCorrected));

        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::getDataForVisualization(std::vector<SRef<CloudPoint>> & outputPointClouds,
                                                                               std::vector<Transform3Df> & keyframePoses) const
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::getDataForVisualization");
        if (m_status != MappingStatus::BOOTSTRAP) {
            std::unique_lock<std::mutex> lock(m_mutexMapData);
            outputPointClouds = m_allPointClouds;
            keyframePoses = m_allKeyframePoses;
            return FrameworkReturnCode::_SUCCESS;
        }
        else
            return FrameworkReturnCode::_ERROR_;

    }

// Private methods

    bool PipelineMappingMonoProcessing::correctPoseAndBootstrap(const SRef<Frame> & frame) 
	{
        LOG_DEBUG("PipelineMappingMonoProcessing::correctPoseAndBootstrap");
        SRef<Image> view;
        if (m_bootstrapper->process(frame, view) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Bootstrap finished: apply bundle adjustement");
            m_bundler->bundleAdjustment();
            // Prepare mapping process
			SRef<Keyframe> keyframe2;
			m_keyframesManager->getKeyframe(1, keyframe2);
            m_lastKeyframeId = 1;
			m_tracking->setNewKeyframe(keyframe2);
            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());
            m_status = MappingStatus::MAPPING;
            getMapData();
        }
        else {
            LOG_DEBUG("Boostrap not finished");
            return false;
        }

        return true;
    }

    void PipelineMappingMonoProcessing::globalBundleAdjustment()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::globalBundleAdjustment");
        // Global bundle adjustment
        m_globalBundler->bundleAdjustment();
		// map pruning
		m_mapManager->pointCloudPruning();
		m_mapManager->keyframePruning();
        LOG_DEBUG("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());
        m_mapManager->saveToFile();
    }

    void PipelineMappingMonoProcessing::processMapping()
    {
        LOG_DEBUG("PipelineMappingMonoProcessing::processMapping");
        std::pair<SRef<Image>, Transform3Df> image_pose_pair;
		if (!m_inputImagePoseBuffer.tryPop(image_pose_pair))
			return;
		SRef<Image> image = image_pose_pair.first;
		Transform3Df pose = image_pose_pair.second;
		// feature extraction image
		std::vector<Keypoint> keypoints, undistortedKeypoints;
		SRef<DescriptorBuffer> descriptors;
		if (m_descriptorExtractor->extract(image, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
			return;		
        m_undistortKeypoints->undistort(keypoints, m_cameraParams, undistortedKeypoints);
		LOG_DEBUG("Keypoints size = {}", keypoints.size());		
        SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_cameraParamsID, pose);

        if (m_status == MappingStatus::BOOTSTRAP) {
            // Process bootstrap
            correctPoseAndBootstrap(frame);
        }
        else {
			// update visibility for the current frame
			SRef<Image> displayImage;
            if (m_tracking->process(frame, displayImage) != FrameworkReturnCode::_SUCCESS){
                LOG_INFO("PipelineMappingMultiProcessing::updateVisibility Tracking lost");
                m_status = MappingStatus::TRACKING_LOST;
                m_lastKeyframeId = m_curKeyframeId;
                return;
            }
            else
                m_status = MappingStatus::MAPPING;
            LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());

            if (!m_tracking->checkNeedNewKeyframe())
                return;

            // mapping
            SRef<Keyframe> keyframe;
            if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
                m_curKeyframeId = keyframe->getId();
                LOG_DEBUG("New keyframe id: {}", keyframe->getId());
				// Local bundle adjustment
				std::vector<uint32_t> bestIdx;
				m_covisibilityGraphManager->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx, NB_LOCALKEYFRAMES);
				bestIdx.push_back(keyframe->getId());
				LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
                double bundleReprojError = m_bundler->bundleAdjustment(bestIdx);
                // map pruning
				std::vector<SRef<CloudPoint>> localMap;
				m_mapManager->getLocalPointCloud(keyframe, m_minWeightNeighbor, localMap);
                int nbRemovedCP = m_mapManager->pointCloudPruning(localMap);
				std::vector<SRef<Keyframe>> localKeyframes;
                bestIdx.pop_back();
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
                        m_status = MappingStatus::LOOP_CLOSURE;
                        // detected loop keyframe
                        LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
                        LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
                        LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
                        // performs loop correction
                        Transform3Df keyframeOldPose = keyframe->getPose();
                        m_loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
                        // loop optimization
                        m_globalBundler->bundleAdjustment();
						// map pruning
						m_mapManager->pointCloudPruning();
						m_mapManager->keyframePruning();
                        m_countNewKeyframes = 0;
                        // update pose correction
                        m_loopTransform = keyframe->getPose() * keyframeOldPose.inverse();
                        LOG_INFO("Loop correction transform: \n{}", m_loopTransform.matrix());
                        m_isDetectedLoop = true;
                        m_status = MappingStatus::MAPPING;
                        std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    }
                }                
				// send new keyframe to tracking
				m_tracking->setNewKeyframe(keyframe);
                // update map data
                getMapData();
            }
            else
                LOG_DEBUG("Mapping fails");
        }
    }

    void PipelineMappingMonoProcessing::getMapData()
    {
        std::unique_lock<std::mutex> lock(m_mutexMapData);
        std::vector<SRef<Keyframe>> allKeyframes;
        m_allKeyframePoses.clear();
        if (m_keyframesManager->getAllKeyframes(allKeyframes) == FrameworkReturnCode::_SUCCESS) {
            for (auto const &it : allKeyframes)
                m_allKeyframePoses.push_back(it->getPose());
        }
        m_allPointClouds.clear();
        m_pointCloudManager->getAllPoints(m_allPointClouds);
    }

    void PipelineMappingMonoProcessing::driftCorrection(datastructure::Transform3Df driftTransform)
    {
        LOG_INFO("Drift correction processing");
        LOG_INFO("Drift transform:\n{}", driftTransform.matrix());
        LOG_INFO("Last keyframe: {}", m_lastKeyframeId);
        LOG_INFO("Current keyframe: {}", m_curKeyframeId);
        // get neighbor keyframe of the current keyframe        
        std::vector<uint32_t> neighborIds;
        std::vector<SRef<Keyframe>> neighborKeyframes;
        m_covisibilityGraphManager->getNeighbors(m_curKeyframeId, m_minWeightNeighbor, neighborIds);
        neighborIds.push_back(m_curKeyframeId);
        for (auto it : neighborIds) {
            SRef<Keyframe> keyframe;
            if (m_keyframesManager->getKeyframe(it, keyframe) == FrameworkReturnCode::_SUCCESS)
                neighborKeyframes.push_back(keyframe);
        }
        // get local map point
        std::vector<SRef<CloudPoint>> localPC;
        m_mapManager->getLocalPointCloud(neighborKeyframes, localPC);
        // correct drift of local keyframes and point cloud
        m_transform3D->transformInPlace(driftTransform, neighborKeyframes);
        m_transform3D->transformInPlace(driftTransform, localPC);
        // apply a bundle adjustment from the last keyframe to the current keyframe
        std::vector<uint32_t> loopKfId;
        for (int i = m_lastKeyframeId + 1; i < m_curKeyframeId; ++i)
            if (m_keyframesManager->isExistKeyframe(i))
                loopKfId.push_back(i);
        LOG_INFO("Nb of keyframes to bundle: {}", loopKfId.size());
        double errorBundle(0.0);
        if (loopKfId.size() > 0)
            errorBundle = m_globalBundler->bundleAdjustment(loopKfId);
        // update last keyframe id
        m_lastKeyframeId = m_curKeyframeId;
        LOG_INFO("Drift correction done with error: {}", errorBundle);
    }

}
}
} // SolAR::PIPELINES::MAPPING
