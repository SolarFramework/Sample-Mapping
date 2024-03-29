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

#include "PipelineMappingMultiProcessing.h"
#include <boost/log/core.hpp>
#include "core/Log.h"
#include "core/Timer.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace MAPPING {

#define NB_LOCALKEYFRAMES 10
#define NB_NEWKEYFRAMES_LOOP 20
// Nb "tracking lost" events on successive frames before sending it back to the client
#define NB_SUCCESSIVE_TRACKING_LOST 2
// After receiving GT frame, we correct the transform
// we consider that no drift within a certain time after the GT frame (e.g. 100 frames corresponding to about 5s)
#define NB_FRAMES_GT_ALIVE 100 

const std::string RELOCALIZATION_CONF_FILE = "./SolARService_Mapping_Multi_Relocalization_conf.xml";

Timer timerPipeline;
int m_nbImageRequest(0), m_nbExtractionProcess(0), m_nbFrameToUpdate(0),
	m_nbUpdateProcess(0), m_nbFrameToMapping(0), m_nbMappingProcess(0), m_idProcessGTReceived(0);

// Public methods

    PipelineMappingMultiProcessing::PipelineMappingMultiProcessing(): base::pipeline::AMappingPipeline(xpcf::toMap<PipelineMappingMultiProcessing>())
    {

        LOG_DEBUG("PipelineMappingMultiProcessing constructor");

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
            declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocPipeline, true);
            declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
            declareInjectable<api::storage::ICameraParametersManager>(m_cameraParametersManager);
            declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
			declareInjectable<api::storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
			declareInjectable<api::storage::IMapManager>(m_mapManager);
            declareInjectable<api::features::IDescriptorsExtractorFromImage>(m_descriptorExtractor);
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);
            declareInjectable<api::geom::I3DTransform>(m_transform3D);
            declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);

            LOG_DEBUG("All component injections declared");

            LOG_DEBUG("Set the mapping function for asynchronous task");

            // Bootstrap processing function
            if (m_bootstrapTask == nullptr) {
                auto fnBootstrapProcessing = [&]() {
                    correctPoseAndBootstrap();
                };

                m_bootstrapTask = new xpcf::DelegateTask(fnBootstrapProcessing);
            }

            // Feature extraction processing function
            if (m_featureExtractionTask == nullptr) {
                auto fnFeatureExtractionProcessing = [&]() {
                    featureExtraction();
                };

                m_featureExtractionTask = new xpcf::DelegateTask(fnFeatureExtractionProcessing);
            }

            // Update visibility processing function
            if (m_updateVisibilityTask == nullptr) {
                auto fnUpdateVisibilityProcessing = [&]() {
                    updateVisibility();
                };

                m_updateVisibilityTask = new xpcf::DelegateTask(fnUpdateVisibilityProcessing);
            }

            // Mapping processing function
            if (m_mappingTask == nullptr) {
                auto fnMappingProcessing = [&]() {
                    mapping();
                };

                m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
            }

            // Loop closure processing function
            if (m_loopClosureTask == nullptr) {
                auto fnLoopClosureProcessing = [&]() {
                    loopClosure();
                };

                m_loopClosureTask = new xpcf::DelegateTask(fnLoopClosureProcessing);
            }

            // Drift correction processing function
            if (m_driftCorrectionTask == nullptr) {
                auto fnDriftCorrection = [&]() {
                    driftCorrection();
                };

                m_driftCorrectionTask = new xpcf::DelegateTask(fnDriftCorrection);
            }
        }
        catch (xpcf::Exception & e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }

    void PipelineMappingMultiProcessing::onInjected()
    {
        LOG_DEBUG("PipelineMappingMultiProcessing::onInjected");
        // Get properties
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
        m_boWFeatureFromMatchedDescriptors = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("boWFeatureFromMatchedDescriptors")->getIntegerValue();
    }

    PipelineMappingMultiProcessing::~PipelineMappingMultiProcessing()
    {
        LOG_DEBUG("PipelineMappingMultiProcessing destructor");
        delete m_bootstrapTask;
        delete m_featureExtractionTask;
        delete m_updateVisibilityTask;
        delete m_mappingTask;
        delete m_loopClosureTask;
        delete m_driftCorrectionTask;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::init()
    {
        LOG_DEBUG("PipelineMappingMultiProcessing init");

        if (m_started)
            stop();

		if (m_init) {
			LOG_WARNING("Pipeline has already been initialized");
			return FrameworkReturnCode::_SUCCESS;
		}

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
        else {
            LOG_ERROR("Map Update pipeline not defined");
        }

		if (m_relocPipeline != nullptr) {

			LOG_DEBUG("Relocalization pipeline URL = {}",
				m_relocPipeline->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

			LOG_DEBUG("Initialize the remote reloc pipeline");

			try {
				if (m_relocPipeline->init() != FrameworkReturnCode::_SUCCESS) {
					LOG_ERROR("Error while initializing the remote reloc pipeline");
					return FrameworkReturnCode::_ERROR_;
				}
			}
			catch (const std::exception &e) {
				LOG_ERROR("Exception raised during remote request to the reloc pipeline: {}", e.what());
				return FrameworkReturnCode::_ERROR_;
			}
		}
		else {
			LOG_ERROR("Reloc pipeline not defined");
		}        

		m_init = true;
		
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::init(const std::string relocalizationServiceURL)
    {
        LOG_DEBUG("PipelineMappingMultiProcessing::init(relocalizationServiceURL)");

        if (relocalizationServiceURL != "") {
            // Open/create configuration file for the Relocalization service
            std::ofstream confFile(RELOCALIZATION_CONF_FILE, std::ofstream::out);

            // Check if file was successfully opened for writing
            if (confFile.is_open())
            {
                confFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>" << std::endl;
                confFile << "<xpcf-registry autoAlias=\"true\">" << std::endl << std::endl;
                confFile << "<properties>" << std::endl;
                confFile << "    <!-- gRPC proxy configuration-->" << std::endl;
                confFile << "    <configure component=\"IRelocalizationPipeline_grpcProxy\">" << std::endl;
                confFile << "        <property name=\"channelUrl\" access=\"rw\" type=\"string\" value=\""
                      << relocalizationServiceURL << "\"/>" << std::endl;
                confFile << "        <property name=\"channelCredentials\" access=\"rw\" type=\"uint\" value=\"0\"/>" << std::endl;
                confFile << "    </configure>" << std::endl << std::endl;
                confFile << "</properties>" << std::endl << std::endl;
                confFile << "</xpcf-registry>" << std::endl;

                confFile.close();
            }
            else {
                LOG_ERROR("Error when creating the Relocalization service configuration file");
                return FrameworkReturnCode::_ERROR_;
            }

            LOG_DEBUG("Load the new Relocalization properties configuration file: {}", RELOCALIZATION_CONF_FILE);

            SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();
            if (cmpMgr->load(RELOCALIZATION_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                LOG_ERROR("Failed to load properties configuration file: {}", RELOCALIZATION_CONF_FILE);
                return FrameworkReturnCode::_ERROR_;
            }

            m_relocPipeline = cmpMgr->resolve<api::pipeline::IRelocalizationPipeline>();
        }
        else {
            LOG_ERROR("Initialization with an empty Relocalization Service URL");
            return FrameworkReturnCode::_ERROR_;
        }

        return init();
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::setCameraParameters(const CameraParameters & cameraParams)
    {
        LOG_DEBUG("PipelineMappingMultiProcessing::setCameraParameters");

        if (!m_init) {
            LOG_ERROR("Pipeline has not been initialized");
            return FrameworkReturnCode::_ERROR_;
        }

        m_cameraParams = cameraParams;
        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
        m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        if (m_relocPipeline != nullptr){

            LOG_DEBUG("Set camera parameters for the relocalization service");

            try {
                if (m_relocPipeline->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting camera parameters for the relocalization service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }

        m_cameraOK = true;

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::setRectificationParameters(const SolAR::datastructure::RectificationParameters & rectCam1,
                                                                                   const SolAR::datastructure::RectificationParameters & rectCam2)
    {
        LOG_ERROR("Stereo camera is not supported for this pipeline");
        return FrameworkReturnCode::_ERROR_;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::set3DTransformSolARToWorld(const SolAR::datastructure::Transform3Df &transform)
    {
        if (m_mapManager == nullptr) {
            LOG_ERROR("Map manager is empty, can not set transform solar to world");
            return FrameworkReturnCode::_ERROR_;
        }
        FrameworkReturnCode msg;
        {
            std::lock_guard<std::mutex> lock(m_mutexMapping);
            SRef<SolAR::datastructure::Map> map;
            msg = m_mapManager->getMap(map);
            map->setTransform3D(transform);
        }
        return msg;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::start()
    {
        LOG_DEBUG("PipelineMappingMultiProcessing::start");

        if (!m_init) {
            LOG_ERROR("Pipeline has not been initialized");
            return FrameworkReturnCode::_ERROR_;
        }

        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
            return FrameworkReturnCode::_ERROR_;
        }

        if (!m_started) {

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members            
            if (m_mapManager != nullptr) {
                m_mapManager->setMap(xpcf::utils::make_shared<Map>());

                // add current camera parameters to the map manager
                SRef<CameraParameters> camParams = xpcf::utils::make_shared<CameraParameters>(m_cameraParams);
                m_mapManager->addCameraParameters(camParams);
                m_cameraParamsID = camParams->id;
            }

            m_countNewKeyframes = 0;
            m_lastTransform = Transform3Df(Maths::Matrix4f::Zero());
            m_status = MappingStatus::BOOTSTRAP;
            m_isDetectedLoop = false;
            m_isDetectedDrift = false;
            m_loopTransform = Transform3Df::Identity();
            m_isMappingIdle = true;
            m_isLoopIdle = true;
            m_isGTPoseReady = false;
            m_nbTrackingLost = 0;
            m_keyframeIds.clear();

            // Init report variables
            m_nbImageRequest = 0;
            m_nbExtractionProcess = 0;
            m_nbFrameToUpdate = 0;
            m_nbUpdateProcess = 0;
            m_nbFrameToMapping = 0;
            m_nbMappingProcess = 0;
            m_idProcessGTReceived = 0;

            LOG_DEBUG("Empty buffers");

            CaptureDropBufferElement imagePose;
            m_dropBufferCamImagePoseCapture.tryPop(imagePose);
            SRef<Frame> frame;
            m_dropBufferFrame.tryPop(frame);
            m_dropBufferFrameBootstrap.tryPop(frame);
            m_dropBufferAddKeyframe.tryPop(frame);
            SRef<Keyframe> keyframe;
            m_dropBufferNewKeyframeLoop.tryPop(keyframe);

			if (m_relocPipeline) {
				LOG_DEBUG("Start remote relocalization pipeline");
				if (m_relocPipeline->start() != FrameworkReturnCode::_SUCCESS) {
					LOG_ERROR("Cannot start relocalization pipeline");
					return FrameworkReturnCode::_ERROR_;
				}
			}

			if (!m_tasksStarted) {
				LOG_DEBUG("Start processing tasks");

                m_bootstrapTask->start();
                m_featureExtractionTask->start();
                m_updateVisibilityTask->start();
                m_mappingTask->start();
                m_loopClosureTask->start();
                m_driftCorrectionTask->start();

                m_tasksStarted = true;
            }

			m_started = true;
        }
        else {
            LOG_WARNING("Pipeline already started");
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::stop()
    {
        LOG_INFO("PipelineMappingMultiProcessing::stop");

        if (!m_init) {
            LOG_ERROR("Pipeline has not been initialized");
            return FrameworkReturnCode::_ERROR_;
        }

        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
            return FrameworkReturnCode::_ERROR_;
        }

        if (m_started) {
			m_started = false;
            if (m_tasksStarted) {
                LOG_DEBUG("Stop processing tasks");

                m_loopClosureTask->stop();
                m_driftCorrectionTask->stop();
                m_mappingTask->stop();
                m_updateVisibilityTask->stop();
                m_featureExtractionTask->stop();
                m_bootstrapTask->stop();                

                m_tasksStarted = false;
            }

			// display performance report
			LOG_INFO("Total of computation time: {} ms", timerPipeline.elapsed());
			LOG_INFO("Nb of request images: {}", m_nbImageRequest);
			LOG_INFO("Nb of feature extraction process: {}", m_nbExtractionProcess);
			LOG_INFO("Nb of update request and process: {} / {}", m_nbFrameToUpdate, m_nbUpdateProcess);
			LOG_INFO("Nb of mapping request and process: {} / {}", m_nbFrameToMapping, m_nbMappingProcess);

            if (m_status != MappingStatus::BOOTSTRAP){
                LOG_INFO("Bundle adjustment, map pruning and global map update");
                globalBundleAdjustment();
            }

			if (m_relocPipeline) {
				LOG_INFO("Stop remote relocalization pipeline");
                if (m_relocPipeline->stop() != FrameworkReturnCode::_SUCCESS) {
					LOG_ERROR("Cannot stop relocalization pipeline");
				}
			}

            m_keyframeIds.clear();
        }
        else {
            LOG_INFO("Pipeline already stopped");
        }
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::mappingProcessRequest(const std::vector<SRef<SolAR::datastructure::Image>> & images,
                                                                              const std::vector<SolAR::datastructure::Transform3Df> & posesArr,
                                                                              bool fixedPose,
                                                                              const SolAR::datastructure::Transform3Df & transformArrWorld,
                                                                              SolAR::datastructure::Transform3Df & updatedTransformArrWorld,
                                                                              MappingStatus & status)
    {
        LOG_DEBUG("PipelineMappingMultSolARImageConvertorOpencviProcessing::mappingProcessRequest");

        if (!m_init) {
            LOG_ERROR("Pipeline has not been initialized");
            return FrameworkReturnCode::_ERROR_;
        }

        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
            return FrameworkReturnCode::_ERROR_;
        }

        if (!m_started){
            LOG_ERROR("Pipeline has not been started");
            return FrameworkReturnCode::_ERROR_;
        }

		// get current transform SolAR to World 
		Transform3Df T_SolAR_World;
		{
			std::lock_guard<std::mutex> lock(m_mutexMapping);
			SRef<SolAR::datastructure::Map> map;
			m_mapManager->getMap(map);
			T_SolAR_World = map->getTransform3D();
		}

		// compute T_ARr_SolAR 
        Transform3Df T_ARr_SolAR = T_SolAR_World.inverse()*transformArrWorld;

		// transform is updated at each reception of GT pose, allowing to compensate for pose drift in ARr
		// this compensation is transferred to T_ARr_SolAR
		// now we can do all the computations in SolAR space ...

        // init last transform
        if (m_lastTransform.matrix().isZero())
            m_lastTransform = T_ARr_SolAR;

        updatedTransformArrWorld = T_ARr_SolAR;

        if (m_status != MappingStatus::BOOTSTRAP) {
            // refine transformation matrix by loop closure detection
            if (m_isDetectedLoop) {
                updatedTransformArrWorld = m_loopTransform * T_ARr_SolAR;
                LOG_INFO("New transform matrix after loop detection:\n{}", updatedTransformArrWorld.matrix());
                m_isDetectedLoop = false;
            }
            // drift correction
            else {
                Transform3Df driftTransform = T_ARr_SolAR * m_lastTransform.inverse();
                if (!driftTransform.isApprox(Transform3Df::Identity()) && (m_status != TRACKING_LOST)){
                    m_isDetectedDrift = true;
                    m_dropBufferDriftTransform.push(driftTransform);
                }
            }
        }

        // Correct pose to the world coordinate system
        Transform3Df poseCorrected = T_ARr_SolAR * posesArr[0];

        // update status
        status = m_status;

        // update last transform
        m_lastTransform = updatedTransformArrWorld;

        // computation finished, convert updatedTransform from T_ARr2SolAR to T_ARr2World 
        updatedTransformArrWorld = T_SolAR_World* updatedTransformArrWorld;

		// Send image and corrected pose to process
		m_nbImageRequest++;
        m_dropBufferCamImagePoseCapture.push({ images[0], poseCorrected, fixedPose });

        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMultiProcessing::getDataForVisualization(std::vector<SRef<CloudPoint>> & outputPointClouds,
                                                std::vector<Transform3Df> & keyframePoses) const
    {
        if (m_status != MappingStatus::BOOTSTRAP) {
			LOG_DEBUG("PipelineMappingMultiProcessing::getDataForVisualization");
            std::unique_lock<std::mutex> lock(m_mutexMapData);
            outputPointClouds = m_allPointClouds;
            keyframePoses = m_allKeyframePoses;
            return FrameworkReturnCode::_SUCCESS;
        }
        else
            return FrameworkReturnCode::_ERROR_;
    }

// Private methods

    void PipelineMappingMultiProcessing::featureExtraction()
    {
        try {

            CaptureDropBufferElement imagePose;
            if (!m_started || !m_dropBufferCamImagePoseCapture.tryPop(imagePose)) {
                xpcf::DelegateTask::yield();
                return;
            }
            Timer clock;
            SRef<Image> image = imagePose.image;
            Transform3Df pose = imagePose.pose;
            std::vector<Keypoint> keypoints, undistortedKeypoints;
            SRef<DescriptorBuffer> descriptors;
            if (m_descriptorExtractor->extract(image, keypoints, descriptors) == FrameworkReturnCode::_SUCCESS) {
                m_undistortKeypoints->undistort(keypoints, m_cameraParams, undistortedKeypoints);
                SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_cameraParamsID, pose);
                frame->setFixedPose(imagePose.fixedPose);
                if (m_status != MappingStatus::BOOTSTRAP) {
                    m_nbFrameToUpdate++;
                    m_dropBufferFrame.push(frame);
                }
                else
                    m_dropBufferFrameBootstrap.push(frame);
            }
            m_nbExtractionProcess++;
            LOG_DEBUG("PipelineMappingMultiProcessing::featureExtraction elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in featureExtraction method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::correctPoseAndBootstrap()
    {
        try {

            SRef<Frame> frame;

            // Try to get next (image, pose) if bootstrap is not finished
            if (!m_started || (m_status != MappingStatus::BOOTSTRAP) || !m_dropBufferFrameBootstrap.tryPop(frame)) {
                xpcf::DelegateTask::yield();
                return;
            }
            Timer clock;
            LOG_DEBUG("PipelineMappingMultiProcessing::correctPoseAndBootstrap: new image to process");

            if (m_relocPipeline) {
                // try to get init map from reloc service
                LOG_DEBUG("Try get map of relocalization service");
                SRef<Map> map;
                if (m_relocPipeline->getMapRequest(map) == FrameworkReturnCode::_SUCCESS) {
                    m_mapManager->setMap(map);
                    // add current camera parameters to the new map
                    SRef<CameraParameters> camParams = xpcf::utils::make_shared<CameraParameters>(m_cameraParams);
                    m_mapManager->addCameraParameters(camParams);
                    m_cameraParamsID = camParams->id;

                    SRef<Keyframe> keyframe;
                    std::vector<uint32_t> retKeyframesId;
                    // find the keyframe most similar to frame and set it as reference keyframe 
                    if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
                        LOG_DEBUG("Successful relocalization. Update reference keyframe id: {}", retKeyframesId[0]);
                        if (m_keyframesManager->getKeyframe(retKeyframesId[0], keyframe) == FrameworkReturnCode::_SUCCESS) {
                            m_lastKeyframeId = retKeyframesId[0];
                            m_curKeyframeId = retKeyframesId[0];
                            m_tracking->setNewKeyframe(keyframe);
                        }
                        else {
                            LOG_ERROR("Get keyframe failed");
                            return;
                        }
                    }
                    else {
                        LOG_ERROR("Relocalization failed");
                        return;
                    }
                    LOG_INFO("Number of initial keyframes: {}", m_keyframesManager->getNbKeyframes());
                    LOG_INFO("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());
                    m_status = MappingStatus::MAPPING;
                    getMapData();
                    timerPipeline.restart();
                    return;
                }
                else {
                    LOG_DEBUG("Cannot get map from relocalization service");
                }
            }

            // do bootstrap
            SRef<Image> view;
            if (m_bootstrapper->process(frame, view) == FrameworkReturnCode::_SUCCESS) {
                LOG_DEBUG("Bootstrap finished: apply bundle adjustement");
                m_bundler->bundleAdjustment();
                SRef<Keyframe> keyframe2;
                m_keyframesManager->getKeyframe(1, keyframe2);
                m_lastKeyframeId = 1;
                m_curKeyframeId = 1;
                m_tracking->setNewKeyframe(keyframe2);
                LOG_INFO("Number of initial keyframes: {}", m_keyframesManager->getNbKeyframes());
                LOG_INFO("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());
                m_status = MappingStatus::MAPPING;
                getMapData();
                timerPipeline.restart();
            }

            LOG_DEBUG("PipelineMappingMultiProcessing::correctPoseAndBootstrap elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in correctPoseAndBootstrap method: {}", e.what());
            return;
        }
	}

    void PipelineMappingMultiProcessing::updateVisibility()
    {
        try {

            SRef<Frame> frame;

            // Frame to process
            if (!m_started || !m_dropBufferFrame.tryPop(frame)) {
                xpcf::DelegateTask::yield();
                return;
            }
            Timer clock;
            m_nbUpdateProcess++;
            // update visibility for the current frame
            SRef<Image> displayImage;
            if (m_tracking->process(frame, displayImage) != FrameworkReturnCode::_SUCCESS){
                LOG_INFO("PipelineMappingMultiProcessing::updateVisibility Tracking lost");
                LOG_DEBUG("PipelineMappingMultiProcessing::updateVisibility elapsed time = {} ms", clock.elapsed());
                m_nbTrackingLost ++;
                if (m_nbTrackingLost >= NB_SUCCESSIVE_TRACKING_LOST)
                    m_status = MappingStatus::TRACKING_LOST;
                m_lastKeyframeId = m_curKeyframeId;
                return;
            }
            else {
                m_status = m_status == MappingStatus::LOOP_CLOSURE ? MappingStatus::LOOP_CLOSURE : MappingStatus::MAPPING;
                m_nbTrackingLost = 0;
            }
            LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());

            // once groundtruth frame is received, wait for the following keyframe and set keyframe fixedPose to true
            // currently, for each received GT frame, we will wait for the following keyframe to see if GT can be used
            if (!m_isGTPoseReady) {
                if (frame->isFixedPose()) {
                    m_isGTPoseReady = true;
                    m_idProcessGTReceived = m_nbUpdateProcess;
                }
            }

            // send frame to mapping task
            if (m_isMappingIdle && m_isLoopIdle && m_tracking->checkNeedNewKeyframe()) {
                if (m_isGTPoseReady) {
                    // drift may be important if too much time passed between frame_GT and keyframe
                    if (m_nbUpdateProcess - m_idProcessGTReceived > NB_FRAMES_GT_ALIVE) {
                        m_isGTPoseReady = false;
                        LOG_ERROR("Error while injecting the ground truth pose, the GT pose is not used since too many frames passed between the frame and the following keyframe");
                        return;
                    }
                    // the keyframe closest to the groundtruth frame is set to be GT keyframe
                    frame->setFixedPose(true);
                    // once the groundtruth is used, waits for the next groundtruth frame
                    m_isGTPoseReady = false;
                }
                m_nbFrameToMapping++;
                m_dropBufferAddKeyframe.push(frame);
            }

            LOG_DEBUG("PipelineMappingMultiProcessing::updateVisibility elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in updateVisibility method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::mapping()
    {
        try {

            SRef<Frame> frame;

            if (!m_started || !m_dropBufferAddKeyframe.tryPop(frame) || !m_isLoopIdle) {
                xpcf::DelegateTask::yield();
                return;
            }

            m_isMappingIdle = false;
            std::unique_lock<std::mutex> lock(m_mutexMapping);
            Timer clock;
            m_nbMappingProcess++;
            SRef<Keyframe> keyframe;
            if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
                m_curKeyframeId = keyframe->getId();
                LOG_DEBUG("New keyframe id: {}", keyframe->getId());
                m_keyframeIds.insert(keyframe->getId());
                // Local bundle adjustment
                std::vector<uint32_t> bestIdx;
                m_covisibilityGraphManager->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx, NB_LOCALKEYFRAMES);
                bestIdx.push_back(keyframe->getId());
                LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
                double bundleReprojError = m_bundler->bundleAdjustment(bestIdx);
                LOG_DEBUG("Local bundle adjustment error: {}", bundleReprojError);
                // map pruning
                std::vector<SRef<CloudPoint>> localMap;
                m_mapManager->getLocalPointCloud(keyframe, m_minWeightNeighbor, localMap);
                int nbRemovedCP = m_mapManager->pointCloudPruning(localMap);
                std::vector<SRef<Keyframe>> localKeyframes;
                bestIdx.pop_back();
                m_keyframesManager->getKeyframes(bestIdx, localKeyframes);
                int nbRemovedKf = m_mapManager->keyframePruning(localKeyframes);
                LOG_DEBUG("Nb of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
                m_countNewKeyframes++;
                m_dropBufferNewKeyframeLoop.push(keyframe);
                // send new keyframe to tracking
                m_tracking->setNewKeyframe(keyframe);
                // update map data
                getMapData();
            }
            else
                LOG_DEBUG("Mapping fails");
            m_isMappingIdle = true;
            lock.unlock();
            LOG_DEBUG("PipelineMappingMultiProcessing::mapping elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in mapping method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::loopClosure()
    {
        try {

            SRef<Keyframe> lastKeyframe;

            if (!m_started || (m_countNewKeyframes < NB_NEWKEYFRAMES_LOOP) || !m_dropBufferNewKeyframeLoop.tryPop(lastKeyframe)) {
                xpcf::DelegateTask::yield();
                return;
            }
            Timer clock;
            SRef<Keyframe> detectedLoopKeyframe;
            Transform3Df sim3Transform;
            std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
            if (m_loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
                m_status = MappingStatus::LOOP_CLOSURE;
                // stop mapping process
                m_isLoopIdle = false;
                std::unique_lock<std::mutex> lock(m_mutexMapping);
                // detected loop keyframe
                LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
                LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
                LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
                // performs loop correction
                Transform3Df keyframeOldPose = lastKeyframe->getPose();
                m_loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
                // loop optimization
                m_globalBundler->bundleAdjustment();
                // map pruning
                m_mapManager->pointCloudPruning();
                m_mapManager->keyframePruning();
                m_countNewKeyframes = 0;
                // update pose correction
                m_loopTransform = lastKeyframe->getPose() * keyframeOldPose.inverse();
                LOG_INFO("Loop correction transform: \n{}", m_loopTransform.matrix());
                // update map data
                getMapData();
                // free mapping
                m_isLoopIdle = true;
                m_isDetectedLoop = true;
                m_status = MappingStatus::MAPPING;
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                lock.unlock();
            }

            LOG_DEBUG("PipelineMappingMultiProcessing::loopClosure elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in loopClosure method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::globalBundleAdjustment()
    {
        try {

            std::unique_lock<std::mutex> lock(m_mutexMapping);
            Timer clock;
            // Global bundle adjustment
            m_globalBundler->bundleAdjustment();
            LOG_INFO("Global BA done");
            // Map pruning
            if (m_mapManager->visibilityPruning() != FrameworkReturnCode::_SUCCESS) {
               LOG_WARNING("Visibility pruning did not succeed, may lead to reduced reloc accuracy from this map");
            }
            else {
               LOG_INFO("Visibilities of keyframes and cloud points pruned");
            }
            int nbCpPruning = m_mapManager->pointCloudPruning();
            LOG_INFO("Nb of pruning cloud points: {}", nbCpPruning);
            int nbKfPruning = m_mapManager->keyframePruning();
            LOG_INFO("Nb of pruning keyframes: {}", nbKfPruning);
            LOG_INFO("Nb of keyframes / cloud points: {} / {}",
                     m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

            if (m_boWFeatureFromMatchedDescriptors > 0) {
                // recompute BoW features using only useful descriptors 
                m_keyframeRetriever->resetKeyframeRetrieval();
                for (auto id : m_keyframeIds) {
                    SRef<Keyframe> keyframe;
                    if (m_keyframesManager->getKeyframe(id, keyframe) == FrameworkReturnCode::_SUCCESS) // may return fail because of keyframe pruning
                        m_keyframeRetriever->addKeyframe(keyframe, true);
                }
                LOG_INFO("Recompute BoW features from matched descriptors for keyframes");
            }
            lock.unlock();

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

            LOG_DEBUG("PipelineMappingMultiProcessing::globalBundleAdjustment elapsed time = {} ms", clock.elapsed());

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in globalBundleAdjustment method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::driftCorrection()
    {
        try {

            Transform3Df driftTransform;
            if (!m_isDetectedDrift || !m_dropBufferDriftTransform.tryPop(driftTransform)) {
                xpcf::DelegateTask::yield();
                return;
            }
            std::unique_lock<std::mutex> lock(m_mutexMapping);
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
            for (uint32_t i = m_lastKeyframeId + 1; i < m_curKeyframeId; ++i)
                if (m_keyframesManager->isExistKeyframe(i))
                    loopKfId.push_back(i);
            LOG_INFO("Nb of keyframes to bundle: {}", loopKfId.size());
            double errorBundle(0.0);
            if (loopKfId.size() > 0)
                errorBundle = m_globalBundler->bundleAdjustment(loopKfId);
            // update last keyframe id
            m_lastKeyframeId = m_curKeyframeId;
            m_isDetectedDrift = false;
            LOG_DEBUG("Drift correction done with error: {}", errorBundle);

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised in driftCorrection method: {}", e.what());
            return;
        }
    }

    void PipelineMappingMultiProcessing::getMapData()
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

}
}
} // SolAR::PIPELINES::MAPPING
