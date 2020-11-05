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

#include "SolARMappingPipelineProcessing.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace MAPPINGPIPELINE {

    SolARMappingPipelineProcessing::SolARMappingPipelineProcessing():ConfigurableBase(xpcf::toUUID<SolARMappingPipelineProcessing>())
    {
        declareInterface<api::pipeline::IMappingPipeline>(this);

        LOG_DEBUG("SolARMappingPipelineProcessing constructor");
    }


    SolARMappingPipelineProcessing::~SolARMappingPipelineProcessing() {

        LOG_DEBUG("SolARMappingPipelineProcessing destructor");

        delete m_mappingTask;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::init(SRef<xpcf::IComponentManager> componentManager) {

        try {

            LOG_DEBUG("SolARMappingPipelineProcessing::init");

            LOG_INFO("Load configuration file");

            if(componentManager->load("xpcf_SolARMappingPipeline_registry.xml")!=org::bcom::xpcf::_SUCCESS)
            {
                LOG_ERROR("Failed to load the configuration file xpcf_SolARMappingPipeline_registry.xml");
                return FrameworkReturnCode::_ERROR_;
            }

            // Declare and create components
            m_fiducialMarkerPoseEstimator = componentManager->resolve<solver::pose::IFiducialMarkerPose>();
            m_bootstrapper = componentManager->resolve<slam::IBootstrapper>();
            m_bundler = componentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
            m_globalBundler = componentManager->resolve<api::solver::map::IBundler>();
            m_mapper = componentManager->resolve<api::solver::map::IMapper>();
            m_mapping = componentManager->resolve<api::slam::IMapping>();
            m_keyframesManager = componentManager->resolve<api::storage::IKeyframesManager>();
            m_pointCloudManager = componentManager->resolve<api::storage::IPointCloudManager>();

            LOG_DEBUG("All components created");

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members
            m_cameraParams.resolution.width = 0;
            m_cameraParams.resolution.height = 0;
            m_fiducialMarker.setWidth(0);
            m_fiducialMarker.setHeight(0);
            m_countNewKeyframes = 0;

            m_isBootstrapFinished = false;
            m_isFoundTransform = false;
            Transform3Df T_M_W = Transform3Df::Identity();
            m_minWeightNeighbor = 0;
            m_framePoses.clear();

            // Get properties
            m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

            // Reset drop buffers
            m_inputDropBufferImagePose.empty();

            LOG_DEBUG("Set the mapping function for asynchronous task");

            // Mapping processing function
            if (m_mappingTask == nullptr) {
                auto fnMappingProcessing = [&]() {
                    processMapping();
                };

                m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
            }

            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());

            return FrameworkReturnCode::_SUCCESS;
        }
        catch (xpcf::Exception e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::setCameraParameters(const CameraParameters & cameraParams) {

        LOG_DEBUG("SolARMappingPipelineProcessing::setCameraParameters");

        m_cameraParams = cameraParams;

        m_fiducialMarkerPoseEstimator->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_bootstrapper->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_mapping->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);

        // get properties
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();


        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
                  m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::setObjectToTrack(const Trackable & trackableObject) {

        LOG_DEBUG("SolARMappingPipelineProcessing::setObjectToTrack");

        if (trackableObject.getType() == FIDUCIAL_MARKER) {

            const FiducialMarker *fiducialMarker = dynamic_cast<const FiducialMarker *>(&trackableObject);

            m_fiducialMarker = *fiducialMarker;

            LOG_DEBUG("Fiducial marker url / width / height = {} / {} / {}",
                     m_fiducialMarker.getURL(), m_fiducialMarker.getWidth(), m_fiducialMarker.getHeight());
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::correctPoseAndBootstrap
                                            (const SRef<Image> & image, const Transform3Df & pose) {

        LOG_DEBUG("SolARMappingPipelineProcessing::correctPoseAndBootstrap");

        // Find T_W_M
        if (!m_isFoundTransform) {
            Transform3Df T_M_C;
            if (m_fiducialMarkerPoseEstimator->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
                m_T_M_W = T_M_C * pose.inverse();
                m_isFoundTransform = true;
                LOG_DEBUG("3D transformation found");
            }
            else {
                LOG_DEBUG("3D transformation not found");
                return FrameworkReturnCode::_STOP;
            }
        }

        // Correct pose
        Transform3Df pose2 = m_T_M_W * pose;

        LOG_DEBUG("3D transformation found: do bootstrap");

        SRef<Image> view;
        if (m_bootstrapper->process(image, view, pose2) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Bootstrap finished: apply bundle adjustement");

            double bundleReprojError = m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
            m_isBootstrapFinished = true;

            // Prepare mapping process
            m_keyframesManager->getKeyframe(1, m_refKeyframe);
            updateLocalMap(m_refKeyframe);
        }
        else {
            LOG_DEBUG("Boostrap not finished");

            return FrameworkReturnCode::_STOP;
        }
    }

    bool SolARMappingPipelineProcessing::isBootstrapFinished() const {

        LOG_DEBUG("SolARMappingPipelineProcessing::isBootstrapFinished");

        return m_isBootstrapFinished;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::start() {

        LOG_DEBUG("SolARMappingPipelineProcessing::start");

        // Check members initialization
        if ((m_cameraParams.resolution.width > 0) && (m_cameraParams.resolution.width > 0)
                && (m_fiducialMarker.getWidth() > 0) && (m_fiducialMarker.getHeight() > 0)) {

            if (m_isBootstrapFinished) {

                LOG_DEBUG("Start mapping processing task");
                m_mappingTask->start();

                return FrameworkReturnCode::_SUCCESS;
            }
            else {
                LOG_DEBUG("Bootstrap step is not finished");
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_DEBUG("Camera parameters and/or fiducial marker description not set");
            return FrameworkReturnCode::_ERROR_;
        }
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::stop() {

        LOG_DEBUG("SolARMappingPipelineProcessing::stop");

        LOG_DEBUG("Bundle adjustment and map pruning");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
        // Map pruning
        m_mapper->pruning();

        LOG_INFO("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        LOG_INFO("Update global map");
        m_mapper->saveToFile();

        LOG_DEBUG("Stop mapping processing task");
        m_mappingTask->stop();

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::mappingProcessRequest(const SRef<Image> & image, const Transform3Df & pose) {

        LOG_DEBUG("SolARMappingPipelineProcessing::mappingProcessRequest");

        // Add pair (image, pose) to input drop buffer
        m_inputDropBufferImagePose.push(std::make_pair(image, pose));
        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::getDataForVisualization(std::vector<CloudPoint> & outputPointClouds,
                                                std::vector<SRef<Transform3Df>> & keyframePoses) {

        LOG_DEBUG("SolARMappingPipelineProcessing::getDataForVisualization");

        return FrameworkReturnCode::_SUCCESS;
    }

    void SolARMappingPipelineProcessing::updateLocalMap(const SRef<Keyframe> & keyframe) {
        m_localMap.clear();
        m_mapper->getLocalPointCloud(keyframe, m_minWeightNeighbor, m_localMap);
    }

    void SolARMappingPipelineProcessing::processMapping() {

        LOG_DEBUG("SolARMappingPipelineProcessing::processMapping");

        std::pair<SRef<Image>, Transform3Df> image_pose_pair;

        while (true) {
            LOG_DEBUG("Try to get (image, pose) pair from input buffer");

            if (m_inputDropBufferImagePose.tryPop(image_pose_pair)) {
                LOG_DEBUG("Got an (image, pose) pair to process");

                // Mapping processing here
            }
            else {
                LOG_DEBUG("No (image, pose) pair to process");
            }
        }
    }
}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE
