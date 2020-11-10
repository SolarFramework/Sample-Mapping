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

            LOG_DEBUG("Load configuration file");

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
            m_keypointsDetector = componentManager->resolve<api::features::IKeypointDetector>();
            m_descriptorExtractor = componentManager->resolve<api::features::IDescriptorsExtractor>();
            m_matcher = componentManager->resolve<api::features::IDescriptorMatcher>();
            m_matchesFilter = componentManager->resolve<api::features::IMatchesFilter>();
            m_corr2D3DFinder = componentManager->resolve<api::solver::pose::I2D3DCorrespondencesFinder>();
            m_projector = componentManager->resolve<api::geom::IProject>();
            m_covisibilityGraph = componentManager->resolve<api::ICovisibilityGraph>();
            m_loopDetector = componentManager->resolve<api::loop::ILoopClosureDetector>();
            m_loopCorrector = componentManager->resolve<api::loop::ILoopCorrector>();

            LOG_DEBUG("All components created");

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members
            m_cameraParams.resolution.width = 0;
            m_cameraParams.resolution.height = 0;
            m_fiducialMarker.setWidth(0);
            m_fiducialMarker.setHeight(0);
            m_countNewKeyframes = 0;

            m_askedToStop = false;
            m_isBootstrapFinished = false;
            m_isFoundTransform = false;
            Transform3Df T_M_W = Transform3Df::Identity();
            m_minWeightNeighbor = 0;

            // Get properties
            m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

            // Reset drop buffers
            m_inputSharedFifoImagePose.empty();

            LOG_DEBUG("Set the mapping function for asynchronous task");

            // Mapping processing function
            if (m_mappingTask == nullptr) {
                auto fnMappingProcessing = [&]() {
                    processMapping();
                };

                m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
            }

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
        m_projector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopDetector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopCorrector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);

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

            return FrameworkReturnCode::_SUCCESS;
        }
        else {
            return FrameworkReturnCode::_ERROR_;
        }
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

            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());
        }
        else {
            LOG_DEBUG("Boostrap not finished");

            return FrameworkReturnCode::_STOP;
        }

        return FrameworkReturnCode::_SUCCESS;
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

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::stop() {

        LOG_DEBUG("SolARMappingPipelineProcessing::stop");

        m_askedToStop = true;

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::mappingProcessRequest(const SRef<Image> & image, const Transform3Df & pose) {

        LOG_DEBUG("SolARMappingPipelineProcessing::mappingProcessRequest");

        // Add pair (image, pose) to input drop buffer
        m_inputSharedFifoImagePose.push(std::make_pair(image, pose));
        LOG_DEBUG("New pair of (image, pose) stored for mapping processing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::getDataForVisualization(std::vector<CloudPoint> & outputPointClouds,
                                                std::vector<SRef<Transform3Df>> & keyframePoses) {

        LOG_DEBUG("SolARMappingPipelineProcessing::getDataForVisualization");

        FrameworkReturnCode result = FrameworkReturnCode::_ERROR_;

        // Get data from mapper
        SRef<IPointCloudManager> pointCloudManager;
        SRef<IKeyframesManager> keyframesManager;

        // Get managers
        if (m_mapper->getPointCloudManager(pointCloudManager) == FrameworkReturnCode::_SUCCESS) {
            if (m_mapper->getKeyframesManager(keyframesManager) == FrameworkReturnCode::_SUCCESS) {
/*
                // Get point clouds
                if (pointCloudManager->getAllPoints(outputPointClouds) == FrameworkReturnCode::_SUCCESS) {
                    // Get key frames
                    result = keyframesManager->getAllKeyframes(keyframePoses);
                }
*/
            }
        }

        return result;
    }

    void SolARMappingPipelineProcessing::updateLocalMap(const SRef<Keyframe> & keyframe) {
        m_localMap.clear();
        m_mapper->getLocalPointCloud(keyframe, m_minWeightNeighbor, m_localMap);
    }

    void SolARMappingPipelineProcessing::globalBundleAdjustment() {
        LOG_DEBUG("SolARMappingPipelineProcessing::globalBundleAdjustment");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
        // Map pruning
        m_mapper->pruning();

        LOG_INFO("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        LOG_DEBUG("Update global map");
        m_mapper->saveToFile();
    }

    void SolARMappingPipelineProcessing::processMapping() {

        LOG_DEBUG("SolARMappingPipelineProcessing::processMapping");

        std::pair<SRef<Image>, Transform3Df> image_pose_pair;

        while (true) {
            LOG_DEBUG("Try to get (image, pose) pair from input buffer");

            LOG_DEBUG("Reference keyframe id: {}", m_refKeyframe->getId());

            if (!m_inputSharedFifoImagePose.empty()) {
                LOG_DEBUG("Got an (image, pose) pair to process");

                m_inputSharedFifoImagePose.pop(image_pose_pair);

                SRef<Image> image = image_pose_pair.first;
                Transform3Df pose = image_pose_pair.second;

                // Correct pose
                pose = m_T_M_W * pose;

                // feature extraction image
                std::vector<Keypoint> keypoints;
                m_keypointsDetector->detect(image, keypoints);
                SRef<DescriptorBuffer> descriptors;
                m_descriptorExtractor->extract(image, keypoints, descriptors);
                SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, m_refKeyframe, pose);

                // feature matching to reference keyframe
                std::vector<DescriptorMatch> matches;
                m_matcher->match(m_refKeyframe->getDescriptors(), descriptors, matches);
                m_matchesFilter->filter(matches, matches, m_refKeyframe->getKeypoints(), keypoints);
                float maxMatchDistance = -FLT_MAX;
                for (const auto &it : matches) {
                    float score = it.getMatchingScore();
                    if (score > maxMatchDistance)
                        maxMatchDistance = score;
                }

                LOG_DEBUG("maxMatchDistance = {}", maxMatchDistance);

                // find 2D-3D point correspondences
                std::vector<Point2Df> pts2d;
                std::vector<Point3Df> pts3d;
                std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
                std::vector<DescriptorMatch> foundMatches;
                std::vector<DescriptorMatch> remainingMatches;
                m_corr2D3DFinder->find(m_refKeyframe, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);
                if (corres2D3D.size() == 0)
                    break;
                std::set<uint32_t> idxCPSeen;					// Index of CP seen
                for (const auto & itCorr : corres2D3D) {
                    idxCPSeen.insert(itCorr.second->getId());
                }

                // Find map visibilities of the current frame
                std::map<uint32_t, uint32_t> newMapVisibility;	// map visibilities
                std::vector< Point2Df > refCPSeenProj;
                m_projector->project(pts3d, refCPSeenProj, pose); // Project ref cloud point seen to current frame to define inliers/outliers
                std::vector<Point2Df> pts2d_inliers, pts2d_outliers;
                for (int i = 0; i < refCPSeenProj.size(); ++i) {
                    float dis = (pts2d[i] - refCPSeenProj[i]).norm();
                    if (dis < m_reprojErrorThreshold) {
                        corres2D3D[i].second->updateConfidence(true);
                        newMapVisibility[corres2D3D[i].first] = corres2D3D[i].second->getId();
                        pts2d_inliers.push_back(pts2d[i]);
                    }
                    else {
                        corres2D3D[i].second->updateConfidence(false);
                        pts2d_outliers.push_back(pts2d[i]);
                    }
                }
                LOG_DEBUG("Number of inliers / outliers: {} / {}", pts2d_inliers.size(), pts2d_outliers.size());

                // Find more visibilities by projecting the rest of local map
                //  projection points
                std::vector<SRef<CloudPoint>> localMapUnseen;
                for (auto &it_cp : m_localMap)
                    if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
                        localMapUnseen.push_back(it_cp);
                std::vector< Point2Df > projected2DPts;
                m_projector->project(localMapUnseen, projected2DPts, pose);

                // find more inlier matches
                std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
                for (auto &it_cp : localMapUnseen) {
                    desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
                }
                std::vector<DescriptorMatch> allMatches;
                m_matcher->matchInRegion(projected2DPts, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance * 1.5);

                // find visibility of new frame
                for (auto &it_match : allMatches) {
                    int idx_2d = it_match.getIndexInDescriptorB();
                    int idx_3d = it_match.getIndexInDescriptorA();
                    auto it2d = newMapVisibility.find(idx_2d);
                    if (it2d == newMapVisibility.end()) {
                        pts2d_inliers.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
                        newMapVisibility[idx_2d] = localMapUnseen[idx_3d]->getId();
                    }
                }

                // Add visibilities to current frame
                frame->addVisibilities(newMapVisibility);
                LOG_DEBUG("Number of tracked points: {}", newMapVisibility.size());
                if (newMapVisibility.size() < m_minWeightNeighbor)
                    break;

                // mapping
                SRef<Keyframe> keyframe;
                if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
                    LOG_DEBUG("New keyframe id: {}", keyframe->getId());
                    // Local bundle adjustment
                    std::vector<uint32_t> bestIdx;
                    m_covisibilityGraph->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx);
                    bestIdx.push_back(keyframe->getId());
                    LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
                    double bundleReprojError = m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion, bestIdx);
                    // map pruning
                    updateLocalMap(keyframe);
                    m_mapper->pruning(m_localMap);
                    // try to loop detection
                    m_countNewKeyframes++;
                    // loop closure
                    if (m_countNewKeyframes >= 10) {
                        SRef<Keyframe> detectedLoopKeyframe;
                        Transform3Df sim3Transform;
                        std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
                        if (m_loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
                            // detected loop keyframe
                            LOG_DEBUG("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
                            LOG_DEBUG("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
                            LOG_DEBUG("sim3Transform: \n{}", sim3Transform.matrix());
                            // performs loop correction
                            Transform3Df keyframeOldPose = keyframe->getPose();
                            m_loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
                            // loop optimization
                            m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
                            // map pruning
                            m_mapper->pruning();
                            m_countNewKeyframes = 0;
                            // update pose correction
                            Transform3Df transform = keyframe->getPose() * keyframeOldPose.inverse();
                            m_T_M_W = transform * m_T_M_W;
                        }
                    }
                }

                // update reference keyframe
                if (keyframe) {
                    m_refKeyframe = keyframe;
                    updateLocalMap(m_refKeyframe);
                }

                // get all keyframes and point cloud
                std::vector<Transform3Df> keyframePoses;
                std::vector<SRef<Keyframe>> allKeyframes;
                m_keyframesManager->getAllKeyframes(allKeyframes);
                for (auto const &it : allKeyframes)
                    keyframePoses.push_back(it->getPose());
                std::vector<SRef<CloudPoint>> pointCloud;
                m_pointCloudManager->getAllPoints(pointCloud);
            }
            else {
                LOG_DEBUG("***** No (image, pose) pair to process *****");

                // Asked to stop?
                if (m_askedToStop) {
                    // Bundle adjustment, map pruning and global map udate
                    globalBundleAdjustment();

                    LOG_DEBUG("Stop mapping processing task");
                    m_mappingTask->stop();
                }
            }
        }
    }
}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE
