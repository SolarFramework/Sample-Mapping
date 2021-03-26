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


// Public methods

    PipelineMappingMonoProcessing::PipelineMappingMonoProcessing():ConfigurableBase(xpcf::toUUID<PipelineMappingMonoProcessing>())
    {
        LOG_DEBUG("PipelineMappingMonoProcessing constructor");

        try {
            declareInterface<api::pipeline::IMappingPipeline>(this);

            LOG_DEBUG("Components injection declaration");
            declareInjectable<api::solver::pose::ITrackablePose>(m_fiducialMarkerPoseEstimator);
            declareInjectable<api::slam::IBootstrapper>(m_bootstrapper);
            declareInjectable<api::solver::map::IBundler>(m_bundler, "BundleFixedKeyframes");
            declareInjectable<api::solver::map::IBundler>(m_globalBundler);
            declareInjectable<api::solver::map::IMapper>(m_mapper);
            declareInjectable<api::slam::IMapping>(m_mapping);
            declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
            declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
            declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
            declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
            declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
            declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
            declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
            declareInjectable<api::geom::IProject>(m_projector);
            declareInjectable<api::storage::ICovisibilityGraph>(m_covisibilityGraph);
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);

            LOG_DEBUG("All component injections declared");

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members
            m_cameraParams.resolution.width = 0;
            m_cameraParams.resolution.height = 0;
            m_trackable = nullptr;
            m_countNewKeyframes = 0;

            m_dataToStore = false;
            m_isFoundTransform = false;
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
        m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
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

        m_fiducialMarkerPoseEstimator->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_bootstrapper->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_mapping->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_projector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopDetector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);
        m_loopCorrector->setCameraParameters(m_cameraParams.intrinsic, m_cameraParams.distortion);

        LOG_DEBUG("Camera width / height / distortion = {} / {} / {}",
                  m_cameraParams.resolution.width, m_cameraParams.resolution.height, m_cameraParams.distortion);

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::setObjectToTrack(const SRef<Trackable> trackableObject) {

        LOG_DEBUG("PipelineMappingMonoProcessing::setObjectToTrack");
        m_trackable = trackableObject;
        return (m_fiducialMarkerPoseEstimator->setTrackable(trackableObject));
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::start() {

        LOG_DEBUG("PipelineMappingMonoProcessing::start");

        // Check members initialization
        if ((m_cameraParams.resolution.width > 0) && (m_cameraParams.resolution.height > 0) && (m_trackable != nullptr)) {

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

        LOG_DEBUG("Stop mapping processing task");
        m_mappingTask->stop();

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode PipelineMappingMonoProcessing::mappingProcessRequest(const SRef<Image> image, const Transform3Df & pose) {

        LOG_DEBUG("PipelineMappingMonoProcessing::mappingProcessRequest");

        // Correct pose
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
                return false;
            }
        }

        LOG_DEBUG("3D transformation found: do bootstrap");

        SRef<Image> view;
        if (m_bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Bootstrap finished: apply bundle adjustement");

            m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);

            // Prepare mapping process
            m_keyframesManager->getKeyframe(1, m_refKeyframe);
            updateLocalMap(m_refKeyframe);

            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());

            setBootstrapSatus(true);
        }
        else {
            LOG_DEBUG("Boostrap not finished");

            return false;
        }

        return true;
    }

    void PipelineMappingMonoProcessing::updateLocalMap(const SRef<Keyframe> & keyframe) {
        m_localMap.clear();
        m_mapper->getLocalPointCloud(keyframe, m_minWeightNeighbor, m_localMap);

        // New data to store
        m_dataToStore = true;
    }

    void PipelineMappingMonoProcessing::globalBundleAdjustment() {
        LOG_DEBUG("PipelineMappingMonoProcessing::globalBundleAdjustment");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
        // Map pruning
        m_mapper->pruning();

        LOG_DEBUG("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        LOG_DEBUG("Update global map");
        m_mapper->saveToFile();
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

                // Mapping

                LOG_DEBUG("Reference keyframe id: {}", m_refKeyframe->getId());

                // feature extraction image
                std::vector<Keypoint> keypoints;
                m_keypointsDetector->detect(image, keypoints);
                LOG_DEBUG("Keypoints size = {}", keypoints.size());

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
                    xpcf::DelegateTask::yield();
                std::set<uint32_t> idxCPSeen;					// Index of CP seen
                for (const auto & itCorr : corres2D3D) {
                    idxCPSeen.insert(itCorr.second->getId());
                }

                // Find map visibilities of the current frame
                std::map<uint32_t, uint32_t> newMapVisibility;	// map visibilities
                std::vector< Point2Df > refCPSeenProj;
                m_projector->project(pts3d, refCPSeenProj, pose); // Project ref cloud point seen to current frame to define inliers/outliers
                std::vector<Point2Df> pts2d_inliers, pts2d_outliers;
                for (unsigned long i = 0; i < refCPSeenProj.size(); ++i) {
                    float dis = (pts2d[i] - refCPSeenProj[i]).norm();
                    LOG_DEBUG("dis / m_reprojErrorThreshold: {} / {}", dis, m_reprojErrorThreshold);
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

				// find unseen local map from current frame                
                std::vector<SRef<CloudPoint>> localMapUnseen;
                for (auto &it_cp : m_localMap) {
                    if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
                        localMapUnseen.push_back(it_cp);
                }

				// Find more visibilities by projecting the rest of local map
				if (localMapUnseen.size() > 0) {
					//  projection points
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
				}

                // Add visibilities to current frame
                frame->addVisibilities(newMapVisibility);
                LOG_DEBUG("Number of tracked points: {}", newMapVisibility.size());
                if (newMapVisibility.size() < m_minWeightNeighbor)
                    xpcf::DelegateTask::yield();;

                // mapping
                SRef<Keyframe> keyframe;
                if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
                    LOG_DEBUG("New keyframe id: {}", keyframe->getId());
                    // Local bundle adjustment
                    std::vector<uint32_t> bestIdx;
                    m_covisibilityGraph->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx);
                    bestIdx.push_back(keyframe->getId());
                    LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
                    m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion, bestIdx);
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

                // Data to store ?
                if (m_dataToStore) {
                    m_dataToStore = false;

                    // Bundle adjustment, map pruning and global map udate
                    globalBundleAdjustment();
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
