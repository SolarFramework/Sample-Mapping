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

#include "SolARMappingPipelineMultiProcessing.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace MAPPINGPIPELINE {

#define NB_NEWKEYFRAMES_LOOP 10

// Public methods

    SolARMappingPipelineMultiProcessing::SolARMappingPipelineMultiProcessing():ConfigurableBase(xpcf::toUUID<SolARMappingPipelineMultiProcessing>())
    {
        LOG_DEBUG("SolARMappingPipelineMultiProcessing constructor");

        try {
            declareInterface<api::pipeline::IMappingPipeline>(this);

            LOG_DEBUG("Components injection declaration");

            declareInjectable<solver::pose::IFiducialMarkerPose>(m_fiducialMarkerPoseEstimator);
            declareInjectable<slam::IBootstrapper>(m_bootstrapper);
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
            declareInjectable<api::ICovisibilityGraph>(m_covisibilityGraph);
            declareInjectable<api::loop::ILoopClosureDetector>(m_loopDetector);
            declareInjectable<api::loop::ILoopCorrector>(m_loopCorrector);

            LOG_DEBUG("All component injections declared");

            LOG_DEBUG("Initialize instance attributes");

            // Initialize private members
            m_cameraParams.resolution.width = 0;
            m_cameraParams.resolution.height = 0;
            m_fiducialMarker = nullptr;
            m_countNewKeyframes = 0;

            m_T_M_W = Transform3Df::Identity();
            m_isFoundTransform = false;
            m_isStopMapping = false;
            m_minWeightNeighbor = 0;

            // Initial bootstrap status
            m_isBootstrapFinished = false;

            LOG_DEBUG("Set the mapping function for asynchronous task");

            // Bootstrap processing function
            if (m_bootstrapTask == nullptr) {
                auto fnBootstrapProcessing = [&]() {
                    correctPoseAndBootstrap();
                };

                m_bootstrapTask = new xpcf::DelegateTask(fnBootstrapProcessing);
            }

            // Keypoints detection processing function
            if (m_keypointsDetectionTask == nullptr) {
                auto fnKeypointsDetectionProcessing = [&]() {
                    keypointsDetection();
                };

                m_keypointsDetectionTask = new xpcf::DelegateTask(fnKeypointsDetectionProcessing);
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
        }
        catch (xpcf::Exception & e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }

    void SolARMappingPipelineMultiProcessing::onInjected() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::onInjected");

        // Get properties
        m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
        m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
    }

    SolARMappingPipelineMultiProcessing::~SolARMappingPipelineMultiProcessing() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing destructor");

        delete m_bootstrapTask;
        delete m_keypointsDetectionTask;
        delete m_featureExtractionTask;
        delete m_updateVisibilityTask;
        delete m_mappingTask;
        delete m_loopClosureTask;
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::init(SRef<xpcf::IComponentManager> componentManager) {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing");

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::setCameraParameters(const CameraParameters & cameraParams) {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::setCameraParameters");

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

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::setObjectToTrack(const SRef<Trackable> & trackableObject) {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::setObjectToTrack");

        if ((trackableObject != 0) && (trackableObject->getType() == FIDUCIAL_MARKER)) {

            m_fiducialMarker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackableObject);

            m_fiducialMarkerPoseEstimator->setMarker(m_fiducialMarker);

            LOG_DEBUG("Fiducial marker url / width / height = {} / {} / {}",
                     m_fiducialMarker->getURL(), m_fiducialMarker->getWidth(), m_fiducialMarker->getHeight());

            return FrameworkReturnCode::_SUCCESS;
        }
        else {
            return FrameworkReturnCode::_ERROR_;
        }
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::start() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::start");

        // Check members initialization
        if ((m_cameraParams.resolution.width > 0) && (m_cameraParams.resolution.height > 0)
                && (m_fiducialMarker != nullptr) && (m_fiducialMarker->getWidth() > 0) && (m_fiducialMarker->getHeight() > 0)) {

            LOG_DEBUG("Start processing tasks");

            m_bootstrapTask->start();
            m_keypointsDetectionTask->start();
            m_featureExtractionTask->start();
            m_updateVisibilityTask->start();
            m_mappingTask->start();
            m_loopClosureTask->start();
        }
        else {
            LOG_DEBUG("Camera parameters and/or fiducial marker description not set");
            return FrameworkReturnCode::_ERROR_;
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::stop() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::stop");

        if (isBootstrapFinished()){
            LOG_DEBUG("Bundle adjustment, map pruning and global map udate");
            globalBundleAdjustment();
        }

        LOG_DEBUG("Stop processing tasks");

        m_loopClosureTask->stop();
        m_mappingTask->stop();
        m_updateVisibilityTask->stop();
        m_featureExtractionTask->stop();
        m_keypointsDetectionTask->stop();
        m_bootstrapTask->stop();

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::mappingProcessRequest(const SRef<Image> & image, const Transform3Df & pose) {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::mappingProcessRequest");

        // Correct pose
        Transform3Df poseCorrected = m_T_M_W * pose;

        if (!isBootstrapFinished()){
            // Add pair (image, pose) to input drop buffer for bootstrap
            m_dropBufferCamImagePoseCaptureBootstrap.push(std::make_pair(image, poseCorrected));

            LOG_DEBUG("New pair of (image, pose) stored for bootstrap processing");
        }
        else {
            // Add pair (image, pose) to input drop buffer for mapping
            m_dropBufferCamImagePoseCapture.push(std::make_pair(image, poseCorrected));

            LOG_DEBUG("New pair of (image, pose) stored for mapping processing");
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineMultiProcessing::getDataForVisualization(std::vector<SRef<CloudPoint>> & outputPointClouds,
                                                std::vector<Transform3Df> & keyframePoses) const {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::getDataForVisualization");

        if (isBootstrapFinished()) {

            std::vector<SRef<Keyframe>> allKeyframes;
            keyframePoses.clear();

            LOG_DEBUG("Bootstrap finished");

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

    void SolARMappingPipelineMultiProcessing::correctPoseAndBootstrap () {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::correctPoseAndBootstrap = {}", isBootstrapFinished());

        std::pair<SRef<Image>, Transform3Df> imagePose;

        // Try to get next (image, pose) if bootstrap is not finished
        if (m_isBootstrapFinished ||
           !m_dropBufferCamImagePoseCaptureBootstrap.tryPop(imagePose)) {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<Image> image = imagePose.first;
        Transform3Df pose = imagePose.second;

        // find T_W_M
        if (!m_isFoundTransform) {
            Transform3Df T_M_C;
            if (m_fiducialMarkerPoseEstimator->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
                m_T_M_W = T_M_C * pose.inverse();
                m_isFoundTransform = true;
                LOG_DEBUG("Transformation found");
            }
            else {
                LOG_DEBUG("3D transformation not found");
            }

            return;
        }

        LOG_DEBUG("3D transformation found: do bootstrap");

        // do bootstrap
        SRef<Image> view;
        if (m_bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {

            LOG_DEBUG("Bootstrap finished: apply bundle adjustement");
            m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
            m_keyframesManager->getKeyframe(1, m_refKeyframe);
            updateLocalMap(m_refKeyframe);

            LOG_DEBUG("Number of initial point cloud: {}", m_pointCloudManager->getNbPoints());

            setBootstrapSatus(true);
        }
    }

    void SolARMappingPipelineMultiProcessing::keypointsDetection() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::keypointsDetection");

        std::pair<SRef<Image>, Transform3Df> imagePose;

        // Bootstrap finished?
        if (!isBootstrapFinished() ||
            !m_dropBufferCamImagePoseCapture.tryPop(imagePose)) {
            xpcf::DelegateTask::yield();
            return;
        }

        LOG_DEBUG("Reference keyframe id: {}", m_refKeyframe->getId());

        std::vector<Keypoint> keypoints;
        m_keypointsDetector->detect(imagePose.first, keypoints);
        if (keypoints.size() > 0) {
            m_dropBufferKeypoints.push(xpcf::utils::make_shared<Frame>(keypoints, nullptr, imagePose.first, imagePose.second));
        }
    }

    void SolARMappingPipelineMultiProcessing::featureExtraction() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::featureExtraction");

        SRef<Frame> frame;

        // Images to process ?
        if (!m_dropBufferKeypoints.tryPop(frame)) {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<DescriptorBuffer> descriptors;
        m_descriptorExtractor->extract(frame->getView(), frame->getKeypoints(), descriptors);
        frame->setDescriptors(descriptors);
        m_dropBufferFrameDescriptors.push(frame);
    }

    void SolARMappingPipelineMultiProcessing::updateVisibility() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::updateVisibility");

        SRef<Frame> frame;

        // Frame to process?
        if (!m_dropBufferFrameDescriptors.tryPop(frame)) {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<Keyframe> newKeyframe;

        if (m_dropBufferNewKeyframe.tryPop(newKeyframe))
        {
            LOG_DEBUG("Update new keyframe in update task");

            m_refKeyframe = newKeyframe;
            updateLocalMap(m_refKeyframe);

            SRef<Frame> tmpFrame;
            m_dropBufferAddKeyframe.tryPop(tmpFrame);
            m_isStopMapping = false;
        }
        frame->setReferenceKeyframe(m_refKeyframe);
        // feature matching to reference keyframe
        std::vector<DescriptorMatch> matches;
        m_matcher->match(m_refKeyframe->getDescriptors(), frame->getDescriptors(), matches);
        m_matchesFilter->filter(matches, matches, m_refKeyframe->getKeypoints(), frame->getKeypoints());
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
        if (corres2D3D.size() == 0) {
            return;
        }
        std::set<uint32_t> idxCPSeen;					// Index of CP seen
        for (const auto & itCorr : corres2D3D) {
            idxCPSeen.insert(itCorr.second->getId());
        }

        // Find map visibilities of the current frame
        std::map<uint32_t, uint32_t> newMapVisibility;	// map visibilities
        std::vector< Point2Df > refCPSeenProj;
        m_projector->project(pts3d, refCPSeenProj, frame->getPose()); // Project ref cloud point seen to current frame to define inliers/outliers
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
        
		// find unseen local map from current frame
        std::vector<SRef<CloudPoint>> localMapUnseen;
        for (auto &it_cp : m_localMap)
            if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
                localMapUnseen.push_back(it_cp);

		// Find more visibilities by projecting the rest of local map
		if (localMapUnseen.size() > 0) {
			//  projection points
			std::vector< Point2Df > projected2DPts;
			m_projector->project(localMapUnseen, projected2DPts, frame->getPose());
			// find more inlier matches
            std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
            for (auto &it_cp : localMapUnseen) {
				desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
			}
			std::vector<DescriptorMatch> allMatches;
			m_matcher->matchInRegion(projected2DPts, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance * 1.5);
			// find visibility of new frame
			const std::vector<Keypoint>& keypoints = frame->getKeypoints();
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
        if (newMapVisibility.size() < m_minWeightNeighbor) {
            return;
        }

        // send frame to mapping task
        m_dropBufferAddKeyframe.push(frame);
    }

    void SolARMappingPipelineMultiProcessing::mapping() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::mapping");

        SRef<Frame> frame;

        // Images to process ?
        if (m_isStopMapping || !m_dropBufferAddKeyframe.tryPop(frame)) {
            xpcf::DelegateTask::yield();
            return;
        }

        SRef<Keyframe> keyframe;
        if (m_mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("New keyframe id: {}", keyframe->getId());
            // Local bundle adjustment
            std::vector<uint32_t> bestIdx;
            m_covisibilityGraph->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx);
            bestIdx.push_back(keyframe->getId());
            m_bundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion, bestIdx);
            m_mapper->pruning();
            m_countNewKeyframes++;
            m_dropBufferNewKeyframeLoop.push(keyframe);
        }

        if (keyframe) {
            m_isStopMapping = true;
            m_dropBufferNewKeyframe.push(keyframe);
        }
    }

    void SolARMappingPipelineMultiProcessing::loopClosure() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::loopClosure");

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
            LOG_DEBUG("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
            LOG_DEBUG("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
            LOG_DEBUG("sim3Transform: \n{}", sim3Transform.matrix());
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
                updateLocalMap(m_refKeyframe);
            }
            // loop optimization
            Transform3Df keyframeOldPose = lastKeyframe->getPose();
            m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
            // map pruning
            m_mapper->pruning();
            // update pose correction
            Transform3Df transform = lastKeyframe->getPose() * keyframeOldPose.inverse();
            m_T_M_W = transform * m_T_M_W;
        }
    }

    void SolARMappingPipelineMultiProcessing::updateLocalMap(const SRef<Keyframe> & keyframe) {

        // Protect from concurrent access
        std::unique_lock<std::mutex> lock(m_mutexUseLocalMap);

        m_localMap.clear();
        m_mapper->getLocalPointCloud(keyframe, m_minWeightNeighbor, m_localMap);
    }

    void SolARMappingPipelineMultiProcessing::globalBundleAdjustment() {

        LOG_DEBUG("SolARMappingPipelineMultiProcessing::globalBundleAdjustment");

        // Global bundle adjustment
        m_globalBundler->bundleAdjustment(m_cameraParams.intrinsic, m_cameraParams.distortion);
        // Map pruning
        m_mapper->pruning();

        LOG_INFO("Nb of keyframes / cloud points: {} / {}",
                 m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());

        LOG_DEBUG("Update global map");
        m_mapper->saveToFile();
    }

    bool SolARMappingPipelineMultiProcessing::isBootstrapFinished() const {

        return m_isBootstrapFinished;
    }

    void SolARMappingPipelineMultiProcessing::setBootstrapSatus(const bool status) {

        LOG_DEBUG("Set bootstrap status to: {}", status);

        m_isBootstrapFinished = status;
    }

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE
