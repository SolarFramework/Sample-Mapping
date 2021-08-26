/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DPointsViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/geom/IProject.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"
#include "api/solver/pose/IFiducialMarkerPose.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
using namespace SolAR::api::reloc;
namespace xpcf  = org::bcom::xpcf;

#define INDEX_USE_CAMERA 0
#define NB_NEWKEYFRAMES_LOOP 20

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    try {
        /* instantiate component manager*/
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("SolARSample_Mapping_Multi_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

        if(xpcfComponentManager->load(configxml.c_str())!=org::bcom::xpcf::_SUCCESS)
        {
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto matchesOverlay = xpcfComponentManager->resolve<api::display::IMatchesOverlay>();
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
		auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
		auto corr2D3DFinder = xpcfComponentManager->resolve<solver::pose::I2D3DCorrespondencesFinder>();
		auto keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
		auto projector = xpcfComponentManager->resolve<api::geom::IProject>();
		auto mapFilter = xpcfComponentManager->resolve<api::solver::map::IMapFilter>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
		auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::IFiducialMarkerPose>();
		LOG_INFO("Components created!");
		LOG_INFO("Start AR device loader");
		// Connect remotely to the HoloLens streaming app
		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		// Load camera intrinsics parameters
		CameraParameters camParams;
		camParams = arDevice->getParameters(0);
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		bootstrapper->setCameraParameters(camParams.intrinsic, camParams.distortion);
		tracking->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		fiducialMarkerPoseEstimator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		projector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		undistortKeypoints->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		// display point cloud function
		auto fnDisplay = [&keyframesManager, &pointCloudManager, &viewer3D](const std::vector<Transform3Df>& framePoses) {
			// get all keyframes and point cloud
			std::vector<Transform3Df>   keyframePoses;
			std::vector<SRef<Keyframe>> allKeyframes;
			keyframesManager->getAllKeyframes(allKeyframes);
			for (auto const &it : allKeyframes)
				keyframePoses.push_back(it->getPose());
			std::vector<SRef<CloudPoint>> pointCloud;
			pointCloudManager->getAllPoints(pointCloud);
			// display point cloud 
			if (viewer3D->display(pointCloud, framePoses.back(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				return false;
			else
				return true;
		};
			
		// buffers
		xpcf::DropBuffer<std::pair<SRef<Image>, Transform3Df>>						m_dropBufferCamImagePoseCaptureBootstrap;
		xpcf::DropBuffer<std::pair<SRef<Image>, Transform3Df>>						m_dropBufferCamImagePoseCapture;
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferKeypoints;
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferFrameDescriptors;
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferAddKeyframe;
		xpcf::DropBuffer<SRef<Keyframe>>											m_dropBufferNewKeyframe;
		xpcf::DropBuffer<SRef<Keyframe>>											m_dropBufferNewKeyframeLoop;
		xpcf::DropBuffer<SRef<Image>>												m_dropBufferDisplay;				

		// variables
		bool stop = false;								// is stop process?
		std::vector<Transform3Df> framePoses;			// frame poses to display		
		bool isStopMapping = false;						// is stop mapping?
		int countNewKeyframes = 0;						// number of keyframes to try loop detection
		int count = 0;									// number of process frames
		std::mutex m_mutexUseLocalMap;					
		Transform3Df T_M_W = Transform3Df::Identity();	// Correct pose and Bootstrap
		bool isFoundTransform = false;				
		bool bootstrapOk = false;						// bootstrap is done?

		// bootstrap task
		auto fnBootstrap = [&]() {
			std::pair<SRef<Image>, Transform3Df> imagePose;
			if (bootstrapOk || !m_dropBufferCamImagePoseCaptureBootstrap.tryPop(imagePose)) {
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<Image> image = imagePose.first;
			Transform3Df pose = imagePose.second;
			// find T_W_M
			if (!isFoundTransform) {
				m_dropBufferDisplay.push(image);
				Transform3Df T_M_C;
				if (fiducialMarkerPoseEstimator->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
					T_M_W = T_M_C * pose.inverse();
					isFoundTransform = true;
				}
				return;
			}
			// do bootstrap
			SRef<Image> view;
			if (bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
				// apply bundle adjustement 
				bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);	
				SRef<Keyframe> keyframe2;
				keyframesManager->getKeyframe(1, keyframe2);
				tracking->updateReferenceKeyframe(keyframe2);
				framePoses.push_back(keyframe2->getPose());			
				LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
				bootstrapOk = true;
				return;
			}
			overlay3D->draw(pose, view);
			m_dropBufferDisplay.push(view);
		};	

		// Camera image capture task
		auto fnCamImageCapture = [&]()
		{			
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				stop = true;
				return;
			}
			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];
			// correct pose
			pose = T_M_W * pose;
			if (bootstrapOk)
				m_dropBufferCamImagePoseCapture.push(std::make_pair(image, pose));			
			else
				m_dropBufferCamImagePoseCaptureBootstrap.push(std::make_pair(image, pose));
		};

		// Keypoint detection task
		auto fnDetection = [&]()
		{						
			std::pair<SRef<Image>, Transform3Df> imagePose;
			if (!bootstrapOk || !m_dropBufferCamImagePoseCapture.tryPop(imagePose)) {
				xpcf::DelegateTask::yield();
				return;
			}
			std::vector<Keypoint> keypoints, undistortedKeypoints;
			keypointsDetector->detect(imagePose.first, keypoints);
			undistortKeypoints->undistort(keypoints, undistortedKeypoints);
			if (keypoints.size() > 0)
				m_dropBufferKeypoints.push(xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, nullptr, imagePose.first, nullptr, imagePose.second));
		};

		// Feature extraction task
		auto fnExtraction = [&]()
		{			
			SRef<Frame> frame;
			if (!m_dropBufferKeypoints.tryPop(frame)) {
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(frame->getView(), frame->getKeypoints(), descriptors);
			frame->setDescriptors(descriptors);
			m_dropBufferFrameDescriptors.push(frame);
		};

		// Update visibilities task					
		auto fnUpdateVisibilities = [&]()
		{			
			SRef<Frame> frame;
			if (!m_dropBufferFrameDescriptors.tryPop(frame)) {
				xpcf::DelegateTask::yield();
				return;
			}
			std::unique_lock<std::mutex> lock(m_mutexUseLocalMap);
			SRef<Keyframe> newKeyframe;
			if (m_dropBufferNewKeyframe.tryPop(newKeyframe))
			{
				LOG_DEBUG("Update new keyframe in update task");
				tracking->updateReferenceKeyframe(newKeyframe);
				SRef<Frame> tmpFrame;
				m_dropBufferAddKeyframe.tryPop(tmpFrame);
				isStopMapping = false;
			}
			framePoses.push_back(frame->getPose());
			
			// update visibility for the current frame
			SRef<Image> displayImage;
			tracking->process(frame, displayImage);
			overlay3D->draw(frame->getPose(), displayImage);
			LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());
			if (frame->getVisibility().size() < minWeightNeighbor) {
				stop = true;
				return;
			}

			// send frame to mapping task
			m_dropBufferAddKeyframe.push(frame);

			// display
			m_dropBufferDisplay.push(displayImage);			
		};

		// Mapping task
		auto fnMapping = [&]()
		{			
			SRef<Frame> frame;
			if (isStopMapping || (!m_dropBufferAddKeyframe.tryPop(frame))) {
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<Keyframe> keyframe;
			if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
				LOG_DEBUG("New keyframe id: {}", keyframe->getId());
				// Local bundle adjustment
				std::vector<uint32_t> bestIdx;
				covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
				bestIdx.push_back(keyframe->getId());
				bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion, bestIdx);
				std::vector<SRef<CloudPoint>> localMap;
				mapper->getLocalPointCloud(keyframe, minWeightNeighbor, localMap);
				mapper->pruning(localMap);
				countNewKeyframes++;
				m_dropBufferNewKeyframeLoop.push(keyframe);
			}
			if (keyframe) {
				isStopMapping = true;
				m_dropBufferNewKeyframe.push(keyframe);
			}
		};


		// loop closure task
		auto fnLoopClosure = [&]() {
			SRef<Keyframe> lastKeyframe;
			if ((countNewKeyframes < NB_NEWKEYFRAMES_LOOP) || (!m_dropBufferNewKeyframeLoop.tryPop(lastKeyframe))) {
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<Keyframe> detectedLoopKeyframe;
			Transform3Df sim3Transform;
			std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
			if (loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
				// detected loop keyframe
				LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
				LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
				LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
				// performs loop correction 	
				{
					std::unique_lock<std::mutex> lock(m_mutexUseLocalMap);
					loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
					countNewKeyframes = 0;
					Eigen::Matrix3f scale;
					Eigen::Matrix3f rot;
					sim3Transform.computeScalingRotation(&scale, &rot);
					sim3Transform.linear() = rot;
					sim3Transform.translation() = sim3Transform.translation() / scale(0, 0);
					T_M_W = sim3Transform * T_M_W;
				}
				// loop optimization
				Transform3Df keyframeOldPose = lastKeyframe->getPose();
				globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
				// map pruning
				mapper->pruning();				
				// update pose correction
				Transform3Df transform = lastKeyframe->getPose() * keyframeOldPose.inverse();
				T_M_W = transform * T_M_W;
			}
		};


		// instantiate and start tasks
		xpcf::DelegateTask taskCamImageCapture(fnCamImageCapture);
		xpcf::DelegateTask taskBootstrap(fnBootstrap);		
		xpcf::DelegateTask taskUpdateVisibilities(fnUpdateVisibilities);
		xpcf::DelegateTask taskDetection(fnDetection);
		xpcf::DelegateTask taskExtraction(fnExtraction);
		xpcf::DelegateTask taskMapping(fnMapping);
		xpcf::DelegateTask taskLoopClosure(fnLoopClosure);

		taskCamImageCapture.start();
		taskBootstrap.start();		
		taskDetection.start();
		taskExtraction.start();
		taskUpdateVisibilities.start();
		taskMapping.start();
		taskLoopClosure.start();

		// Start tracking
		clock_t start, end;
		start = clock();
		while (!stop)
		{
			SRef<Image> displayImage;
			if (!m_dropBufferDisplay.tryPop(displayImage)) {
				xpcf::DelegateTask::yield();
				continue;
			}
			if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
				stop = true;
			if (bootstrapOk) {
				if (!fnDisplay(framePoses))
					stop = true;
			}
			++count;
		}		

		// Stop tasks
		taskCamImageCapture.stop();
		taskBootstrap.stop();
		taskDetection.stop();
		taskExtraction.stop();
		taskUpdateVisibilities.stop();
		taskMapping.stop();
		taskLoopClosure.stop();

		// display stats on frame rate
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);


		// global bundle adjustment
		globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
		// map pruning
		mapper->pruning();

		LOG_INFO("Nb keyframes of map: {}", keyframesManager->getNbKeyframes());
		LOG_INFO("Nb cloud points of map: {}", pointCloudManager->getNbPoints());

		// visualize final map	
		while (fnDisplay(framePoses)) {}

		// Save map
		mapper->saveToFile();
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
