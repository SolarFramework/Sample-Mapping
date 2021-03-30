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
#include "core/Log.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DPointsViewer.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
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
#define NB_LOCALKEYFRAMES 10
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

        std::string configxml = std::string("SolARSample_Mapping_Mono_conf.xml");
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
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
		auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
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
        camParams = arDevice->getParameters(INDEX_USE_CAMERA);
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		bootstrapper->setCameraParameters(camParams.intrinsic, camParams.distortion);
		tracking->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		fiducialMarkerPoseEstimator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		undistortKeypoints->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();

		// Correct pose and Bootstrap
		Transform3Df T_M_W = Transform3Df::Identity();
		bool isFoundTransform = false;
		bool bootstrapOk = false;
		while (!bootstrapOk) {
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];
			// find T_W_M
			if (!isFoundTransform) {
				if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
					exit(0);
				Transform3Df T_M_C;
				if (fiducialMarkerPoseEstimator->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
					T_M_W = T_M_C * pose.inverse();
					isFoundTransform = true;
				}
				else
					continue;
			}				
			// correct pose
			pose = T_M_W * pose;
			// do bootstrap
			SRef<Image> view;
			if (bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
				// apply bundle adjustement 
				double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
				bootstrapOk = true;
			}
			overlay3D->draw(pose, view);
			if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
				exit(0);
		}
		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());

        // Mapping
		std::vector<Transform3Df>   framePoses;
		SRef<Keyframe> keyframe2; 
		keyframesManager->getKeyframe(1, keyframe2);
		tracking->updateReferenceKeyframe(keyframe2);
		int countNewKeyframes(0);
		while (true)
		{
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
            std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];

			// correct pose
			pose = T_M_W * pose;

			// feature extraction image
			std::vector<Keypoint> keypoints, undistortedKeypoints;
			keypointsDetector->detect(image, keypoints);
			undistortKeypoints->undistort(keypoints, undistortedKeypoints);
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(image, keypoints, descriptors);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, pose);
			framePoses.push_back(pose);

			// update visibility for the current frame
			SRef<Image> displayImage;
			tracking->process(frame, displayImage);
			LOG_DEBUG("Number of tracked points: {}", frame->getVisibility().size());
			if (frame->getVisibility().size() < minWeightNeighbor)
				break;

			// mapping
			SRef<Keyframe> keyframe;
			if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
				LOG_DEBUG("New keyframe id: {}", keyframe->getId());			
				// Local bundle adjustment
				std::vector<uint32_t> bestIdx, bestIdxToOptimize;
				covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
				if (bestIdx.size() < NB_LOCALKEYFRAMES)
					bestIdxToOptimize = bestIdx;
				else
					bestIdxToOptimize.insert(bestIdxToOptimize.begin(), bestIdx.begin(), bestIdx.begin() + NB_LOCALKEYFRAMES);
				bestIdxToOptimize.push_back(keyframe->getId());
				LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdxToOptimize.size());
				double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion, bestIdxToOptimize);
				// map pruning
				std::vector<SRef<CloudPoint>> localMap;
				mapper->getLocalPointCloud(keyframe, minWeightNeighbor, localMap);
				int nbRemovedCP = mapper->pointCloudPruning(localMap);
				std::vector<SRef<Keyframe>> localKeyframes;
				keyframesManager->getKeyframes(bestIdx, localKeyframes);
				int nbRemovedKf = mapper->keyframePruning(localKeyframes);
				LOG_DEBUG("Nb of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
				// try to loop detection
				countNewKeyframes++;
				// loop closure
				if (countNewKeyframes >= NB_NEWKEYFRAMES_LOOP) {
					SRef<Keyframe> detectedLoopKeyframe;
					Transform3Df sim3Transform;
					std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
					if (loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
						// detected loop keyframe
						LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
						LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
						LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
						// performs loop correction 						
						Transform3Df keyframeOldPose = keyframe->getPose();
						loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
						// loop optimization
						globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);						
						// map pruning
						mapper->pointCloudPruning();
						mapper->keyframePruning();
						countNewKeyframes = 0;
						// update pose correction
						Transform3Df transform = keyframe->getPose() * keyframeOldPose.inverse();
						T_M_W = transform * T_M_W;
					}
				}
			}			

			// update reference keyframe
			if (keyframe) {
				tracking->updateReferenceKeyframe(keyframe);
			}			
			// draw pose
			overlay3D->draw(frame->getPose(), displayImage);

			// display
			if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
				break;
			// get all keyframes and point cloud
			std::vector<Transform3Df> keyframePoses;	
			std::vector<SRef<Keyframe>> allKeyframes;
			keyframesManager->getAllKeyframes(allKeyframes);
			for (auto const &it : allKeyframes)
				keyframePoses.push_back(it->getPose());
			std::vector<SRef<CloudPoint>> pointCloud;
			pointCloudManager->getAllPoints(pointCloud);
			// display point cloud 
			if (viewer3D->display(pointCloud, frame->getPose(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				break;
        }

		// global bundle adjustment
		globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
		// map pruning
		mapper->pointCloudPruning();
		mapper->keyframePruning();
		LOG_INFO("Nb keyframes of map: {}", keyframesManager->getNbKeyframes());
		LOG_INFO("Nb cloud points of map: {}", pointCloudManager->getNbPoints());

		// display
		std::vector<Transform3Df> keyframePoses;
		std::vector<SRef<Keyframe>> allKeyframes;
		keyframesManager->getAllKeyframes(allKeyframes);
		for (auto const &it : allKeyframes)
			keyframePoses.push_back(it->getPose());
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloudManager->getAllPoints(pointCloud);
		while (true) {						 
			if (viewer3D->display(pointCloud, keyframePoses[0], keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				break;
		}

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
