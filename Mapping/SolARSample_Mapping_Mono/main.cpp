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
#include "api/display/I3DPointsViewer.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/IMapManager.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"

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
		auto covisibilityGraphManager = xpcfComponentManager->resolve<ICovisibilityGraphManager>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapManager = xpcfComponentManager->resolve<IMapManager>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractorFromImage>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
        auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::ITrackablePose>();
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
		CameraRigParameters camRigParams = arDevice->getCameraParameters();
		CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		bootstrapper->setCameraParameters(camParams.intrinsic, camParams.distortion);
		tracking->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams);
		fiducialMarkerPoseEstimator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		undistortKeypoints->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();

		// Correct pose and Bootstrap
		Transform3Df T_M_W = Transform3Df::Identity();
		bool isFoundTransform = false;
		bool bootstrapOk = false;

        // Load Trackable
        SRef<Trackable> trackable;
        if (trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("cannot load fiducial marker");
            return -1;
        }
        else
        {
            if (fiducialMarkerPoseEstimator->setTrackable(trackable)!= FrameworkReturnCode::_SUCCESS)
            {
                LOG_ERROR("cannot set fiducial marker as a trackable ofr fiducial marker pose estimator");
                return -1;
            }
        }		

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
			if (framePoses.size() == 0
				|| viewer3D->display(pointCloud, framePoses.back(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				return false;
			else
				return true;
		};

        // Mapping
		std::vector<Transform3Df> framePoses;
		int countNewKeyframes(0);
		while (true)
		{
			// get data
			SRef<Image> displayImage;
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
					return -1;
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
			
			// feature extraction image
			std::vector<Keypoint> keypoints, undistortedKeypoints;			
			SRef<DescriptorBuffer> descriptors;
			if (descriptorExtractor->extract(image, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
				continue;
			undistortKeypoints->undistort(keypoints, undistortedKeypoints);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, pose);
			framePoses.push_back(pose);

			// check bootstrap
			if (!bootstrapOk) {
				if (bootstrapper->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
					double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
					SRef<Keyframe> keyframe2;
					keyframesManager->getKeyframe(1, keyframe2);
					tracking->updateReferenceKeyframe(keyframe2);
					bootstrapOk = true;
					LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
				}
			}
			else {
				// update visibility for the current frame				
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
					covisibilityGraphManager->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
					if (bestIdx.size() < NB_LOCALKEYFRAMES)
						bestIdxToOptimize = bestIdx;
					else
						bestIdxToOptimize.insert(bestIdxToOptimize.begin(), bestIdx.begin(), bestIdx.begin() + NB_LOCALKEYFRAMES);
					bestIdxToOptimize.push_back(keyframe->getId());
					LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdxToOptimize.size());
					double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion, bestIdxToOptimize);
					// map pruning
					std::vector<SRef<CloudPoint>> localMap;
					mapManager->getLocalPointCloud(keyframe, minWeightNeighbor, localMap);
					int nbRemovedCP = mapManager->pointCloudPruning(localMap);
					std::vector<SRef<Keyframe>> localKeyframes;
					keyframesManager->getKeyframes(bestIdx, localKeyframes);
					int nbRemovedKf = mapManager->keyframePruning(localKeyframes);
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
							mapManager->pointCloudPruning();
							mapManager->keyframePruning();
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
			}
			// draw pose
			overlay3D->draw(frame->getPose(), displayImage);
			// display image
			if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
				break;
			// display keyframes and point cloud
			if (bootstrapOk && !fnDisplay(framePoses))
				break;
        }

		// global bundle adjustment
		globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
		// map pruning
		mapManager->pointCloudPruning();
		mapManager->keyframePruning();
		LOG_INFO("Nb keyframes of map: {}", keyframesManager->getNbKeyframes());
		LOG_INFO("Nb cloud points of map: {}", pointCloudManager->getNbPoints());

		// display
		while (fnDisplay(framePoses));

		// Save map
        mapManager->saveToFile();
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
