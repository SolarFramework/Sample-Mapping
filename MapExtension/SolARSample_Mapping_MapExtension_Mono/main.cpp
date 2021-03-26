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
#include "api/display/IMatchesOverlay.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IProject.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/IMapping.h"


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
using namespace SolAR::api::reloc;
namespace xpcf  = org::bcom::xpcf;

#define INDEX_USE_CAMERA 0

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

        std::string configxml = std::string("SolARSample_Mapping_MapExtension_Mono_conf.xml");
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
		auto overlay2DGreen = xpcfComponentManager->resolve<display::I2DOverlay>("Green");
		auto overlay2DRed = xpcfComponentManager->resolve<display::I2DOverlay>("Red");
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
		auto pnpRansac = xpcfComponentManager->resolve<api::solver::pose::I3DTransformSACFinderFrom2D3D>();
		auto keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
		auto projector = xpcfComponentManager->resolve<api::geom::IProject>();
		auto mapFilter = xpcfComponentManager->resolve<api::solver::map::IMapFilter>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
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
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		projector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		pnpRansac->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		// Correct pose
		Transform3Df T_M_W = Transform3Df::Identity();

		// Load map from file
		if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map done!");
		}
		else {
			LOG_ERROR("Cannot load map");
			return 1;
		}

		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
		LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());
		
		// update local point cloud
		std::vector<SRef<CloudPoint>> localMap;
		auto updateLocalMap=[&](const SRef<Keyframe> &keyframe){
			localMap.clear();			
			mapper->getLocalPointCloud(keyframe, minWeightNeighbor, localMap);
		};

        // Relocalization and mapping
		std::vector<Transform3Df>   framePoses;
		SRef<Keyframe> refKeyframe; 
		int countNewKeyframes(0);
		bool lostTracking(true);
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
			SRef<Image> displayImage = image->copy();

			// feature extraction image
			std::vector<Keypoint> keypoints;
			keypointsDetector->detect(image, keypoints);
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(image, keypoints, descriptors);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, refKeyframe, pose);

			// Relocalization
			if (lostTracking) {	
				std::vector < uint32_t> retKeyframesId;
				if (keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {					
					SRef<Keyframe> bestRetKeyframe;
					keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe);
					// feature matching to reference keyframe			
					std::vector<DescriptorMatch> matches;
					matcher->match(bestRetKeyframe->getDescriptors(), frame->getDescriptors(), matches);
					matchesFilter->filter(matches, matches, bestRetKeyframe->getKeypoints(), frame->getKeypoints());					

					// find 2D-3D point correspondences
					std::vector<Point2Df> pts2d;
					std::vector<Point3Df> pts3d;
					std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
					std::vector<DescriptorMatch> foundMatches;
					std::vector<DescriptorMatch> remainingMatches;
					corr2D3DFinder->find(bestRetKeyframe, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);

					// pnp ransac
					std::vector<uint32_t> inliers;
					Transform3Df poseFound;
					if (pnpRansac->estimate(pts2d, pts3d, inliers, poseFound) == FrameworkReturnCode::_SUCCESS) {
						LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3d.size());
						LOG_INFO(" Retrieval Success with keyframe id: {}", retKeyframesId[0]);
						// found T_W_M
						T_M_W = poseFound * frame->getPose().inverse();
						// update reference keyframe
						refKeyframe = bestRetKeyframe;
						updateLocalMap(refKeyframe);
						// set pose for the current frame
						frame->setPose(poseFound);
						lostTracking = false;
					}
				}
			}
			else {	// Mapping
				LOG_DEBUG("Reference keyframe id: {}", refKeyframe->getId());
				// correct pose
				pose = T_M_W * pose;				
				framePoses.push_back(pose);
				frame->setPose(pose);

				// feature matching to reference keyframe			
				std::vector<DescriptorMatch> matches;
				matcher->match(refKeyframe->getDescriptors(), descriptors, matches);
				matchesFilter->filter(matches, matches, refKeyframe->getKeypoints(), keypoints);
				float maxMatchDistance = -FLT_MAX;
				for (const auto &it : matches) {
					float score = it.getMatchingScore();
					if (score > maxMatchDistance)
						maxMatchDistance = score;
				}

				// find 2D-3D point correspondences
				std::vector<Point2Df> pts2d;
				std::vector<Point3Df> pts3d;
				std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
				std::vector<DescriptorMatch> foundMatches;
				std::vector<DescriptorMatch> remainingMatches;
				corr2D3DFinder->find(refKeyframe, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);
				if (corres2D3D.size() == 0)
					break;
				std::set<uint32_t> idxCPSeen;					// Index of CP seen
				for (const auto & itCorr : corres2D3D) {
					idxCPSeen.insert(itCorr.second->getId());
				}
				// Find map visibilities of the current frame
				std::map<uint32_t, uint32_t> newMapVisibility;	// map visibilities			
				std::vector< Point2Df > refCPSeenProj;
				projector->project(pts3d, refCPSeenProj, pose); // Project ref cloud point seen to current frame to define inliers/outliers
				std::vector<Point2Df> pts2d_inliers, pts2d_outliers;
				for (int i = 0; i < refCPSeenProj.size(); ++i) {
					float dis = (pts2d[i] - refCPSeenProj[i]).norm();
					if (dis < reprojErrorThreshold) {
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
				for (auto &it_cp : localMap)
					if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
						localMapUnseen.push_back(it_cp);
				std::vector< Point2Df > projected2DPts;
				projector->project(localMapUnseen, projected2DPts, pose);

				// find more inlier matches
				std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
				for (auto &it_cp : localMapUnseen) {
					desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
				}
				std::vector<DescriptorMatch> allMatches;
				matcher->matchInRegion(projected2DPts, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance * 1.5);
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
				if (newMapVisibility.size() < minWeightNeighbor)
					break;

				// mapping
				SRef<Keyframe> keyframe;
				if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
					LOG_DEBUG("New keyframe id: {}", keyframe->getId());
					// Local bundle adjustment
					std::vector<uint32_t> bestIdx;
					covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
					bestIdx.push_back(keyframe->getId());
					LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
					double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion, bestIdx);
					// map pruning
					updateLocalMap(keyframe);
					mapper->pruning(localMap);
					// try to loop detection
					countNewKeyframes++;
					// loop closure
					if (countNewKeyframes >= 10) {
						SRef<Keyframe> detectedLoopKeyframe;
						Transform3Df sim3Transform;
						std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
						if (loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
							// detected loop keyframe
							LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
							LOG_DEBUG("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
							LOG_DEBUG("sim3Transform: \n{}", sim3Transform.matrix());
							// performs loop correction 						
							Transform3Df keyframeOldPose = keyframe->getPose();
							loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
							// loop optimization
							globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
							// map pruning
							mapper->pruning();
							countNewKeyframes = 0;
							// update pose correction
							Transform3Df transform = keyframe->getPose() * keyframeOldPose.inverse();
							T_M_W = transform * T_M_W;
						}
					}
				}
				// update reference keyframe
				if (keyframe) {
					refKeyframe = keyframe;
					updateLocalMap(refKeyframe);
				}

				// draw pose				
				overlay3D->draw(frame->getPose(), displayImage);
				overlay2DRed->drawCircles(pts2d_outliers, displayImage);
				overlay2DGreen->drawCircles(pts2d_inliers, displayImage);
			}						

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
			if (lostTracking) {
				if (viewer3D->display(pointCloud, Transform3Df::Identity(), keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
					break;
			}
			else {
				if (viewer3D->display(pointCloud, frame->getPose(), keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
					break;
			}
        }

		// global bundle adjustment
		globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
		// map pruning
		mapper->pruning();

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
