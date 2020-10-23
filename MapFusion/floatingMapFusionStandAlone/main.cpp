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
#include "api/loop/IOverlapDetector.h"
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/solver/map/IMapFusion.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
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

		std::string configxml = std::string("conf_floatingMapFusion.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
			return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto globalMapper = xpcfComponentManager->resolve<solver::map::IMapper>("globalMapper");
		auto floatingMapper = xpcfComponentManager->resolve<solver::map::IMapper>("floatingMapper");
		auto mapOverlapDetector = xpcfComponentManager->resolve<loop::IOverlapDetector>();
		auto mapFusion = xpcfComponentManager->resolve<solver::map::IMapFusion>();
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();


		LOG_INFO("Components created!");

		// Load camera intrinsics parameters
		CameraParameters camParams = arDevice->getParameters(INDEX_USE_CAMERA);

		if (globalMapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map A done!");
		}
		if (floatingMapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map B done!");
		}

		mapOverlapDetector->setGlobalMapper(globalMapper);
		LOG_INFO("overlap detector setted");
		SRef<IPointCloudManager> floatingPointCloudManager, globalPointCloudManager;
		SRef<IKeyframesManager> floatingKeyframesManager, globalKeyframesManager;

		globalMapper->getPointCloudManager(globalPointCloudManager);
		globalMapper->getKeyframesManager(globalKeyframesManager);

		floatingMapper->getPointCloudManager(floatingPointCloudManager);
		floatingMapper->getKeyframesManager(floatingKeyframesManager);

		LOG_INFO("map A");
		LOG_INFO("Number of point cloud: {}", globalPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", globalKeyframesManager->getNbKeyframes());

		LOG_INFO("map B");
		LOG_INFO("Number of point cloud: {}", floatingPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", floatingKeyframesManager->getNbKeyframes());

		// get point clouds and keyframes
		std::vector<SRef<Keyframe>> floatingKeyframes, globalKeyframes;
		std::vector<SRef<CloudPoint>> floatingPointCloud, globalPointCloud;

		globalKeyframesManager->getAllKeyframes(globalKeyframes);
		globalPointCloudManager->getAllPoints(globalPointCloud);

		floatingKeyframesManager->getAllKeyframes(floatingKeyframes);
		floatingPointCloudManager->getAllPoints(floatingPointCloud);

		std::vector<SRef<CloudPoint>>globalPointCloud_before;
		for (auto &m : globalPointCloud) {
			SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(Point3Df(m->getX(), m->getY(), m->getZ()));
			globalPointCloud_before.push_back(cp);
		}
	
		for (auto &m : floatingPointCloud) {
			SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(Point3Df(m->getX(), m->getY(), m->getZ()));
			globalPointCloud_before.push_back(cp);
		}

		LOG_INFO("map C");
		LOG_INFO("Number of point cloud: {}", globalPointCloud_before.size());
	
		std::vector<Transform3Df>aKfPoses, bKfPoses, aPoses, bPoses;
		aPoses = {}; bPoses = {};

		//aKfPoses = {}; bKfPoses = {};
		std::vector<Transform3Df> sim3Transform;
		Transform3Df globalBestPose, floatingBestPose;
		std::vector<std::pair<uint32_t, uint32_t>>overlaps;
		std::vector<double>overlapScores;
		SRef<Image>globalView;
		SRef<Image>floatinglView;

		// detect overlap from global/floating map and extract sim3
		LOG_INFO("<Overlaps detection: >");
		if (mapOverlapDetector->detect(globalMapper,floatingMapper, sim3Transform, overlaps, overlapScores) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("	->number of overlaps detected: {}", overlaps.size());

			globalView = globalKeyframes[overlaps[0].second]->getView();
			floatinglView = globalKeyframes[overlaps[0].first]->getView();
			auto maxOverlap = std::max_element(overlapScores.begin(), overlapScores.end());
			int idxBestOverlap = std::distance(overlapScores.begin(), maxOverlap);
			LOG_INFO("	->best at: {}  with {} ", idxBestOverlap ,overlapScores[idxBestOverlap]);
			LOG_INFO("	->floating kf id {} - global kf id {}", overlaps[idxBestOverlap].first , overlaps[idxBestOverlap].second);
			
			// save overlaped poses as frames poses for viz purpose
			aPoses.push_back(floatingKeyframes[overlaps[idxBestOverlap].first]->getPose());
			aPoses.push_back(globalKeyframes[overlaps[idxBestOverlap].second]->getPose());

			Transform3Df  bestSim3Transform = sim3Transform[idxBestOverlap];
			// map fusion
			uint32_t nbMatches;
			float error;
			if (mapFusion->merge(floatingMapper, globalMapper, bestSim3Transform, nbMatches, error) == FrameworkReturnCode::_SUCCESS) {

				LOG_INFO("The refined Transformation matrix: {}", bestSim3Transform.matrix());
				LOG_INFO("Number of matched cloud points: {}", nbMatches);
				LOG_INFO("Error: {}", error);

				// global bundle adjustment
				globalBundler->setMapper(globalMapper);
				globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);

				globalMapper->pruning();

				// display		
				globalPointCloud.clear();
				globalPointCloudManager->getAllPoints(globalPointCloud);
				globalKeyframes.clear();
				globalKeyframesManager->getAllKeyframes(globalKeyframes);
				for (const auto &aKf : globalKeyframes)
					aKfPoses.push_back(aKf->getPose());

				viewer3D->display(globalPointCloud, Transform3Df::Identity(), aKfPoses, {}, {}, {});
				while (true) {
					if (viewer3D->display(globalPointCloud, Transform3Df::Identity(), aKfPoses, {}, globalPointCloud_before, {}) == FrameworkReturnCode::_STOP)
						break;
				}
			}
			else {
				LOG_INFO(" no merge operated");
				return -1;
			}
		}else {
			LOG_INFO(" no overlaps detected")
				return -1;
		}
	}
	catch (xpcf::Exception e){
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}
    return 0;
}
