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

		std::string configxml = std::string("../conf_floatingMapFusion.xml");
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
		LOG_INFO("AR device created");
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		LOG_INFO("viewer3D created");
		auto viewerImage = xpcfComponentManager->resolve<display::IImageViewer>("viewerImage");
		LOG_INFO("viewerImage created");
		auto globalMapper = xpcfComponentManager->resolve<solver::map::IMapper>("globalMapper");
		LOG_INFO("global mapper created");
		auto floatingMapper = xpcfComponentManager->resolve<solver::map::IMapper>("floatingMapper");

		auto currentGlobalMapper = xpcfComponentManager->resolve<solver::map::IMapper>("currentGlobalMapper");
		LOG_INFO("global mapper created");
		auto currentFloatingMapper = xpcfComponentManager->resolve<solver::map::IMapper>("currentFloatingMapper");

		LOG_INFO("floating mapper created");
		auto mapOverlapDetector = xpcfComponentManager->resolve<loop::IOverlapDetector>();
		LOG_INFO("map overlap detector created");
		auto mapFusion = xpcfComponentManager->resolve<solver::map::IMapFusion>();
		LOG_INFO("map fusion created");
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		LOG_INFO("global bundler created");

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

		// get point clouds and keyframes


		//std::vector<SRef<CloudPoint>>globalPointCloud_before;
		//for (auto &m : globalPointCloud) {
		//	SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(Point3Df(m->getX(), m->getY(), m->getZ()));
		//	globalPointCloud_before.push_back(cp);
		//}
	
		//for (auto &m : floatingPointCloud) {
		//	SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(Point3Df(m->getX(), m->getY(), m->getZ()));
		//	globalPointCloud_before.push_back(cp);
		//}
	
		std::vector<Transform3Df>aKfPoses, bKfPoses, aPoses, bPoses;
		aPoses = {}; bPoses = {};

		//aKfPoses = {}; bKfPoses = {};
		std::vector<Transform3Df> sim3Transform;
		Transform3Df globalBestPose, floatingBestPose;
		std::vector<std::pair<uint32_t, uint32_t>>overlaps;
		std::vector<double>overlapScores;
		SRef<Image>globalView;
		
		// detect overlap from global/floating map and extract sim3
		LOG_INFO("<Overlaps detection: >");
		if (mapOverlapDetector->detect(globalMapper, floatingMapper, sim3Transform, overlaps, overlapScores) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("	->number of overlaps detected: {}", overlaps.size());
			auto maxOverlap = std::max_element(overlapScores.begin(), overlapScores.end());
			int idxBestOverlap = std::distance(overlapScores.begin(), maxOverlap);
			LOG_INFO("	->best at: {}  with {} ", idxBestOverlap, overlapScores[idxBestOverlap]);
			LOG_INFO("	->floating kf id {} - global kf id {}", overlaps[idxBestOverlap].first, overlaps[idxBestOverlap].second);

			for (const auto &t : sim3Transform) {
				std::cout << "------------------------------" << std::endl;
				LOG_INFO("sim3: {}", t.matrix());
			}


			Transform3Df  gtSim3Transform;
			std::string transformFileName = "../TransformLocalToGlobal.txt";
			std::ifstream transformFile;
			transformFile.open(transformFileName);
			if (!transformFile.is_open()) {
				LOG_ERROR("Cannot open transform file: {}", transformFileName);
				return 1;
			}
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
					transformFile >> gtSim3Transform(i, j);
			transformFile.close();
			//std::cout << " gt transform loaded corretcly" << std::endl;
			//// map fusion

			//// display		
			int idxOverlap = 0;
			while (idxOverlap< sim3Transform.size()) {
				globalMapper->loadFromFile(); floatingMapper->loadFromFile();
				std::cout << "------------->operating with overlap: " << idxOverlap <<" / "<<sim3Transform.size()<<" with: "<<
							overlapScores[idxOverlap]<<std::endl;

				currentFloatingMapper->set(floatingMapper);
				currentGlobalMapper->set(globalMapper);
				Transform3Df  bestSim3Transform = sim3Transform[idxOverlap];
				uint32_t nbMatches;
				float error;
				int idxWeiting = 500;
			//	LOG_INFO("sim3: \n{}", sim3Transform[idxOverlap].matrix());
			//	LOG_INFO("sim3: \n{}", gtSim3Transform.matrix());
				if (mapFusion->merge(currentFloatingMapper, currentGlobalMapper,
					bestSim3Transform, nbMatches, error) == FrameworkReturnCode::_SUCCESS) {

					SRef<IPointCloudManager>globalPointCloudManager;
					SRef<IKeyframesManager> globalKeyframesManager;
					SRef<ICovisibilityGraph> globalCovisibilityManager;

					std::vector<SRef<Keyframe>> globalKeyframes;
					std::vector<SRef<CloudPoint>> globalPointCloud;

					globalPointCloud.clear();
					globalKeyframes.clear();

					//std::cout << "---> injecting parameters to bundler" << std::endl;
					currentGlobalMapper->getPointCloudManager(globalPointCloudManager);
					currentGlobalMapper->getKeyframesManager(globalKeyframesManager);
					currentGlobalMapper->getCovisibilityGraph(globalCovisibilityManager);

					//globalBundler->bindTo < xpcf::IInjectable >()->inject<IPointCloudManager>(globalPointCloudManager);
					//globalBundler->bindTo < xpcf::IInjectable >()->inject<IKeyframesManager>(globalKeyframesManager);
					//globalBundler->bindTo < xpcf::IInjectable >()->inject<ICovisibilityGraph>(globalCovisibilityManager);

					//double error_bundle = globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
					//currentGlobalMapper->pruning();
					//std::cout << "---> error bundle: " << error_bundle << std::endl;


					globalPointCloudManager->getAllPoints(globalPointCloud);
					globalKeyframesManager->getAllKeyframes(globalKeyframes);
					aKfPoses.clear();

					for (const auto &aKf : globalKeyframes)
						aKfPoses.push_back(aKf->getPose());
					for (unsigned int w = 0; w < idxWeiting; ++w)
						viewer3D->display(globalPointCloud, Transform3Df::Identity(), aKfPoses, {}, {}, {});
				}
				else {
					LOG_INFO(" no merge operated");
				}
				++idxOverlap;
			}
		}
		else {
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
