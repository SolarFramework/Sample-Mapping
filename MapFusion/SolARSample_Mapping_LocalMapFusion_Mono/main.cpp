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
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/solver/map/IMapFusion.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
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

        std::string configxml = std::string("SolARSample_Mapping_LocalMapFusion_Mono_conf.xml");
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
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto globalMap = xpcfComponentManager->resolve<solver::map::IMapper>("Global");
		auto localMap = xpcfComponentManager->resolve<solver::map::IMapper>("Local");
		auto mapFusion = xpcfComponentManager->resolve<solver::map::IMapFusion>();
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();		
		LOG_INFO("Components created!");

		// get intrinsic parameters of camera
		CameraParameters camParams = arDevice->getParameters(INDEX_USE_CAMERA);

		// Load global map from file
		if (globalMap->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load global map done!");
		}
		else {
			LOG_ERROR("Cannot load global map");
			return 1;
		}		

		// Load local map from file
		if (localMap->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load local map done!");
		}
		else {
			LOG_ERROR("Cannot load local map");
			return 1;
		}

		// get keyframe mananger and point cloud mananger
		SRef<IPointCloudManager> globalPointCloudManager, localPointCloudManager;
		SRef<IKeyframesManager> globalKeyframesManager, localKeyframesManager;
		globalMap->getPointCloudManager(globalPointCloudManager);
		globalMap->getKeyframesManager(globalKeyframesManager);
		localMap->getPointCloudManager(localPointCloudManager);
		localMap->getKeyframesManager(localKeyframesManager);

		LOG_INFO("Global map");
		LOG_INFO("Number of point cloud: {}", globalPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", globalKeyframesManager->getNbKeyframes());

		LOG_INFO("Local map");
		LOG_INFO("Number of point cloud: {}", localPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", localKeyframesManager->getNbKeyframes());

		// load transformation matrix
		Transform3Df transformLocalToGlobal;
		std::string transformFileName = "TransformLocalToGlobal.txt";
		std::ifstream transformFile;
		transformFile.open(transformFileName);
		if (!transformFile.is_open()) {
			LOG_ERROR("Cannot open transform file: {}", transformFileName);
			return 1;
		}
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				transformFile >> transformLocalToGlobal(i, j);
		transformFile.close();
		LOG_INFO("Transformation matrix: \n{}", transformLocalToGlobal.matrix());

		// map fusion
		uint32_t nbMatches;
		float error;
		if (mapFusion->merge(localMap, globalMap, transformLocalToGlobal, nbMatches, error) == FrameworkReturnCode::_ERROR_) {
			LOG_INFO("Cannot merge two maps");
			return 0;
		}
		LOG_INFO("The refined Transformation matrix: \n{}", transformLocalToGlobal.matrix());
		LOG_INFO("Number of matched cloud points: {}", nbMatches);
		LOG_INFO("Error: {}", error);

		// global bundle adjustment
		globalBundler->setMapper(globalMap);
		globalBundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);

		// pruning
		globalMap->pruning();
		
		// display		
		std::vector<SRef<Keyframe>> globalKeyframes;
		std::vector<SRef<CloudPoint>> globalPointCloud;
		globalKeyframesManager->getAllKeyframes(globalKeyframes);
		globalPointCloudManager->getAllPoints(globalPointCloud);
		std::vector<Transform3Df> globalKeyframesPoses;
		for (const auto &it : globalKeyframes)
			globalKeyframesPoses.push_back(it->getPose());
		while (true) {
			if (viewer3D->display(globalPointCloud, Transform3Df::Identity(), globalKeyframesPoses, {}, {}, {}) == FrameworkReturnCode::_STOP)
				break;
		}
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
