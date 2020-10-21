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
		auto globalMapper = xpcfComponentManager->resolve<solver::map::IMapper>("aMapper");
		auto floatingMapper = xpcfComponentManager->resolve<solver::map::IMapper>("bMapper");
		auto mapOverlapDetector = xpcfComponentManager->resolve<loop::IOverlapDetector>();	
		LOG_INFO("Components created!");

		// Load camera intrinsics parameters
		CameraParameters camParams;
		camParams = arDevice->getParameters(0);

		if (globalMapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map A done!");
		}
		if (floatingMapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map B done!");
		}

		mapOverlapDetector->setGlobalMapper(globalMapper);
		LOG_INFO("overlap detector setted");
		SRef<IPointCloudManager> aPointCloudManager, bPointCloudManager;
		SRef<IKeyframesManager> aKeyframesManager, bKeyframesManager;

		globalMapper->getPointCloudManager(aPointCloudManager);
		globalMapper->getKeyframesManager(aKeyframesManager);

		floatingMapper->getPointCloudManager(bPointCloudManager);
		floatingMapper->getKeyframesManager(bKeyframesManager);

		LOG_INFO("map A");
		LOG_INFO("Number of point cloud: {}", aPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", aKeyframesManager->getNbKeyframes());

		LOG_INFO("map B");
		LOG_INFO("Number of point cloud: {}", bPointCloudManager->getNbPoints());
		LOG_INFO("Number of keyframes: {}", bKeyframesManager->getNbKeyframes());

		// get point clouds and keyframes
		std::vector<SRef<Keyframe>> aKeyframes, bKeyframes;
		std::vector<SRef<CloudPoint>> aPointCloud, bPointCloud;

		aKeyframesManager->getAllKeyframes(aKeyframes);
		bKeyframesManager->getAllKeyframes(bKeyframes);

		aPointCloudManager->getAllPoints(aPointCloud);
		bPointCloudManager->getAllPoints(bPointCloud);

		std::vector<Transform3Df>aKfPoses, bKfPoses, aPoses,bPoses;
		aPoses = {}; bPoses = {};
		for (const auto &aKf : aKeyframes)
			aKfPoses.push_back(aKf->getPose());

		for (const auto &bKf : bKeyframes)
			bKfPoses.push_back(bKf->getPose());

		SRef<Keyframe> detectedLoopKeyframe;
		Transform3Df sim3Transform;
		std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
		for (const auto & aKf : aKeyframes) {
			LOG_INFO("trying to detect loop with kf: {}", aKf->getId());
		}
		while (true) {
			viewer3D->display(aPointCloud, Transform3Df::Identity(), aKfPoses,aPoses,bPointCloud,bKfPoses);
		}
    return 0;
}
