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
		auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::IFiducialMarkerPose>();
		LOG_INFO("Components created!");
		LOG_INFO("Started!");

		// Load camera intrinsics parameters
		CameraParameters camParams;
		camParams = arDevice->getParameters(0);
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		bootstrapper->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		fiducialMarkerPoseEstimator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		projector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		pnpRansac->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);


		LOG_INFO("loading map from: ",)
		if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map done!");
		}
		LOG_INFO("Mapper info: ");

		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
		LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());

		std::vector<Transform3Df> keyframePoses;
		std::vector<SRef<Keyframe>> allKeyframes;

		keyframesManager->getAllKeyframes(allKeyframes);
		for (auto const &it : allKeyframes)
			keyframePoses.push_back(it->getPose());
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloudManager->getAllPoints(pointCloud);
		// display point cloud 


		LOG_INFO(" kf no: ", allKeyframes.size());
		LOG_INFO(" cloud no: ", pointCloud.size());
		while (true) {
			viewer3D->display(pointCloud, Transform3Df::Identity(), keyframePoses);
		}



    return 0;
}
