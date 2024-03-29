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
#include "api/display/I3DPointsViewer.h"
#include "api/storage/IMapManager.h"
#include "api/storage/IPointCloudManager.h"

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

		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("SolARSample_Mapping_MapVisualizer_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
			return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto mapManager = xpcfComponentManager->resolve<storage::IMapManager>("mapViz");
		LOG_INFO("Components created!");

		if (mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map done!");
		}
		else {
			LOG_INFO("Load map failed!");
			return 0;
		}

		SRef<Map> map;
		mapManager->getMap(map);

		const SRef<PointCloud>& pointCloud = map->getConstPointCloud();
		const SRef<KeyframeCollection>& keyframeCollection = map->getConstKeyframeCollection();

		LOG_INFO("map information:");
		LOG_INFO("Number of point cloud: {}", pointCloud->getNbPoints());
		LOG_INFO("Number of keyframes: {}", keyframeCollection->getNbKeyframes());

		// get point clouds and keyframes
		std::vector<SRef<CloudPoint>> cloudPoints;
		std::vector<SRef<Keyframe>> keyframes;
		keyframeCollection->getAllKeyframes(keyframes);
		pointCloud->getAllPoints(cloudPoints);

		std::vector<Transform3Df>kfPoses, fPoses;
		fPoses = {};
		for (const auto &kf : keyframes) {
			kfPoses.push_back(kf->getPose());
		}
		while (true) {
			if (viewer3D->display(cloudPoints, Transform3Df::Identity(), kfPoses, fPoses) == FrameworkReturnCode::_STOP)
				break;
		}
    return 0;
}
