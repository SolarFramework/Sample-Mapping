/**
 * @copyright Copyright (c) 2020 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <xpcf/xpcf.h>
#include <boost/log/core.hpp>
#include "core/Log.h"
#include "api/pipeline/IMappingPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/input/files/ITrackableLoader.h"
#include "datastructure/FiducialMarker.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;

#define INDEX_USE_CAMERA 0

int main(int argc, char ** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    // Default configuration file
    char * config_file = (char *)"SolARPipelineTest_Mapping_Mono_conf.xml";

    if (argc > 1) {
        // Get mapping pipeline configuration file path and name from main args
        config_file = argv[1];
    }

    try {
        LOG_INFO("Get Component Manager instance");
		SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();
		xpcf::XPCFErrorCode errorLoad = componentMgr->load(config_file);
		if (errorLoad != xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", config_file);
			return -1;
		}		
		auto gMappingPipeline = componentMgr->resolve<pipeline::IMappingPipeline>();        
		LOG_INFO("Mapping pipeline created");		
		auto gArDevice = componentMgr->resolve<input::devices::IARDevice>();
		LOG_INFO("AR device component created");		
		auto trackableLoader = componentMgr->resolve<input::files::ITrackableLoader>();
		LOG_INFO("Trackable loader component created");		
		auto gImageViewer = componentMgr->resolve<display::IImageViewer>();
		LOG_INFO("Image viewer component created");
		auto g3DViewer = componentMgr->resolve<display::I3DPointsViewer>();
		LOG_INFO("3D viewer component created");
        // Start device
		if (gArDevice->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot start AR device loader");
			return -1;
		}
        // Load camera intrinsics parameters
        CameraParameters camParams;
        camParams = gArDevice->getParameters(INDEX_USE_CAMERA);
		// Set camera parameters
        gMappingPipeline->setCameraParameters(camParams);
        // Load trackable
        SRef<Trackable> trackableObject = trackableLoader->loadTrackable();
		// Check and set trackable
        if (trackableObject) {
            LOG_INFO("Fiducial marker created: url = {}", trackableObject->getURL());
            gMappingPipeline->setObjectToTrack(trackableObject);                        
        }
        else {
            LOG_ERROR("Error while loading fiducial marker");
            return -1;
        }

		if (gMappingPipeline->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot start mapping pipeline");
			return -1;
		}
		
		while (true) {
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (gArDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];
			// display image
			if (gImageViewer->display(image) == FrameworkReturnCode::_STOP) break;
			// mapping
			gMappingPipeline->mappingProcessRequest(image, pose);
			// get map
			std::vector<SRef<CloudPoint>> pointCloud;
			std::vector<Transform3Df> keyframePoses;
			if (gMappingPipeline->getDataForVisualization(pointCloud, keyframePoses) == FrameworkReturnCode::_SUCCESS) {
				if (g3DViewer->display(pointCloud, keyframePoses[keyframePoses.size() - 1], keyframePoses) == FrameworkReturnCode::_STOP)
					break;
			}
		}		
		
		std::vector<SRef<CloudPoint>> pointCloud;
		std::vector<Transform3Df> keyframePoses;
		if (gMappingPipeline->getDataForVisualization(pointCloud, keyframePoses) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Number of cloud points: {}", pointCloud.size());
			LOG_INFO("Number of keyframes: {}", keyframePoses.size());
			while (g3DViewer->display(pointCloud, keyframePoses[keyframePoses.size() - 1], keyframePoses) == FrameworkReturnCode::_SUCCESS);
		}
		gMappingPipeline->stop();
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return -1;
    }

    return 0;
}
