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
#include "xpcf/threading/BaseTask.h"
#include <iostream>
#include <boost/log/core.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>

#include "core/Log.h"
#include "api/pipeline/IMappingPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"
#include "api/display/I3DOverlay.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf = org::bcom::xpcf;

#define INDEX_USE_CAMERA 0

// Global XPCF Component Manager
SRef<xpcf::IComponentManager> gXpcfComponentManager = 0;

// Global Mapping Pipeline Multithreads instance
SRef<pipeline::IMappingPipeline> gMappingPipelineMulti = 0;

// Global client threads
xpcf::DelegateTask * gClientProducerTask = 0;
xpcf::DelegateTask * gClientViewerTask = 0;

// Viewer used by viewer client
SRef<api::display::I3DPointsViewer> gViewer3D = 0;

// Components used by producer client
SRef<input::devices::IARDevice> gArDevice = 0;
SRef<display::IImageViewer> gImageViewer = 0;
SRef<input::files::ITrackableLoader> gTrackableLoader = 0;
SRef<solver::pose::ITrackablePose> gTrackablePose = 0;
SRef<display::I3DOverlay> g3DOverlay = 0;
// Indicates if producer client has images to send to mapping pipeline
bool gImageToSend = true;
// Nb of images sent by producer client
int gNbImages = 0;
// Check reloc based on a marker
bool gIsReloc = false;
// Transformation matrix to the world coordinate system
Transform3Df gT_M_W = Transform3Df::Identity();

// Fonction for producer client thread
auto fnClientProducer = []() {

	std::vector<SRef<Image>> images;
	std::vector<Transform3Df> poses;
	std::chrono::system_clock::time_point timestamp;

	// Still images to process?
	if (gImageToSend) {
		// Get data from hololens files
		if (gArDevice->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {

			//            gNbImages ++;
			//            LOG_INFO("Producer client: Send (image, pose) num {} to mapping pipeline", gNbImages);

			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];
			SRef<Image> displayImage = image->copy();

			// find T_W_M
			if (!gIsReloc) {
				Transform3Df T_M_C;
				if (gTrackablePose->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
					gT_M_W = T_M_C * pose.inverse();
					gIsReloc = true;
				}
			}
			else {
				// correct pose
				pose = gT_M_W * pose;
				// send to mapping
				gMappingPipelineMulti->mappingProcessRequest(image, pose);
				// draw pose
				g3DOverlay->draw(pose, displayImage);
			}

			if (gImageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP) {
				gClientProducerTask->stop();
			}
		}
		else {
			gImageToSend = false;
			LOG_INFO("Producer client: no more images to send");
		}
	}
};

// Fonction for viewer client thread
auto fnClientViewer = []() {

	// Point clouds and keyframe poses used by client viewer
	std::vector<SRef<CloudPoint>> pointClouds;
	std::vector<Transform3Df> keyframePoses;

	// Try to get point clouds and key frame poses to display
	if (gMappingPipelineMulti->getDataForVisualization(pointClouds, keyframePoses) == FrameworkReturnCode::_SUCCESS) {

		LOG_DEBUG("Viewer client: get point cloud and keyframe poses");

		if (gViewer3D == 0) {
			gViewer3D = gXpcfComponentManager->resolve<display::I3DPointsViewer>();
			LOG_INFO("Viewer client: I3DPointsViewer component created");
		}

		// Display new data
		gViewer3D->display(pointClouds, keyframePoses[keyframePoses.size() - 1], keyframePoses, {}, {});

	}
	else {
		LOG_DEBUG("Viewer client: nothing to display");
	}
};

// Function called when interruption signal is triggered
static void SigInt(int signo) {

	LOG_INFO("\n\n===> Program interruption\n");

	LOG_INFO("Stop producer client thread");

	if (gClientProducerTask != 0)
		gClientProducerTask->stop();

	LOG_INFO("Stop viewer client thread");

	if (gClientViewerTask != 0)
		gClientViewerTask->stop();

	LOG_INFO("Stop mapping pipeline process");

	if (gMappingPipelineMulti != 0)
		gMappingPipelineMulti->stop();

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	LOG_INFO("End of test");

	exit(0);
}


///
/// \brief Test application for SolARMappingPipeline
///

int main(int argc, char ** argv)
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();
	
	// Signal interruption function (Ctrl + C)
	signal(SIGINT, SigInt);

	// Default configuration file
	char * config_file = (char *)"SolARPipelineTest_Mapping_Multi_NoDrop_Processing_conf.xml";

	if (argc > 1) {
		// Get mapping pipeline configuration file path and name from main args
		config_file = argv[1];
	}

	try {
		LOG_INFO("Get Component Manager instance");

		gXpcfComponentManager = xpcf::getComponentManagerInstance();

		LOG_INFO("Load Mapping Pipeline Multithreads configuration file");

		if (gXpcfComponentManager->load(config_file) == org::bcom::xpcf::_SUCCESS)
		{
			// Create Mapping Pipeline component
			gMappingPipelineMulti = gXpcfComponentManager->resolve<pipeline::IMappingPipeline>();
			LOG_INFO("Mapping pipeline multithreads component created");
			// init
			if (gMappingPipelineMulti->init() != FrameworkReturnCode::_SUCCESS)
				return -1;
		}
		else {
			LOG_ERROR("Failed to load the configuration file {}", config_file);
			return -1;
		}

		// Manage producer client thread
		if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Multi_NoDrop_Producer_conf.xml") == org::bcom::xpcf::_SUCCESS)
		{
			LOG_INFO("Producer client configuration file loaded");

			gArDevice = gXpcfComponentManager->resolve<input::devices::IARDevice>();
			LOG_INFO("Producer client: AR device component created");

			gImageViewer = gXpcfComponentManager->resolve<display::IImageViewer>();
			LOG_INFO("Producer client: Image viewer component created");

			gTrackableLoader = gXpcfComponentManager->resolve<input::files::ITrackableLoader>();
			LOG_INFO("Producer client: Trackable loader component created");

			gTrackablePose = gXpcfComponentManager->resolve<solver::pose::ITrackablePose>();
			LOG_INFO("Producer client: Trackable pose estimator component created");

			g3DOverlay = gXpcfComponentManager->resolve<display::I3DOverlay>();
			LOG_INFO("Producer client: 3D overlay component created");

			// set camera parameters
			CameraRigParameters camRigParams = gArDevice->getCameraParameters();
			CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];
			g3DOverlay->setCameraParameters(camParams.intrinsic, camParams.distortion);
			gTrackablePose->setCameraParameters(camParams.intrinsic, camParams.distortion);

			// Load and set Trackable
			SRef<Trackable> trackable;
			if (gTrackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
			{
				LOG_ERROR("Cannot load trackable");
				return -1;
			}
			else
			{
				if (gTrackablePose->setTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
				{
					LOG_ERROR("Cannot set trackable to trackable pose estimator");
					return -1;
				}
			}

			// Connect remotely to the HoloLens streaming app
			if (gArDevice->start() == FrameworkReturnCode::_SUCCESS) {

				// Load camera intrinsics parameters
				CameraRigParameters camRigParams = gArDevice->getCameraParameters();
				CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

				LOG_INFO("Producer client: Set mapping pipeline camera parameters");
				gMappingPipelineMulti->setCameraParameters(camParams);

				LOG_INFO("Producer client: Start mapping pipeline");

				if (gMappingPipelineMulti->start() == FrameworkReturnCode::_SUCCESS) {
					LOG_INFO("Start producer client thread");

					gClientProducerTask = new xpcf::DelegateTask(fnClientProducer);
					gClientProducerTask->start();
				}
				else {
					LOG_ERROR("Cannot start mapping pipeline");
				}
			}
			else {
				LOG_ERROR("Cannot start AR device loader");
				return -1;
			}
		}
		else {
			LOG_ERROR("Failed to load the producer client configuration file");
			return -1;
		}

		// Manage viewer client thread
		if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Multi_NoDrop_Viewer_conf.xml") == org::bcom::xpcf::_SUCCESS)
		{
			LOG_INFO("Viewer client configuration file loaded");

			LOG_INFO("Start viewer client thread");

			gClientViewerTask = new xpcf::DelegateTask(fnClientViewer);
			gClientViewerTask->start();
		}
		else {
			LOG_ERROR("Failed to load the viewer client configuration file");
			return -1;
		}

		LOG_INFO("\n\n***** Control+C to stop *****\n");

		// Wait for interruption
		while (true);

	}
	catch (xpcf::Exception & e) {
		LOG_ERROR("The following exception has been caught {}", e.what());
		return -1;
	}

	return 0;
}
