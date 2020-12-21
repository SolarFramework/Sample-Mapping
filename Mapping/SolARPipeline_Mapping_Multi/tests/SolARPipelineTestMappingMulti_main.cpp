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

// Global XPCF Component Manager
SRef<xpcf::IComponentManager> gXpcfComponentManager = 0;

// Global Mapping Pipeline Multithreads instance
SRef<pipeline::IMappingPipeline> gMappingPipelineMulti = 0;

// Global client threads
xpcf::DelegateTask * gClientProducerTask = 0;
xpcf::DelegateTask * gClientViewerTask = 0;

// Viewer used by viewer client
SRef<api::display::I3DPointsViewer> gViewer3D = 0;
// Point clouds and keyframe poses used by client viewer
std::vector<SRef<CloudPoint>> gPointClouds;
std::vector<Transform3Df> gKeyframePoses;

// Components used by producer client
SRef<input::devices::IARDevice> gArDevice = 0;
SRef<display::IImageViewer> gImageViewer = 0;
// Indicates if producer client has images to send to mapping pipeline
bool gImageToSend = true;
// Nb of images sent by producer client
int gNbImages = 0;

// Metrics global var
LARGE_INTEGER frequency;
LARGE_INTEGER start_time;
LARGE_INTEGER end_time;
double interval;
int nb_keyframePoses = 0;
bool display_metric = false;


// Fonction for producer client thread
auto fnClientProducer = [&]() {

    std::vector<SRef<Image>> images;
    std::vector<Transform3Df> poses;
    std::chrono::system_clock::time_point timestamp;

    // Still images to process?
    if (gImageToSend) {
        // Get data from hololens files
        if (gArDevice->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {

            gNbImages ++;
            LOG_DEBUG("Producer client: Send (image, pose) num {} to mapping pipeline", gNbImages);

            SRef<Image> image = images[INDEX_USE_CAMERA];
            Transform3Df pose = poses[INDEX_USE_CAMERA];

            gMappingPipelineMulti->mappingProcessRequest(image, pose);

            if (gImageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP) {
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
auto fnClientViewer = [&]() {

    // Try to get point clouds and key frame poses to display
    if (gMappingPipelineMulti->getDataForVisualization(gPointClouds, gKeyframePoses) == FrameworkReturnCode::_SUCCESS) {

        if (gViewer3D == 0) {
            gViewer3D = gXpcfComponentManager->resolve<display::I3DPointsViewer>();
            LOG_INFO("Viewer client: I3DPointsViewer component created");
        }

        // Display new data
        gViewer3D->display(gPointClouds, gKeyframePoses[gKeyframePoses.size()-1], gKeyframePoses, {}, {});

        // For time processing calculation
        if (gKeyframePoses.size() > nb_keyframePoses) {
            nb_keyframePoses = gKeyframePoses.size();
            display_metric = true;
        }
        else {
            if ((!gImageToSend) && (display_metric)) {
                // End of processing
                QueryPerformanceCounter(&end_time);
                interval = (double) (end_time.QuadPart - start_time.QuadPart) / frequency.QuadPart;
                LOG_INFO("Processing time = {}\n", interval);
                display_metric = false;
            }
        }
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

    // Processing start time
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start_time);

    LOG_ADD_LOG_TO_CONSOLE();

    // Signal interruption function (Ctrl + C)
    signal(SIGINT, SigInt);

    // Default configuration file
    char * config_file = (char *)"SolARPipelineTest_Mapping_Multi_Processing_conf.xml";

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
        }
        else {
            LOG_ERROR("Failed to load the configuration file {}", config_file);
            return -1;
        }

        // Manage producer client thread
        if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Multi_Producer_conf.xml") == org::bcom::xpcf::_SUCCESS)
        {
            LOG_INFO("Producer client configuration file loaded");

            gArDevice = gXpcfComponentManager->resolve<input::devices::IARDevice>();
            LOG_INFO("Producer client: AR device component created");

            auto trackableLoader = gXpcfComponentManager->resolve<input::files::ITrackableLoader>();
            LOG_INFO("Producer client: Trackable loader component created");

            gImageViewer = gXpcfComponentManager->resolve<display::IImageViewer>();
            LOG_INFO("Producer client: Image viewer component created");

            // Connect remotely to the HoloLens streaming app
            if (gArDevice->start() == FrameworkReturnCode::_SUCCESS) {

                // Load camera intrinsics parameters
                CameraParameters camParams;
                camParams = gArDevice->getParameters(0);

                LOG_INFO("Producer client: Set mapping pipeline camera parameters");
                gMappingPipelineMulti->setCameraParameters(camParams);

                LOG_INFO("Producer client: Load fiducial marker description file");
                SRef<Trackable> trackableObject = trackableLoader->loadTrackable();

                if (trackableObject != 0) {
                    LOG_INFO("Producer client: Fiducial marker created: url = {}", trackableObject->getURL());

                    LOG_INFO("Producer client: Set mapping pipeline fiducial marker");
                    gMappingPipelineMulti->setObjectToTrack(trackableObject);

                    LOG_INFO("Producer client: Start mapping pipeline");

                    if (gMappingPipelineMulti->start() == FrameworkReturnCode::_SUCCESS) {
                        LOG_INFO("Start producer client thread");

                        gClientProducerTask  = new xpcf::DelegateTask(fnClientProducer);
                        gClientProducerTask->start();
                    }
                    else {
                        LOG_ERROR("Cannot start mapping pipeline");
                    }
                }
                else {
                    LOG_ERROR("Error while loading fiducial marker");
                    return -1;
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
        if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Multi_Viewer_conf.xml") == org::bcom::xpcf::_SUCCESS)
        {
            LOG_INFO("Viewer client configuration file loaded");

            LOG_INFO("Start viewer client thread");

            gClientViewerTask  = new xpcf::DelegateTask(fnClientViewer);
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
