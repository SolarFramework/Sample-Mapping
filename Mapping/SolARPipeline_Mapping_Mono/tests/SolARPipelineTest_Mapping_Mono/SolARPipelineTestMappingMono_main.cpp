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
#include "core/Timer.h"
#include "xpcf/threading/DropBuffer.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::api::pipeline;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;

#define INDEX_USE_CAMERA 1
#define DELAY_BETWEEN_REQUESTS 2000
#define NB_IMAGE_BETWEEN_RELOC 5

// Global XPCF Component Manager
SRef<xpcf::IComponentManager> gXpcfComponentManager = 0;

// Global Mapping Pipeline instance
SRef<pipeline::IMappingPipeline> gMappingPipelineMono = 0;

// Global client threads
xpcf::DelegateTask * gClientProducerTask = 0;
xpcf::DelegateTask * gClientViewerTask = 0;
xpcf::DelegateTask * gRelocMarkerTask = 0;

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
std::atomic_int gNbImages = NB_IMAGE_BETWEEN_RELOC;
// Check reloc based on a marker
std::atomic_bool gIsReloc = false;
// Transformation matrix to the world coordinate system
Transform3Df gT_M_W = Transform3Df::Identity();
// Timer to manage delay between two requests to the Mapping service
Timer timer;
// data for visualization
std::vector<SRef<CloudPoint>> pointClouds;
std::vector<Transform3Df> keyframePoses;
// Drop buffer for reloc marker
xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> gDropBufferRelocalizationMarker;

// Function for reloc marker thread
auto fnRelocMarker = []() {
    std::pair<SRef<Image>, Transform3Df> imagePose;

    if (!gDropBufferRelocalizationMarker.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    SRef<Image> image = imagePose.first;
    Transform3Df pose = imagePose.second;
    Transform3Df new_pose;

    LOG_DEBUG("Relocalization marker processing");

    if (gTrackablePose->estimate(image, new_pose) == FrameworkReturnCode::_SUCCESS) {
        gT_M_W = new_pose * pose.inverse();
        LOG_INFO("Transform matrix:\n{}", gT_M_W.matrix());
        gIsReloc = true;
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }
};

// Function for producer client thread
auto fnClientProducer = []() {

    if (!gImageToSend) {
        xpcf::DelegateTask::yield();
        return;
    }

    std::vector<SRef<Image>> images;
    std::vector<Transform3Df> poses;
    std::chrono::system_clock::time_point timestamp;

    // Get data from hololens files
    if (gArDevice->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {
        SRef<Image> image = images[INDEX_USE_CAMERA];
        Transform3Df pose = poses[INDEX_USE_CAMERA];
        SRef<Image> displayImage = image->copy();

        if (gNbImages == NB_IMAGE_BETWEEN_RELOC){
            gDropBufferRelocalizationMarker.push(std::make_pair(image, pose));
            gNbImages = 0;
        }
        else
            gNbImages++;

        if (gIsReloc) {
            // send to mapping
            Transform3Df updateT_M_W;
            MappingStatus status;
            gMappingPipelineMono->mappingProcessRequest({image}, {pose}, gT_M_W, updateT_M_W, status);
            gT_M_W = updateT_M_W;
            switch (status) {
            case BOOTSTRAP:
                LOG_INFO("Bootstrap");
                break;
            case TRACKING_LOST:
                LOG_INFO("Tracking lost");
                break;
            case LOOP_CLOSURE:
                LOG_INFO("Loop closure");
                break;
            default:
                LOG_INFO("Mapping");
            }
            // draw pose
            g3DOverlay->draw(gT_M_W * pose, displayImage);
        }

        if (gImageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP) {
            gClientProducerTask->stop();
        }
    }
    else {
        gImageToSend = false;
        LOG_INFO("Producer client: no more images to send");
    }
};

// Function for viewer client thread
auto fnClientViewer = []() {
    if (timer.elapsed() > DELAY_BETWEEN_REQUESTS) {
        // Try to get point clouds and key frame poses to display
        if (gMappingPipelineMono->getDataForVisualization(pointClouds, keyframePoses) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Viewer client: get point cloud and keyframe poses");
        }
        timer.restart();
    }

    if (pointClouds.size() > 0) {
        if (gViewer3D == 0) {
            gViewer3D = gXpcfComponentManager->resolve<display::I3DPointsViewer>();
            LOG_INFO("Viewer client: I3DPointsViewer component created");
        }
        // Display
        if (gViewer3D->display(pointClouds, keyframePoses[keyframePoses.size() - 1], keyframePoses, {}, {}) == FrameworkReturnCode::_STOP) {
            gClientViewerTask->stop();
        }
    }
    else {
        LOG_DEBUG("Viewer client: nothing to display");
        xpcf::DelegateTask::yield();
    }
};

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop producer client thread");

    if (gClientProducerTask != 0)
        gClientProducerTask->stop();

    LOG_INFO("Stop mapping pipeline process");

    if (gMappingPipelineMono != 0)
        gMappingPipelineMono->stop();

    LOG_INFO("Stop viewer client thread");

    if (gClientViewerTask != 0)
        gClientViewerTask->stop();

    LOG_INFO("Stop reloc marker thread");

    if (gRelocMarkerTask != 0)
        gRelocMarkerTask->stop();

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
    char * config_file = (char *)"SolARPipelineTest_Mapping_Mono_Processing_conf.xml";

    if (argc > 1) {
        // Get mapping pipeline configuration file path and name from main args
        config_file = argv[1];
    }

    try {
        LOG_INFO("Get Component Manager instance");

        gXpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Load Mapping Pipeline configuration file");

        if (gXpcfComponentManager->load(config_file) == org::bcom::xpcf::_SUCCESS)
        {
            // Create Mapping Pipeline component
            gMappingPipelineMono = gXpcfComponentManager->resolve<pipeline::IMappingPipeline>();
            LOG_INFO("Mapping pipeline component created");
            // init
            if (gMappingPipelineMono->init() != FrameworkReturnCode::_SUCCESS)
                return -1;
        }
        else {
            LOG_ERROR("Failed to load the configuration file {}", config_file);
            return -1;
        }

        // Manage producer client thread
        if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Mono_Producer_conf.xml") == org::bcom::xpcf::_SUCCESS)
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
                gMappingPipelineMono->setCameraParameters(camParams);

                LOG_INFO("Producer client: Start mapping pipeline");

                if (gMappingPipelineMono->start() == FrameworkReturnCode::_SUCCESS) {
                    LOG_INFO("Start producer client thread");

                    gClientProducerTask  = new xpcf::DelegateTask(fnClientProducer);
                    gClientProducerTask->start();

                    gRelocMarkerTask  = new xpcf::DelegateTask(fnRelocMarker);
                    gRelocMarkerTask->start();
                }
                else {
                    LOG_ERROR("Cannot start mapping pipeline");
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
        if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Mono_Viewer_conf.xml") == org::bcom::xpcf::_SUCCESS)
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
