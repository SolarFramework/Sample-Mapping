// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <iostream>

#include <unistd.h>
#include <cxxopts.hpp>

#include <xpcf/xpcf.h>
#include "xpcf/threading/BaseTask.h"
#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <boost/thread/thread.hpp>
#include <boost/log/core.hpp>
#include <signal.h>

#include "core/Log.h"
#include "api/pipeline/IMappingPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/input/files/ITrackableLoader.h"
#include "datastructure/FiducialMarker.h"
#include "api/display/IImageViewer.h"

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

// Components used by producer client
SRef<input::devices::IARDevice> gArDevice = 0;
SRef<display::IImageViewer> gImageViewer = 0;
// Indicates if producer client has images to send to mapping pipeline
bool gImageToSend = true;
// Nb of images sent by producer client
int gNbImages = 0;

// print help options
void print_help(const cxxopts::Options& options)
{
    cout << options.help({""}) << '\n';
}

// print error message
void print_error(const string& msg)
{
    cerr << msg << '\n';
}

// Fonction for producer client thread
auto fnClientProducer = []() {

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
/*
            LOG_DEBUG("Image buffer size = {}", image->getBufferSize());
            LOG_DEBUG("Image layout = {}", image->getImageLayout());
            LOG_DEBUG("Image pixel order = {}", image->getPixelOrder());
            LOG_DEBUG("Image data type = {}", image->getDataType());
            LOG_DEBUG("Image nb channels = {}", image->getNbChannels());
            LOG_DEBUG("Image nb bits per component = {}", image->getNbBitsPerComponent());
            LOG_DEBUG("Image size = {},{}", image->getWidth(), image->getHeight());
            LOG_DEBUG("Image step = {}", image->getStep());

            LOG_DEBUG("Pose rows/cols = {}/{}", pose.rows(), pose.cols());
*/
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

// Return SolAR Framework return code text value
string getReturnCodeTextValue (SolAR::FrameworkReturnCode return_code) {

    string text_value = "";

    switch(return_code) {
        case SolAR::FrameworkReturnCode::_STOP:
            text_value.resize(5);
            text_value = "STOP";
            break;
        case SolAR::FrameworkReturnCode::_SUCCESS:
            text_value.resize(8);
            text_value = "SUCCESS";
            break;
        case SolAR::FrameworkReturnCode::_ERROR_:
            text_value.resize(6);
            text_value = "ERROR";
            break;
        case SolAR::FrameworkReturnCode::_NOT_IMPLEMENTED:
            text_value.resize(16);
            text_value = "NOT IMPLEMENTED";
            break;
        case SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE:
            text_value.resize(17);
            text_value = "ERROR LOAD IMAGE";
            break;
        case SolAR::FrameworkReturnCode::_ERROR_ACCESS_IMAGE:
            text_value.resize(19);
            text_value = "ERROR ACCESS IMAGE";
            break;
        default:
            text_value.resize(8);
            text_value = "UNKNOWN";
            break;
    }

    return text_value;
}

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop producer client thread");

    if (gClientProducerTask != 0)
        gClientProducerTask->stop();

    LOG_INFO("Stop mapping pipeline process");

    if (gMappingPipelineMulti != 0)
        gMappingPipelineMulti->stop();

    LOG_INFO("End of test");

    exit(0);
}

int main(int argc, char* argv[])
{
    #if NDEBUG
        boost::log::core::get()->set_logging_enabled(false);
    #endif

    LOG_ADD_LOG_TO_CONSOLE();

    // Signal interruption function (Ctrl + C)
    signal(SIGINT, SigInt);

    cxxopts::Options option_list("SolARPipelineTest_Mapping_Multi_Remote_Producer",
                                 "SolARPipelineTest_Mapping_Multi_Remote_Producer - The commandline interface to the xpcf grpc client test application.\n");
    option_list.add_options()
            ("h,help", "display this help and exit")
            ("v,version", "display version information and exit")
            ("f,file", "xpcf grpc client configuration file",
             cxxopts::value<string>());

    auto options = option_list.parse(argc, argv);
    if (options.count("help")) {
        print_help(option_list);
        return 0;
    }
    else if (options.count("version"))
    {
        cout << "SolARPipelineTest_Mapping_Multi_Remote_Producer version 0.9.3 \n";
        cout << '\n';
        return 0;
    }
    else if (!options.count("file") || options["file"].as<string>().empty()) {
        print_error("missing one of file or database dir argument");
        return 1;
    }

    try {

        LOG_INFO("Get component manager instance");
        gXpcfComponentManager = xpcf::getComponentManagerInstance();

        string file = options["file"].as<string>();
        LOG_INFO("Load Client Remote Mapping Pipeline configuration file: {}", file);

        if (gXpcfComponentManager->load(file.c_str()) == org::bcom::xpcf::_SUCCESS)
        {
            LOG_INFO("Resolve IMappingPipeline interface");
            gMappingPipelineMulti = gXpcfComponentManager->resolve<SolAR::api::pipeline::IMappingPipeline>();
        }
        else {
            LOG_INFO("Failed to load Client Remote Mapping Pipeline configuration file: {}", file);
            return -1;
        }

        SolAR::FrameworkReturnCode result;

        gArDevice = gXpcfComponentManager->resolve<SolAR::api::input::devices::IARDevice>();
        LOG_INFO("Remote producer client: AR device component created");

        auto trackableLoader = gXpcfComponentManager->resolve<SolAR::api::input::files::ITrackableLoader>();
        LOG_INFO("Remote producer client: Trackable loader component created");

        gImageViewer = gXpcfComponentManager->resolve<SolAR::api::display::IImageViewer>();
        LOG_INFO("Remote producer client: AR device component created");

        // Connect remotely to the HoloLens streaming app
        if (gArDevice->start() == FrameworkReturnCode::_SUCCESS) {

            // Load camera intrinsics parameters
            CameraParameters camParams;
            camParams = gArDevice->getParameters(0);

            result = gMappingPipelineMulti->setCameraParameters(camParams);
            LOG_INFO("Remote producer client: Set mapping pipeline camera parameters result = {}",
                     getReturnCodeTextValue(result));

            LOG_INFO("Remote producer client: Load fiducial marker description file");
            SRef<Trackable> trackableObject;
            if (trackableLoader->loadTrackable(trackableObject) != FrameworkReturnCode::_SUCCESS)
            {
                LOG_INFO("Cannot load fiducial marker");
                return -1;
            }

            if (trackableObject != 0) {
                LOG_INFO("Remote producer client: Trackable object created: url = {}", trackableObject->getURL());

                result = gMappingPipelineMulti->setObjectToTrack(trackableObject);
                LOG_INFO("Remote producer client: Set mapping pipeline fiducial marker result = {}",
                         getReturnCodeTextValue(result));

                LOG_INFO("Remote producer client: Start remote mapping pipeline");

                if (gMappingPipelineMulti->start() == FrameworkReturnCode::_SUCCESS) {
                    LOG_INFO("Start remote producer client thread");

                    gClientProducerTask  = new xpcf::DelegateTask(fnClientProducer);
                    gClientProducerTask->start();
                }
                else {
                    LOG_ERROR("Cannot start mapping pipeline");
                }
            }
            else {
                LOG_INFO("Error while loading fiducial marker");
                return -1;
            }

        }
        else {
            LOG_INFO("Cannot start AR device loader");
            return -1;
        }

        LOG_INFO("\n\n***** Control+C to stop *****\n");

        // Wait for end of images or interruption
        while (gImageToSend) {

        }

        LOG_INFO("Stop producer client thread");

        if (gClientProducerTask != 0)
            gClientProducerTask->stop();

        LOG_INFO("Stop mapping pipeline processing");

        if (gMappingPipelineMulti != 0)
            gMappingPipelineMulti->stop();

        LOG_INFO("End of test");

    }
    catch (xpcf::Exception & e) {
        LOG_INFO("The following exception has been caught: {}", e.what());
        return -1;
    }

    return 0;
}
