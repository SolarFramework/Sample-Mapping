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
#include <boost/timer.hpp>
#include <signal.h>

#include "core/Log.h"
#include "api/pipeline/IMappingPipeline.h"
#include "api/input/devices/IARDevice.h"
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
// Nb of images sent by producer client
int gNbImages = 0;
// Delay between to images sent
boost::timer delay_between_images;
// gRPC request duration
boost::timer grpc_request_duration;

// print help options
void print_help(const cxxopts::Options& options)
{
    cout << options.help({""}) << std::endl;
}

// print error message
void print_error(const string& msg)
{
    cerr << msg << std::endl;
}

// Fonction for producer client thread
auto fnClientProducer = []() {

    std::vector<SRef<Image>> images;
    std::vector<Transform3Df> poses;
    std::chrono::system_clock::time_point timestamp;

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
        if (gNbImages > 1) {
            LOG_DEBUG("Producer client: Delay between 2 images sent = {} ms", delay_between_images.elapsed() * 1000);
        }

        delay_between_images.restart();

        grpc_request_duration.restart();

        // Set image encoding for serialization
        image->setImageEncoding(Image::ENCODING_JPEG);
        image->setImageEncodingQuality(60);
        gMappingPipelineMulti->mappingProcessRequest(image, pose);

        LOG_DEBUG("Producer client: gRPC request for (image, pose) number {} takes {} ms",
                 gNbImages, grpc_request_duration.elapsed() * 1000);

        gImageViewer->display(image);
    }
    else {
        LOG_INFO("Producer client: no more images to send");

        LOG_INFO("Stop mapping pipeline process");

        if (gMappingPipelineMulti != 0)
            gMappingPipelineMulti->stop();

        LOG_INFO("End of test");

        exit(0);
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

    cxxopts::Options option_list("SolARServiceTest_Mapping_Multi_Producer",
                                 "SolARServiceTest_Mapping_Multi_Producer - The commandline interface to the xpcf grpc client test application.\n");
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
        std::cout << "SolARServiceTest_MapUpdate version " << MYVERSION << std::endl << std::endl;
        return 0;
    }
    else if (!options.count("file") || options["file"].as<string>().empty()) {
        print_error("missing one of file or database dir argument");
        return 1;
    }

    try {
        // Check if log level is defined in environment variable SOLAR_LOG_LEVEL
        char * log_level = getenv("SOLAR_LOG_LEVEL");
        std::string str_log_level = "INFO(default)";

        if (log_level != nullptr) {
            str_log_level = std::string(log_level);

            if (str_log_level == "DEBUG"){
                LOG_SET_DEBUG_LEVEL();
            }
            else if (str_log_level == "CRITICAL"){
                LOG_SET_CRITICAL_LEVEL();
            }
            else if (str_log_level == "ERROR"){
                LOG_SET_ERROR_LEVEL();
            }
            else if (str_log_level == "INFO"){
                LOG_SET_INFO_LEVEL();
            }
            else if (str_log_level == "TRACE"){
                LOG_SET_TRACE_LEVEL();
            }
            else if (str_log_level == "WARNING"){
                LOG_SET_WARNING_LEVEL();
            }
            else {
                LOG_ERROR ("'SOLAR_LOG_LEVEL' environment variable: invalid value");
                LOG_ERROR ("Expected values are: DEBUG, CRITICAL, ERROR, INFO, TRACE or WARNING");
            }

            LOG_DEBUG("Environment variable SOLAR_LOG_LEVEL={}", str_log_level);
        }

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

        gImageViewer = gXpcfComponentManager->resolve<SolAR::api::display::IImageViewer>();
        LOG_INFO("Remote producer client: AR device component created");

        // Connect remotely to the HoloLens streaming app
        if (gArDevice->start() == FrameworkReturnCode::_SUCCESS) {

            // Load camera intrinsics parameters
            CameraRigParameters camRigParams = gArDevice->getCameraParameters();
            CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

            result = gMappingPipelineMulti->init();
            LOG_INFO("Remote producer client: Init mapping pipeline result = {}",
                     getReturnCodeTextValue(result));

            result = gMappingPipelineMulti->setCameraParameters(camParams);
            LOG_INFO("Remote producer client: Set mapping pipeline camera parameters result = {}",
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
            LOG_INFO("Cannot start AR device loader");
            return -1;
        }

        LOG_INFO("\n\n***** Control+C to stop *****\n");

        // Wait for end of images or interruption
        while (true) {
        }
    }
    catch (xpcf::Exception & e) {
        LOG_INFO("The following exception has been caught: {}", e.what());
        return -1;
    }

    return 0;
}
