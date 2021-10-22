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
#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"

#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;

#define INDEX_USE_CAMERA 0
#define NB_IMAGES_BETWEEN_RELOCALIZATION 10  // number of read images between 2 requests for relocalization
#define NB_RELOCALIZATION_REQUESTS 10        // number of requests for relocalization

// Global XPCF Component Manager
SRef<xpcf::IComponentManager> gXpcfComponentManager = 0;

// Global Mapping Pipeline Multithreads instance
SRef<pipeline::IMappingPipeline> gMappingPipelineMulti = 0;
// Global Relocalization Pipeline Multithreads instance
SRef<pipeline::IRelocalizationPipeline> gRelocalizationPipeline = 0;

// Global client threads
xpcf::DelegateTask * gClientMappingTask = 0;
xpcf::DelegateTask * gClientRelocalizationTask = 0;

// Components used by mapping client
SRef<display::IImageViewer> gImageViewer = 0;

// Indicate if the relocalization service has found the new pose
bool gRelocalizationSucceed = false;

// Transformation matrix from Hololen to World
Transform3Df T_M_W = Transform3Df::Identity();

// Drop buffers used by mapping client thread
xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferCamImagePoseMapping;
xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferCamImageRelocalization;

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

// Fonction for mapping client thread
auto fnClientMapping = []() {

    std::pair<SRef<Image>, Transform3Df> imagePose;

    // Try to get next (image, pose)
    if (m_dropBufferCamImagePoseMapping.tryPop(imagePose)) {

        LOG_INFO("Mapping client: Send (image, pose) to mapping pipeline");

        SRef<Image> image = imagePose.first;
        Transform3Df pose = imagePose.second;

        gMappingPipelineMulti->mappingProcessRequest(image, pose);

        gImageViewer->display(image);
    }
};

// Fonction for relocalization client thread
auto fnClientRelocalization = []() {

    std::pair<SRef<Image>, Transform3Df> imagePose;

    // Try to get next image
    if (m_dropBufferCamImageRelocalization.tryPop(imagePose)) {

        Transform3Df new_pose;
        float confidence;

        SRef<Image> image = imagePose.first;
        Transform3Df pose = imagePose.second;

        LOG_INFO("Relocalization client: Send image to relocalization pipeline");

        if (gRelocalizationPipeline->start() == SolAR::FrameworkReturnCode::_SUCCESS) {
            if (gRelocalizationPipeline->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
                LOG_INFO("=> Relocalization succeeded");

                LOG_INFO("Hololens pose: {}", pose.matrix());
                LOG_INFO("World pose: {}", new_pose.matrix());

                T_M_W = new_pose * pose.inverse();

                LOG_INFO("Transformation matrix from Hololens to World: {}", T_M_W.matrix());

                gRelocalizationSucceed = true;
            }

            gRelocalizationPipeline->stop();
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

    LOG_INFO("Stop mapping client thread");

    if (gClientMappingTask != 0)
        gClientMappingTask->stop();

    LOG_INFO("Stop relocalization client thread");

    if (gClientRelocalizationTask != 0)
        gClientRelocalizationTask->stop();

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

    cxxopts::Options option_list("SolARPipelineTest_Mapping_Multi_Relocalization_Remote",
                                 "SolARPipelineTest_Mapping_Multi_Relocalization_Remote - The commandline interface to the xpcf grpc client test application.\n");
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
        std::cout << "SolARPipelineTest_Mapping_Multi_Relocalization_Remote version " << MYVERSION << std::endl << std::endl;
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
        LOG_INFO("Load configuration file: {}", file);

        if (gXpcfComponentManager->load(file.c_str()) == org::bcom::xpcf::_SUCCESS)
        {
            LOG_INFO("Resolve IMappingPipeline interface");
            gMappingPipelineMulti = gXpcfComponentManager->resolve<SolAR::api::pipeline::IMappingPipeline>();

            LOG_INFO("Resolve IRelocalizationPipeline interface");
            gRelocalizationPipeline = gXpcfComponentManager->resolve<SolAR::api::pipeline::IRelocalizationPipeline>();
        }
        else {
            LOG_INFO("Failed to load configuration file: {}", file);
            return -1;
        }

        SolAR::FrameworkReturnCode result;

        SRef<input::devices::IARDevice> AR_device = gXpcfComponentManager->resolve<SolAR::api::input::devices::IARDevice>();
        LOG_INFO("Remote producer client: AR device component created");

        gImageViewer = gXpcfComponentManager->resolve<SolAR::api::display::IImageViewer>();
        LOG_INFO("Remote producer client: AR device component created");

        // Connect remotely to the HoloLens streaming app
        if (AR_device->start() == FrameworkReturnCode::_SUCCESS) {

            // Load camera intrinsics parameters
            CameraRigParameters camRigParams = AR_device->getCameraParameters();
            CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

            result = gMappingPipelineMulti->init();
            LOG_INFO("Remote mapping client: Init mapping pipeline result = {}",
                     getReturnCodeTextValue(result));

            if (result == FrameworkReturnCode::_ERROR_) {
                LOG_ERROR("Cannot initialize mapping pipeline");
                return -1;
            }

            result = gMappingPipelineMulti->setCameraParameters(camParams);
            LOG_INFO("Remote mapping client: Set mapping pipeline camera parameters result = {}",
                     getReturnCodeTextValue(result));

            result = gRelocalizationPipeline->init();
            LOG_INFO("Remote relocalization client: Init relocalization pipeline result = {}",
                     getReturnCodeTextValue(result));

            if (result == FrameworkReturnCode::_ERROR_) {
                LOG_ERROR("Cannot initialize relocalization pipeline");
                return -1;
            }

            result = gRelocalizationPipeline->setCameraParameters(camParams);
            LOG_INFO("Remote relocalization client: Set relocalization pipeline camera parameters result = {}",
                     getReturnCodeTextValue(result));

            LOG_INFO("Remote mapping client: Start remote mapping pipeline");

            if (gMappingPipelineMulti->start() == FrameworkReturnCode::_SUCCESS) {

                LOG_INFO("Start remote mapping client thread");

                gClientMappingTask  = new xpcf::DelegateTask(fnClientMapping);
                gClientMappingTask->start();

                LOG_INFO("Start remote relocalization client thread");

                gClientRelocalizationTask  = new xpcf::DelegateTask(fnClientRelocalization);
                gClientRelocalizationTask->start();

                LOG_INFO("Read images and poses from hololens files");

                LOG_INFO("\n\n***** Control+C to stop *****\n");

                uint32_t nbImagesRelocalization = NB_IMAGES_BETWEEN_RELOCALIZATION, nbRelocalizationRequests = 0;

                while (true) {

                    std::vector<SRef<Image>> images;
                    std::vector<Transform3Df> poses;
                    std::chrono::system_clock::time_point timestamp;

                    if (AR_device->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {

                        nbImagesRelocalization ++;

                        SRef<Image> image = images[INDEX_USE_CAMERA];
                        Transform3Df pose = poses[INDEX_USE_CAMERA];

//                        gImageViewer->display(image);

                        // Send images to relocalization service if less then 'n' images have been read
                        // and if relocalization is still not ok
                        if (!gRelocalizationSucceed && (nbImagesRelocalization > NB_IMAGES_BETWEEN_RELOCALIZATION)
                            && (nbRelocalizationRequests < NB_RELOCALIZATION_REQUESTS)){
                            nbImagesRelocalization = 0;
                            nbRelocalizationRequests ++;

                            LOG_INFO("Add image to input drop buffer for relocalization");
                            m_dropBufferCamImageRelocalization.push(std::make_pair(image, pose));
                        }

                        if ((nbRelocalizationRequests == NB_RELOCALIZATION_REQUESTS) && !gRelocalizationSucceed) {
                            LOG_INFO("=> Relocalization failed");
                            nbRelocalizationRequests ++;
                        }

                        // Send images to mapping service

                        LOG_INFO("Add pair (image, pose) to input drop buffer for mapping");
                        m_dropBufferCamImagePoseMapping.push(std::make_pair(image, pose * T_M_W));
                    }
                }
            }
            else {
                LOG_ERROR("Cannot start mapping pipeline");
                return -1;
            }
        }
        else {
            LOG_INFO("Cannot start AR device loader");
            return -1;
        }
    }
    catch (xpcf::Exception & e) {
        LOG_INFO("The following exception has been caught: {}", e.what());
        return -1;
    }

    return 0;
}
