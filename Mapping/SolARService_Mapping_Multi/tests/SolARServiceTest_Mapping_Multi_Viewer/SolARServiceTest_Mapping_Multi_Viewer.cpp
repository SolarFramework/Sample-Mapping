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
xpcf::DelegateTask * gClientViewerTask = 0;

// Viewer used by viewer client
SRef<api::display::I3DPointsViewer> gViewer3D = 0;

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
        if (gViewer3D->display(pointClouds, keyframePoses[keyframePoses.size()-1], keyframePoses, {}, {}) == FrameworkReturnCode::_STOP) {
            LOG_INFO("Viewer client: ESC pressed on viewer");

            LOG_INFO("End of test");

            exit(0);
        }

    }
    else {
        LOG_DEBUG("Viewer client: nothing to display");
    }
};

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop viewer client thread");

    if (gClientViewerTask != 0)
        gClientViewerTask->stop();

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

    cxxopts::Options option_list("SolARServiceTest_Mapping_Multi_Viewer",
                                 "SolARServiceTest_Mapping_Multi_Viewer - The commandline interface to the xpcf grpc client test application.\n");
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
        cout << "SolARServiceTest_Mapping_Multi_Viewer version 0.9.3 \n";
        cout << '\n';
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

        LOG_INFO("Start viewer client thread");

        gClientViewerTask  = new xpcf::DelegateTask(fnClientViewer);
        gClientViewerTask->start();

        LOG_INFO("\n\n***** Control+C to stop *****\n");

        // Wait for interruption
        while (true);
    }
    catch (xpcf::Exception & e) {
        LOG_INFO("The following exception has been caught: {}", e.what());
        return -1;
    }

    return 0;
}
