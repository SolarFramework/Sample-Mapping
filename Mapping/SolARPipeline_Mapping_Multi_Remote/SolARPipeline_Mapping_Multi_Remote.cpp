// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <iostream>

#include <cxxopts.hpp>
#include <boost/log/core.hpp>

#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include "GrpcServerManager.h"
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/filesystem/detail/utf8_codecvt_facet.hpp>
#include "core/Log.h"

using namespace SolAR;

namespace fs = boost::filesystem;

namespace xpcf = org::bcom::xpcf;

// print help options
void print_help(const cxxopts::Options& options)
{
    std::cout << options.help({""}) << '\n';
}

// print error message
void print_error(const std::string& msg)
{
    std::cerr << msg << '\n';
}

int main(int argc, char* argv[])
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    fs::detail::utf8_codecvt_facet utf8;
    SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();
    cmpMgr->bindLocal<xpcf::IGrpcServerManager,xpcf::GrpcServerManager>();
    std::string configSrc;
    fs::path currentPath(boost::filesystem::initial_path().generic_string(utf8));
    configSrc = currentPath.generic_string(utf8);

    cxxopts::Options option_list("SolARPipeline_Mapping_Multi_Remote",
                                 "SolARPipeline_Mapping_Multi_Remote - The commandline interface to the xpcf grpc server application.\n");
    option_list.add_options()
            ("h,help", "display this help and exit")
            ("v,version", "display version information and exit")
            ("m,modules", "XPCF modules configuration file",
             cxxopts::value<std::string>())
            ("p,properties", "XPCF properties configuration file",
             cxxopts::value<std::string>());

    auto options = option_list.parse(argc, argv);
    if (options.count("help")) {
        print_help(option_list);
        return 0;
    }
    else if (options.count("version"))
    {
        std::cout << "SolARPipeline_Mapping_Multi_Remote version 0.9.3 \n";
        std::cout << '\n';
        return 0;
    }
    else if ((!options.count("modules") || options["modules"].as<std::string>().empty())
          || (!options.count("properties") || options["properties"].as<std::string>().empty())) {
        print_error("missing one of modules (-m) or properties (-p) argument");
        return -1;
    }

    configSrc = options["modules"].as<std::string>();

    std::cout << "Load modules configuration file: " << configSrc << "\n";

    cmpMgr->load(configSrc.c_str());

    configSrc = options["properties"].as<std::string>();

    std::cout << "Load properties configuration file: " << configSrc << "\n";

    cmpMgr->load(configSrc.c_str());

    auto serverMgr = cmpMgr->resolve<xpcf::IGrpcServerManager>();
    char * serverURL = getenv("XPCF_GRPC_SERVER_URL");
    if (serverURL != nullptr) {
        serverMgr->bindTo<xpcf::IConfigurable>()->getProperty("server_address")->setStringValue(serverURL);
    }
    else {
        std::cout<<"No 'XPCF_GRPC_SERVER_URL' environment variable found: ";
    }
    std::cout<<"xpcf_grpc_server listens on: "<<serverMgr->bindTo<xpcf::IConfigurable>()->getProperty("server_address")->getStringValue()<<std::endl;
    serverMgr->runServer();
    return 0;
}
