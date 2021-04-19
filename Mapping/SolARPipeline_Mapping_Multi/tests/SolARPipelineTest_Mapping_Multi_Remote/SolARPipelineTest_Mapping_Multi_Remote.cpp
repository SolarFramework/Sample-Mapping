// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <iostream>

#include <unistd.h>
#include <cxxopts.hpp>

#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>

#include "api/pipeline/IMappingPipeline.h"

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
    cxxopts::Options option_list("xpcf_grpc_client",
                                 "xpcf_grpc_client - The commandline interface to the xpcf grpc client test application.\n");
    option_list.add_options()
            ("h,help", "display this help and exit")
            ("v,version", "display version information and exit")
            ("f,file", "xpcf grpc client configuration file",
             cxxopts::value<std::string>());

    auto options = option_list.parse(argc, argv);
    if (options.count("help"))
        print_help(option_list);
    else if (options.count("version"))
    {
        std::cout << "xpcf_grpc_client version MYVERSION \n";
        std::cout << '\n';
    }
    else if (!options.count("file") || options["file"].as<std::string>().empty()) {
        print_error("missing one of file or database dir argument");
        return 1;
    }

    SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();
    std::string file = options["file"].as<std::string>();
    cmpMgr->load(file.c_str());

    auto pipeline_mapping = cmpMgr->resolve<SolAR::api::pipeline::IMappingPipeline>();

    pipeline_mapping->start();

    sleep(3);

    pipeline_mapping->stop();

    return 0;
}
