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

// Global Mapping Pipeline instance
SRef<pipeline::IMappingPipeline> gMappingPipeline = 0;

///
/// \brief Test application for SolARMappingPipeline
///

int main(int argc, char ** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    // fonction for viewer client thread
    auto fnClientViewer = [&]() {

        try {
            LOG_INFO("Viewer client: Get Component Manager instance");

            SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

            LOG_INFO("Viewer client: Load configuration file");

            if (xpcfComponentManager->load("TestSolARMappingPipelineViewer_conf.xml") == org::bcom::xpcf::_SUCCESS)
            {
                LOG_INFO("Configuration loaded");

                SRef<api::display::I3DPointsViewer> viewer3D = 0;

                // display point cloud
                std::vector<SRef<CloudPoint>> pointClouds ;
                std::vector<Transform3Df> keyframePoses;

                LOG_INFO("Viewer client: wait for end of bootstrap");

                // Display mapping pipeline results
                while (true) {

                    if (gMappingPipeline->getDataForVisualization(pointClouds, keyframePoses) == FrameworkReturnCode::_SUCCESS) {

                        if (viewer3D == 0) {
                            viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
                            LOG_INFO("Viewer client: I3DPointsViewer component created");
                        }

                        viewer3D->display(pointClouds, keyframePoses[keyframePoses.size()-1], keyframePoses, {}, {});
                    }
                }
            }
            else {
                LOG_ERROR("Failed to load the configuration file TestSolARMappingPipelineViewer_conf.xml");
            }
        }
        catch (xpcf::Exception e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    };

    // fonction for producer client thread
    auto fnClientProducer = [&]() {

        try {
            int nb_images = 0;

            LOG_INFO("Producer client: Get Component Manager instance");

            SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

            LOG_INFO("Producer client: Load configuration file");

            if (xpcfComponentManager->load("TestSolARMappingPipelineProducer_conf.xml") == org::bcom::xpcf::_SUCCESS)
            {
                // declare and create components
                LOG_INFO("Producer client: Start creating components");

                auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
                LOG_INFO("Producer client: AR device component created");

                auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
                LOG_INFO("Producer client: Trackable loader component created");

                auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
                LOG_INFO("Producer client: Image viewer component created");

                LOG_INFO("Producer client: Start AR device loader");

                // Connect remotely to the HoloLens streaming app
                if (arDevice->start() == FrameworkReturnCode::_SUCCESS) {

                    // Load camera intrinsics parameters
                    CameraParameters camParams;
                    camParams = arDevice->getParameters(0);

                    LOG_INFO("Producer client: Set mapping pipeline camera parameters");
                    gMappingPipeline->setCameraParameters(camParams);

                    LOG_INFO("Producer client: Load fiducial marker description file");
                    SRef<Trackable> trackableObject = trackableLoader->loadTrackable();

                    if (trackableObject != 0) {
                        LOG_INFO("Producer client: Fiducial marker created: url = {}", trackableObject->getURL());

                        LOG_INFO("Producer client: Set mapping pipeline fiducial marker");
                        gMappingPipeline->setObjectToTrack(trackableObject);

                        LOG_INFO("Producer client: Start mapping pipeline");

                        if (gMappingPipeline->start() == FrameworkReturnCode::_SUCCESS) {
                            LOG_INFO("Producer client: Start to send data from hololens to mapping pipeline");

                            while (true) {
                                std::vector<SRef<Image>> images;
                                std::vector<Transform3Df> poses;
                                std::chrono::system_clock::time_point timestamp;

                                // Get data from hololens files
                                if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
                                    LOG_ERROR ("Error while geting Hololens data");
                                    break;
                                }

                                nb_images ++;
//                                    LOG_INFO("Producer client: Send (image, pose) num {} to mapping pipeline", nb_images);

                                SRef<Image> image = images[INDEX_USE_CAMERA];
                                Transform3Df pose = poses[INDEX_USE_CAMERA];

                                gMappingPipeline->mappingProcessRequest(image, pose);

                                if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
                                    exit(0);

                                // Delay between 2 image
                                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                            }

                            LOG_INFO("Producer client: no more hololens images to send");
                        }
                        else {
                            LOG_ERROR("Cannot start mapping pipeline");
                        }
                    }
                    else {
                        LOG_ERROR ("Error while loading fiducial marker");
                    }
                }
                else {
                    LOG_ERROR("Cannot start AR device loader");
                }

                LOG_INFO("Producer client: end of thread");

                // Wait...
                while(true);
            }
            else {
                LOG_ERROR("Failed to load the configuration file TestSolARMappingPipelineProducer_conf.xml");
            }

        }
        catch (xpcf::Exception e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    };

    // Main code

    if (argc > 1) {
        // Get configuration file path and name from main args
        char * config_file = argv[1];

        try {
            // Get unique component manager instance
            SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

            LOG_INFO("Load Mapping Pipeline configuration file");

            if (xpcfComponentManager->load(config_file) == org::bcom::xpcf::_SUCCESS)
            {
                // Create Mapping Pipeline component
                gMappingPipeline = xpcfComponentManager->resolve<pipeline::IMappingPipeline>();

                LOG_INFO("Mapping pipeline component created");

                LOG_INFO("Start clients...");

                // Start producer client process
                xpcf::DelegateTask * clientProducerTask  = new xpcf::DelegateTask(fnClientProducer);
                clientProducerTask->start();

                // Start viewer client process
                xpcf::DelegateTask * clientViewerTask  = new xpcf::DelegateTask(fnClientViewer);
                clientViewerTask->start();

                // Wait...
                while (true);
            }
            else {
                LOG_ERROR("Failed to load the configuration file {}", config_file);
            }
        }
        catch (xpcf::Exception e) {
            LOG_ERROR("The following exception has been caught {}", e.what());
        }
    }
    else {
        LOG_ERROR("No configuration file given in arguments");
    }

    return 0;
}
