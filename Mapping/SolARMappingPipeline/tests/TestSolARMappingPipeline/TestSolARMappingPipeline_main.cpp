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

int main(int argc, char ** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    int nb_images = 0;

    // fonction for viewer client process
    auto fnClientViewer = [&]() {
        LOG_INFO("Client viewer: Get Component Manager instance");

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Client viewer: Load configuration file");

        if (xpcfComponentManager->load("TestSolARMappingPipeline_conf.xml") == org::bcom::xpcf::_SUCCESS)
        {
            // declare and create components
            LOG_INFO("Client viewer: Start creating components");

            auto mapping_pipeline = xpcfComponentManager->resolve<pipeline::IMappingPipeline>();
            LOG_INFO("Client viewer: Mapping pipeline component created");

            // Wait for bootstrap finished
            LOG_INFO("Client viewer: waiting for pose correction and boostrap");

            while (!mapping_pipeline->isBootstrapFinished()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }

            LOG_INFO("Client viewer: bootstrap finished => try to display mapping results");

            // display point cloud
            std::vector<SRef<CloudPoint>> pointClouds ;
            std::vector<Transform3Df> keyframePoses;

            auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
            LOG_INFO("Client viewer: I3DPointsViewer component created");

            while (true) {
                if (mapping_pipeline->getDataForVisualization(pointClouds, keyframePoses) == FrameworkReturnCode::_SUCCESS) {
                    viewer3D->display(pointClouds, keyframePoses[keyframePoses.size()-1], keyframePoses, {}, {});
                }

//                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }
        else {
            LOG_ERROR("Failed to load the configuration file TestSolARMappingPipeline_conf.xml", argv[1]);
        }

    };

    try {

        LOG_INFO("Client producer: Get Component Manager instance");

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Client producer: Load configuration file");

        if (xpcfComponentManager->load("TestSolARMappingPipeline_conf.xml") != org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file TestSolARMappingPipeline_conf.xml", argv[1])
            return -1;
        }

        // declare and create components
        LOG_INFO("Client producer: Start creating components");

        auto mapping_pipeline = xpcfComponentManager->resolve<pipeline::IMappingPipeline>();
        LOG_INFO("Client producer: Mapping pipeline component created");

        auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
        LOG_INFO("Client producer: AR device component created");

        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
        LOG_INFO("Client producer: Trackable loader component created");

        auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
        LOG_INFO("Client producer: Image viewer component created");

        // Start viewer client process
        xpcf::DelegateTask * clientViewerTask  = new xpcf::DelegateTask(fnClientViewer);
        clientViewerTask->start();

        LOG_INFO("Client producer: Init mapping pipeline");
        if (mapping_pipeline->init(xpcfComponentManager) == FrameworkReturnCode::_SUCCESS) {

            LOG_INFO("Client producer: Start AR device loader");
            // Connect remotely to the HoloLens streaming app
            if (arDevice->start() == FrameworkReturnCode::_ERROR_) {
                LOG_ERROR("Cannot start AR device loader");
                return -1;
            }

            // Load camera intrinsics parameters
            CameraParameters camParams;
            camParams = arDevice->getParameters(0);

            LOG_INFO("Client producer: Set mapping pipeline camera parameters");
            mapping_pipeline->setCameraParameters(camParams);

            LOG_INFO("Client producer: Load fiducial marker description file");
            Trackable * trackableObject = trackableLoader->loadTrackable();

            if (trackableObject != nullptr) {
                LOG_INFO("Client producer: Fiducial marker created: url = {}", trackableObject->getURL());

                LOG_INFO("Client producer: Set mapping pipeline fiducial marker");
                mapping_pipeline->setObjectToTrack(*trackableObject);

                delete trackableObject;

                LOG_INFO("Client producer: Process pose correction and bootstrap");

                while (!mapping_pipeline->isBootstrapFinished()) {
                    std::vector<SRef<Image>> images;
                    std::vector<Transform3Df> poses;
                    std::chrono::system_clock::time_point timestamp;

                    if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR ("Error while geting Hololens data");
                        return -1;
                    }

                    SRef<Image> image = images[INDEX_USE_CAMERA];
                    Transform3Df pose = poses[INDEX_USE_CAMERA];

                    if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
                        exit(0);

                    mapping_pipeline->correctPoseAndBootstrap(image, pose);
                }

                LOG_INFO("Client producer: Pose correction and bootstrap finished");

                LOG_INFO("Client producer: Start mapping pipeline");
                if (mapping_pipeline->start() == FrameworkReturnCode::_SUCCESS) {
                    LOG_INFO("Client producer: Start to send data from hololens to mapping pipeline");

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
                        LOG_INFO("Client producer: Send (image, pose) num {} to mapping pipeline", nb_images);

                        SRef<Image> image = images[INDEX_USE_CAMERA];
                        Transform3Df pose = poses[INDEX_USE_CAMERA];

                        mapping_pipeline->mappingProcessRequest(image, pose);

                        if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
                            exit(0);

//                        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                    }
                }
                else {
                    LOG_ERROR("Cannot start mapping pipeline");
                    return -1;
                }
            }
            else {
                LOG_ERROR ("Error while loading fiducial marker");
                return -1;
            }
        }

        LOG_INFO("Client producer: no more hololens images to send");

        while(true) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }

        LOG_INFO("Client producer: Stop mapping pipeline");
        mapping_pipeline->stop();

        clientViewerTask->stop();
        delete clientViewerTask;
    }
    catch (xpcf::Exception e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return -1;
    }

    return 0;
}
