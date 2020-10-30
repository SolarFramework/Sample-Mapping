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
#include <iostream>
#include <boost/log/core.hpp>

#include "core/Log.h"
#include "api/input/devices/ICamera.h"
#include "api/input/files/ITrackableLoader.h"
#include "datastructure/FiducialMarker.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;


int main(int argc, char ** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif
    LOG_ADD_LOG_TO_CONSOLE();

    FiducialMarker *fiducialMarker = nullptr; // Fiducial marker trackable object
    SRef<Image> camImage; // Current camera image

    try {
        LOG_INFO("Get Component Manager instance");

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Load configuration file");

        if(xpcfComponentManager->load("TestSolARMappingPipeline_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file TestSolARMappingPipeline_conf.xml", argv[1])
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");

        auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
        LOG_INFO("Camera component created");

        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
        LOG_INFO("Trackable loader component created");

        LOG_INFO("Load fiducial marker description file");

        Trackable * trackableObject = trackableLoader->loadTrackable();

        if ((trackableObject != nullptr) && (trackableObject->getType() == FIDUCIAL_MARKER)) {
            LOG_INFO("Fiducial marker created");

            LOG_INFO("Trackable object UUID = {}", trackableObject->getUUID());
            LOG_INFO("Trackable object type = {}", trackableObject->getType());

            fiducialMarker = dynamic_cast<FiducialMarker *>(trackableObject);

            LOG_INFO("Fiducial marker width/height = {}/{}", fiducialMarker->getWidth(), fiducialMarker->getHeight());
        }
        else {
            LOG_ERROR("Failed to create fiducial marker object")
            return -1;
        }

        LOG_INFO("Get camera configuration from yml file and start it");
        if (camera->start() != FrameworkReturnCode::_SUCCESS) // Camera
        {
            LOG_ERROR ("Error while configuring and starting camera");

            if (fiducialMarker != nullptr)
                delete fiducialMarker;

            return -1;
        }

        // Get images from camera in loop
        while (true)
        {
            if (camera->getNextImage(camImage) == SolAR::FrameworkReturnCode::_ERROR_)
                break;
        }

        if (fiducialMarker != nullptr)
            delete fiducialMarker;

    }
    catch (xpcf::Exception e) {
        LOG_ERROR("The following exception has been catch {}", e.what());

        if (fiducialMarker != nullptr)
            delete fiducialMarker;

        return -1;
    }

    return 0;
}
