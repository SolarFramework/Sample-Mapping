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

#define INDEX_USE_CAMERA 0
#define DELAY_BETWEEN_REQUESTS 500
#define NB_IMAGE_BETWEEN_RELOC 5
#define NB_MAX_FID 10

// groundtruth related
struct groundtruth {
    groundtruth() {}
    groundtruth(int i, const Transform3Df& t): marker_id(i), transform(t) {}
    int marker_id;
    Transform3Df transform;
};
std::map<std::chrono::system_clock::time_point, groundtruth> gGT; // map from timestamp to gt 
std::array<std::vector<std::chrono::system_clock::time_point>, NB_MAX_FID> gGTActivateTimestamps;  // activated timestamps of each FID
std::array<bool, NB_MAX_FID> gGTPoseInjected;
std::vector<Transform3Df> gFIDtransforms; // FID transform to the reference one 

// method split string into sub ones 
vector<string> ssplit(const string& str, string delimiter = " ") {
    vector<string> out;
    string strtmp = str;
    auto pos = strtmp.find(delimiter);
    while (pos != string::npos) {
        out.push_back(strtmp.substr(0, pos));
        strtmp.erase(0, pos+1);
        pos = strtmp.find(delimiter);
    }
    out.push_back(strtmp);
    return out;
}

MappingStatus gMappingStatus = BOOTSTRAP; 

// Global XPCF Component Manager
SRef<xpcf::IComponentManager> gXpcfComponentManager = 0;

// Global Mapping Pipeline Multithreads instance
SRef<pipeline::IMappingPipeline> gMappingPipelineMulti = 0;

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
// Camera parameters
CameraParameters camParams;

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
	if (gMappingStatus != BOOTSTRAP)
		return;
	if (gTrackablePose->estimate(image, camParams, new_pose) == FrameworkReturnCode::_SUCCESS) {
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
			if (gMappingStatus == BOOTSTRAP) {
				if (gGT.empty())
					gDropBufferRelocalizationMarker.push(std::make_pair(image, pose));
				else {
					auto gtData = gGT.find(timestamp);
					if (gtData != gGT.end()) // groundtruth found
						if (gtData->second.marker_id == 0) // reference FID (FID_0 is used by Bootstrap)
						    gDropBufferRelocalizationMarker.push(std::make_pair(image, pose));
				}
			}
            gNbImages = 0;
        }
        else
            gNbImages++;

        if (gIsReloc) {
            // check if current data contains FID other than the reference one 
            auto gtData = gGT.find(timestamp);
            bool other_marker_found = gtData != gGT.end() && gtData->second.marker_id > 0;

            // send to mapping
            Transform3Df poseCamera;
			Transform3Df updateT_M_W = gT_M_W;
            MappingStatus status;
            if (!other_marker_found) {
                gMappingPipelineMulti->mappingProcessRequest({image}, {pose}, /* fixedPose = */ false, gT_M_W, updateT_M_W, status);
            }
            else {
                // other FID found, get its id 
                int other_fid = gtData->second.marker_id;
                if (!gGTPoseInjected[other_fid]) {
					// check if current frame is activated for GT pose estimation 
					const auto& fid_acti_map = gGTActivateTimestamps[other_fid];
					if (std::find(fid_acti_map.begin(), fid_acti_map.end(), timestamp) != fid_acti_map.end()) { // if activated 
						Transform3Df pose_fid;
						if (gTrackablePose->estimate(image, camParams, pose_fid) == FrameworkReturnCode::_SUCCESS) {
							// GT camera pose in SolAR CS (1st FID CS)
							auto gt_pose_solar = gtData->second.transform * pose_fid;
							// correct gT_M_W
							gT_M_W = gt_pose_solar * pose.inverse();
							gMappingPipelineMulti->mappingProcessRequest({ image }, { pose }, true, gT_M_W, updateT_M_W, status);
							gGTPoseInjected[other_fid] = true;
						}
					}
                }
                else {
                    // GT pose already injected (gT_M_W has been updated), continue mapping 
                    gMappingPipelineMulti->mappingProcessRequest({image}, {pose}, /* fixedPose = */ false, gT_M_W, updateT_M_W, status);
                }
            }
			poseCamera = updateT_M_W * pose;
            gT_M_W = updateT_M_W;
			gMappingStatus = status;

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
            g3DOverlay->draw(poseCamera, camParams, displayImage);
			// draw pose on other FID 
			for (auto i = 0; i < gFIDtransforms.size(); i++)
				if (!gFIDtransforms[i].isApprox(Transform3Df::Identity()))
					g3DOverlay->draw(gFIDtransforms[i].inverse()*poseCamera, camParams, displayImage);
        }

        if (gImageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP) {
            gClientProducerTask->stop();
        }
    }
    else {
        gImageToSend = false;
        LOG_INFO("Producer client: no more images to send");
		if (gMappingPipelineMulti != 0)
			gMappingPipelineMulti->stop();
    }
};

// Function for viewer client thread
auto fnClientViewer = []() {
    if (timer.elapsed() > DELAY_BETWEEN_REQUESTS) {		
		// Try to get point clouds and key frame poses to display
        if (gMappingPipelineMulti->getDataForVisualization(pointClouds, keyframePoses) == FrameworkReturnCode::_SUCCESS) {
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

    if (gMappingPipelineMulti != 0)
        gMappingPipelineMulti->stop();

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
    char * config_file = (char *)"SolARPipelineTest_Mapping_Multi_Processing_conf.xml";
    char * config_file_producer = (char *)"SolARPipelineTest_Mapping_Multi_Producer_conf.xml";

    if (argc > 1) {
        // Get mapping pipeline configuration file path and name from main args
        config_file = argv[1];
    }
    if (argc > 2) {
        // Get mapping producer configuration file path and name from main args
        config_file_producer = argv[2];
    }

    try {
        LOG_INFO("Get Component Manager instance");

        gXpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Load Mapping Pipeline Multithreads configuration file");

        if (gXpcfComponentManager->load(config_file) == org::bcom::xpcf::_SUCCESS)
        {
            // Create Mapping Pipeline component
            gMappingPipelineMulti = gXpcfComponentManager->resolve<pipeline::IMappingPipeline>();
            LOG_INFO("Mapping pipeline multithreads component created");
			// init
			if (gMappingPipelineMulti->init() != FrameworkReturnCode::_SUCCESS)
				return -1;
        }
        else {
            LOG_ERROR("Failed to load the configuration file {}", config_file);
            return -1;
        }

        // Manage producer client thread
        if (gXpcfComponentManager->load(config_file_producer) == org::bcom::xpcf::_SUCCESS)
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

            // load groundtruth data if provided 
			gGTPoseInjected.fill(false);
			gFIDtransforms.resize(NB_MAX_FID-1, Transform3Df::Identity()); // other FID to ref, therefore NB_MAX_FID-1
            std::string pathToData = gArDevice->bindTo<xpcf::IConfigurable>()->getProperty("pathToData")->getStringValue();
            std::ifstream infile(pathToData + "/groundtruth.txt");
            if (infile.is_open()) {
                std::ifstream fileTimestamps(pathToData + "/timestamps.txt");
                if (!fileTimestamps.is_open()) {
                    LOG_ERROR("Failed to open timestamp file");
                    return -1;
                }
                std::vector<std::chrono::system_clock::time_point> timestamps;
                std::string line;
                while (std::getline(fileTimestamps, line)) {
                    std::chrono::milliseconds dur(std::stoll(line));
                    timestamps.push_back(std::chrono::time_point<std::chrono::system_clock>(dur));
                }
                fileTimestamps.close();

                // read GT data
                while (std::getline(infile, line)) {
                    vector<string> line_subs;
                    int marker_id, frame_start, frame_end;
                    auto key = line.substr(0, line.find_first_of("_"));
                    if (key == "FID" || key == "ACTIVATED") {
                        line_subs = ssplit(line, ",");
                        marker_id = std::stoi(ssplit(line_subs[0], "_")[1]);
                        frame_start = std::stoi(ssplit(line_subs[1], "_")[0]);
                        frame_end = std::stoi(ssplit(line_subs[1], "_")[1]);
                    }
                    if (key == "FID") {
                        auto mat = ssplit(line_subs[2], "_");
                        Transform3Df transform;
                        for (int r = 0; r < 4; r++)
                            for (int c = 0; c < 4; c++)
                                transform(r,c) = std::stof(mat[r*4+c]);
                        for (int t = frame_start; t <= frame_end; t++)
                            gGT[timestamps[t]] = groundtruth(marker_id, transform);
                        if (marker_id >= 1) // FID other than the reference one (ID 0)
                            gFIDtransforms[marker_id-1] = transform;
                    }
					else if (key == "ACTIVATED") {
						gGTActivateTimestamps[marker_id] = { timestamps.begin() + frame_start, timestamps.begin() + frame_end + 1 };
					}
                }
                infile.close();
            }

			// set camera parameters
			CameraRigParameters camRigParams = gArDevice->getCameraParameters();
			camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

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
                gMappingPipelineMulti->setCameraParameters(camParams);

                LOG_INFO("Producer client: Start mapping pipeline");

                if (gMappingPipelineMulti->start() == FrameworkReturnCode::_SUCCESS) {
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
        if (gXpcfComponentManager->load("SolARPipelineTest_Mapping_Multi_Viewer_conf.xml") == org::bcom::xpcf::_SUCCESS)
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
    catch(std::invalid_argument const& ex)
    {
        LOG_ERROR("Invalid argument has been caught: {}", ex.what());
        return -1;
    }

    return 0;
}
