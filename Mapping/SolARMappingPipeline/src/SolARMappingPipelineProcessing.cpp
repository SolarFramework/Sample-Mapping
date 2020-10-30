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

#include "SolARMappingPipelineProcessing.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace MAPPINGPIPELINE {

    SolARMappingPipelineProcessing::SolARMappingPipelineProcessing():ComponentBase(xpcf::toUUID<SolARMappingPipelineProcessing>())
    {
        declareInterface<api::pipeline::IMappingPipeline>(this);

        LOG_DEBUG("SolARMappingPipelineProcessing constructor");
    }


    SolARMappingPipelineProcessing::~SolARMappingPipelineProcessing() {

        LOG_DEBUG("SolARMappingPipelineProcessing destructor");
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::init(SRef<xpcf::IComponentManager> componentManager) {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::start() {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::stop() {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::setCameraParameters(const CameraParameters & cameraParams) {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::setObjectToTrack(const Trackable & trackableObject) {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::mappingProcessRequest(const SRef<Image> & image, const Transform3Df & pose) {
        return FrameworkReturnCode::_SUCCESS;
    }

    FrameworkReturnCode SolARMappingPipelineProcessing::getDataForVisualization(std::vector<CloudPoint> & outputPointClouds,
                                                std::vector<SRef<Transform3Df>> & keyframePoses) {
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE
