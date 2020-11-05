/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLARMAPPINGPIPELINEPROCESSING_H
#define SOLARMAPPINGPIPELINEPROCESSING_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolARMappingPipelineAPI.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include "api/pipeline/IMappingPipeline.h"
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/slam/IBootstrapper.h"
#include "api/solver/map/IBundler.h"
#include "api/solver/map/IMapper.h"
#include "api/slam/IMapping.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "datastructure/CameraDefinitions.h"
#include "datastructure/Image.h"
#include "datastructure/CloudPoint.h"
#include "datastructure/Keypoint.h"
#include "datastructure/Trackable.h"
#include "datastructure/FiducialMarker.h"


namespace SolAR {
using namespace datastructure;
using namespace SolAR::api;
namespace PIPELINES {
namespace MAPPINGPIPELINE {

    /**
     * @class SolARMappingPipelineProcessing
     * @brief Implementation of a mapping vision pipeline
     * <TT>UUID: 54b93f91-1628-4e63-b4d9-c8735b768b8b</TT>
     */

    class SolARMappingPipeline_EXPORT_API SolARMappingPipelineProcessing : public org::bcom::xpcf::ConfigurableBase,
            public api::pipeline::IMappingPipeline
    {
    public:
        SolARMappingPipelineProcessing();
        ~SolARMappingPipelineProcessing() override;

        void unloadComponent() override final {}

        /// @brief Initialization of the pipeline
        /// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
        /// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
        FrameworkReturnCode init(SRef<xpcf::IComponentManager> componentManager) override;

        /// @brief Set the camera parameters
        /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
        /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setCameraParameters(const CameraParameters & cameraParams) override;

        /// @brief Set the object to track during mapping
        /// @param[in] trackableObject: the trackable object
        /// @return FrameworkReturnCode::_SUCCESS if the trackable object is correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setObjectToTrack(const Trackable & trackableObject) override;

        /// @brief Correct pose and do bootstrap using an image and the associated pose
        /// This method must be called with successive pairs of (image, pose)
        /// until the bootstrap process is finished (i.e. isBootstrapFinished returns True)
        /// @param[in] image: the input image to process
        /// @param[in] pose: the input pose to process
        /// @return FrameworkReturnCode::_SUCCESS if the processing is ok, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode correctPoseAndBootstrap(const SRef<Image> & image, const Transform3Df & pose) override;

        /// @brief Returns true if the boostrap process is finished
        /// @return bool::true if the bootstrap is finished, else false
        bool isBootstrapFinished() const override;

        /// @brief Start the pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the stard succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode start() override;

        /// @brief Stop the pipeline.
        /// @return FrameworkReturnCode::_SUCCESS if the stop succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode stop() override;

        /// @brief Request to the mapping pipeline to process a new image/pose
        /// Retrieve the new image (and pose) to process, in the current pipeline context
        /// (camera configuration, fiducial marker, point cloud, key frames, key points)
        /// @param[in] image: the input image to process
        /// @param[in] pose: the input pose to process
        /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode mappingProcessRequest(const SRef<Image> & image, const Transform3Df & pose) override;

        /// @brief Provide the current data from the mapping pipeline context for visualization
        /// (resulting from all mapping processing since the start of the pipeline)
        /// @param[out] outputPointClouds: pipeline current point clouds
        /// @param[out] keyframePoses: pipeline current keyframe poses
        /// @return FrameworkReturnCode::_SUCCESS if data are available, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode getDataForVisualization(std::vector<CloudPoint> & outputPointClouds,
                                                            std::vector<SRef<Transform3Df>> & keyframePoses) override;

    private:

        CameraParameters m_cameraParams;  // camera parameters
        FiducialMarker m_fiducialMarker;  // fiducial marker description

        // Components used
        SRef<api::solver::pose::IFiducialMarkerPose> m_fiducialMarkerPoseEstimator;
        SRef<api::slam::IBootstrapper> m_bootstrapper;
        SRef<api::solver::map::IBundler> m_bundler, m_globalBundler;
        SRef<api::solver::map::IMapper> m_mapper;
        SRef<api::slam::IMapping> m_mapping;
        SRef<api::storage::IKeyframesManager> m_keyframesManager;
        SRef<api::storage::IPointCloudManager> m_pointCloudManager;

        bool m_isBootstrapFinished;       // indicates if the bootstrap step is finished
        bool m_isFoundTransform;          // indicates if the 3D transformation as been found
        Transform3Df m_T_M_W;             // 3D transformation matrix
        std::vector<SRef<CloudPoint>> m_localMap; // Local map
        float m_minWeightNeighbor, m_reprojErrorThreshold;
        std::vector<Transform3Df> m_framePoses;
        SRef<Keyframe> m_refKeyframe;
        int m_countNewKeyframes;

        // Delegate task dedicated to asynchronous mapping processing
        xpcf::DelegateTask * m_mappingTask = nullptr;

        // Drop buffer containing (image,pose) pairs for mapping pipeline processing
        xpcf::DropBuffer<std::pair<SRef<Image>, Transform3Df>> m_inputDropBufferImagePose;

        /// @brief Clear local map and initialize mapper
        /// @param[in] keyframe: reference key frame
        void updateLocalMap(const SRef<Keyframe> & keyframe);

        /// @brief method that implementes the full maping processing
        void processMapping();
    };

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE

#endif // SOLARMAPPINGPIPELINEPROCESSING_H
