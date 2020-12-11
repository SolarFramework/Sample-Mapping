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

#ifndef SOLARMAPPINGPIPELINEMULTIPROCESSING_H
#define SOLARMAPPINGPIPELINEMULTIPROCESSING_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolARMappingPipelineMultiAPI.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include <mutex>  // For std::unique_lock
#include <shared_mutex>

#include "api/pipeline/IMappingPipeline.h"
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/slam/IBootstrapper.h"
#include "api/solver/map/IBundler.h"
#include "api/solver/map/IMapper.h"
#include "api/slam/IMapping.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/geom/IProject.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "datastructure/CameraDefinitions.h"
#include "datastructure/Image.h"
#include "datastructure/CloudPoint.h"
#include "datastructure/Keypoint.h"
#include "datastructure/Trackable.h"
#include "datastructure/FiducialMarker.h"


namespace SolAR {
namespace PIPELINES {
namespace MAPPINGPIPELINE {

    /**
     * @class SolARMappingPipelineMultiProcessing
     * @brief Implementation of a mapping vision pipeline
     * <TT>UUID: dc734eb4-fcc6-4178-8452-7429939f04bd</TT>
     */

    class SolARMappingPipelineMulti_EXPORT_API SolARMappingPipelineMultiProcessing : public org::bcom::xpcf::ConfigurableBase,
            public api::pipeline::IMappingPipeline
    {
    public:
        SolARMappingPipelineMultiProcessing();
        ~SolARMappingPipelineMultiProcessing() override;

        /// @brief Method called when all component injections have been done
        void onInjected() override;

        void unloadComponent() override final {}

        /// @brief Initialization of the pipeline
        /// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
        /// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
        FrameworkReturnCode init(SRef<xpcf::IComponentManager> componentManager) override;

        /// @brief Set the camera parameters
        /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
        /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setCameraParameters(const datastructure::CameraParameters & cameraParams) override;

        /// @brief Set the object to track during mapping
        /// @param[in] trackableObject: the trackable object
        /// @return FrameworkReturnCode::_SUCCESS if the trackable object is correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setObjectToTrack(const SRef<datastructure::Trackable> & trackableObject) override;

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
        FrameworkReturnCode mappingProcessRequest(const SRef<datastructure::Image> & image, const datastructure::Transform3Df & pose) override;

        /// @brief Provide the current data from the mapping pipeline context for visualization
        /// (resulting from all mapping processing since the start of the pipeline)
        /// @param[out] outputPointClouds: pipeline current point clouds
        /// @param[out] keyframePoses: pipeline current keyframe poses
        /// @return FrameworkReturnCode::_SUCCESS if data are available, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode getDataForVisualization(std::vector<SRef<datastructure::CloudPoint>> & outputPointClouds,
                                                            std::vector<datastructure::Transform3Df> & keyframePoses) const override;

    private:

        bool m_isBootstrapFinished; // indicates if the bootstrap step is finished
        mutable std::shared_mutex m_bootstrap_mutex;  // Mutex used for bootstrap state
        std::mutex m_mutexUseLocalMap; // Mutex used for mapping task

        datastructure::CameraParameters m_cameraParams;        // camera parameters
        SRef<datastructure::FiducialMarker> m_fiducialMarker;  // fiducial marker description

        // Components used
        SRef<api::solver::pose::IFiducialMarkerPose> m_fiducialMarkerPoseEstimator;
        SRef<api::slam::IBootstrapper> m_bootstrapper;
        SRef<api::solver::map::IBundler> m_bundler, m_globalBundler;
        SRef<api::solver::map::IMapper> m_mapper;
        SRef<api::slam::IMapping> m_mapping;
        SRef<api::storage::IKeyframesManager> m_keyframesManager;
        SRef<api::storage::IPointCloudManager> m_pointCloudManager;
        SRef<api::features::IKeypointDetector> m_keypointsDetector;
        SRef<api::features::IDescriptorsExtractor> m_descriptorExtractor;
        SRef<api::features::IDescriptorMatcher> m_matcher;
        SRef<api::features::IMatchesFilter> m_matchesFilter;
        SRef<api::solver::pose::I2D3DCorrespondencesFinder> m_corr2D3DFinder;
        SRef<api::geom::IProject> m_projector;
        SRef<api::storage::ICovisibilityGraph> m_covisibilityGraph;
        SRef<api::loop::ILoopClosureDetector> m_loopDetector;
        SRef<api::loop::ILoopCorrector> m_loopCorrector;

        bool m_isFoundTransform;            // indicates if the 3D transformation as been found
        bool m_isStopMapping;               // indicates if the mapping task is stopped
        datastructure::Transform3Df m_T_M_W;               // 3D transformation matrix
        std::vector<SRef<datastructure::CloudPoint>> m_localMap; // Local map
        float m_minWeightNeighbor, m_reprojErrorThreshold;
        SRef<datastructure::Keyframe> m_refKeyframe;
        int m_countNewKeyframes;

        // Delegate task dedicated to asynchronous mapping processing
        xpcf::DelegateTask * m_bootstrapTask = nullptr;
        xpcf::DelegateTask * m_keypointsDetectionTask = nullptr;
        xpcf::DelegateTask * m_featureExtractionTask = nullptr;
        xpcf::DelegateTask * m_updateVisibilityTask = nullptr;
        xpcf::DelegateTask * m_mappingTask = nullptr;
        xpcf::DelegateTask * m_loopClosureTask = nullptr;

        // Drop buffers used by mapping processing
        xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>>  m_dropBufferCamImagePoseCaptureBootstrap;
        xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>>  m_dropBufferCamImagePoseCapture;
        xpcf::DropBuffer<SRef<datastructure::Frame>>                           m_dropBufferKeypoints;
        xpcf::DropBuffer<SRef<datastructure::Frame>>                           m_dropBufferFrameDescriptors;
        xpcf::DropBuffer<SRef<datastructure::Frame>>                           m_dropBufferAddKeyframe;
        xpcf::DropBuffer<SRef<datastructure::Keyframe>>                        m_dropBufferNewKeyframe;
        xpcf::DropBuffer<SRef<datastructure::Keyframe>>                        m_dropBufferNewKeyframeLoop;

        /// @brief Correct pose and do bootstrap using an image and the associated pose
        /// This method must be called with successive pairs of (image, pose)
        /// until the bootstrap process is finished (i.e. m_isBootstrapFinished is True)
        void correctPoseAndBootstrap();

        /// @brief Detection of keypoints
        void keypointsDetection();

        /// @brief Feature extraction
        void featureExtraction();

        /// @bried Update visibility
        void updateVisibility();

        /// @bried Mapping
        void mapping();

        /// @bried Loop closure detection
        void loopClosure();

        /// @brief Update local map
        /// @param[in] keyframe: reference key frame
        void updateLocalMap(const SRef<datastructure::Keyframe> & keyframe);

        /// @brief Process to bundle adjustment, map pruning
        /// and update global map
        void globalBundleAdjustment();

        /// @brief returns the status of bootstrap
        /// @return true if bootstrap is finished (m_isBootstrapFinished value)
        bool isBootstrapFinished() const;

        /// @brief sets the bootstrap status
        /// (the m_isBootstrapFinished variable value)
        /// @param status: true (finished) or false (not finished)
        void setBootstrapSatus(const bool status);
    };

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE

#endif // SOLARMAPPINGPIPELINEMULTIPROCESSING_H
