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

#ifndef PipelineMappingMonoProcessing_H
#define PipelineMappingMonoProcessing_H

#if _WIN32
#ifdef SolARPipelineMappingMono_API_DLLEXPORT
#define SolARPipelineMappingMono_EXPORT_API __declspec(dllexport)
#else // SolARPipelineMappingMono_API_DLLEXPORT
#define SolARPipelineMappingMono_EXPORT_API __declspec(dllimport)
#endif // SolARPipelineMappingMono_API_DLLEXPORT
#else //_WIN32
#define SolARPipelineMappingMono_EXPORT_API
#endif //_WIN32

#include "xpcf/component/ConfigurableBase.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include <mutex>  // For std::unique_lock
#include <shared_mutex>

#include "api/pipeline/IMappingPipeline.h"
#include "api/slam/IBootstrapper.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IMapManager.h"
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
namespace MAPPING {

    /**
     * @class PipelineMappingMonoProcessing
     * @brief Implementation of a mapping vision pipeline
     * <TT>UUID: 54b93f91-1628-4e63-b4d9-c8735b768b8b</TT>
     *
     * @SolARComponentInjectablesBegin
     * @SolARComponentInjectable{SolAR::api::slam::IBootstrapper}
     * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}     
     * @SolARComponentInjectable{SolAR::api::slam::IMapping}
     * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
     * @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
	 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraphManager}
	 * @SolARComponentInjectable{SolAR::api::storage::IMapManager}
     * @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
     * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
     * @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
     * @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
     * @SolARComponentInjectable{SolAR::api::solver::pose::I2D3DCorrespondencesFinder}
     * @SolARComponentInjectable{SolAR::api::geom::IProject}     
     * @SolARComponentInjectable{SolAR::api::loop::ILoopClosureDetector}
     * @SolARComponentInjectable{SolAR::api::loop::ILoopCorrector}
     * @SolARComponentInjectable{SolAR::api::geom::IUndistortPoints}
     * @SolARComponentInjectablesEnd
     *
     */

    class SolARPipelineMappingMono_EXPORT_API PipelineMappingMonoProcessing : public org::bcom::xpcf::ConfigurableBase,
            public api::pipeline::IMappingPipeline
    {
    public:
        PipelineMappingMonoProcessing();
        ~PipelineMappingMonoProcessing() override;

        /// @brief Method called when all component injections have been done
        void onInjected() override;

        void unloadComponent() override final {}

        /// @brief Initialization of the pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode init() override;

        /// @brief Set the camera parameters
        /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
        /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setCameraParameters(const datastructure::CameraParameters & cameraParams) override;

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
        FrameworkReturnCode mappingProcessRequest(const SRef<datastructure::Image> image, const datastructure::Transform3Df & pose) override;

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

        datastructure::CameraParameters m_cameraParams;        // camera parameters

        // Components used
        SRef<api::slam::IBootstrapper> m_bootstrapper;
        SRef<api::solver::map::IBundler> m_bundler, m_globalBundler;        
        SRef<api::slam::ITracking> m_tracking;
        SRef<api::slam::IMapping> m_mapping;
        SRef<api::storage::IKeyframesManager> m_keyframesManager;
        SRef<api::storage::IPointCloudManager> m_pointCloudManager;
		SRef<api::storage::ICovisibilityGraphManager> m_covisibilityGraphManager;
		SRef<api::storage::IMapManager> m_mapManager;
        SRef<api::features::IKeypointDetector> m_keypointsDetector;
        SRef<api::features::IDescriptorsExtractor> m_descriptorExtractor;
        SRef<api::features::IDescriptorMatcher> m_matcher;
        SRef<api::features::IMatchesFilter> m_matchesFilter;
        SRef<api::solver::pose::I2D3DCorrespondencesFinder> m_corr2D3DFinder;
        SRef<api::geom::IProject> m_projector;        
        SRef<api::loop::ILoopClosureDetector> m_loopDetector;
        SRef<api::loop::ILoopCorrector> m_loopCorrector;
		SRef<api::geom::IUndistortPoints> m_undistortKeypoints;

        datastructure::Transform3Df m_T_M_W;               // 3D transformation matrix
        float m_minWeightNeighbor, m_reprojErrorThreshold;
        int m_countNewKeyframes;

        // Delegate task dedicated to asynchronous mapping processing
        xpcf::DelegateTask * m_mappingTask = nullptr;

        // Drop buffer containing (image,pose) pairs sent by client
        xpcf::SharedFifo<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_inputImagePoseBuffer;

        /// @brief Correct pose and do bootstrap using an image and the associated pose
        /// This method must be called with successive pairs of (image, pose)
        /// until the bootstrap process is finished (i.e. m_isBootstrapFinished is True)
        /// @param[in] image: the input image to process
        /// @param[in] pose: the input pose to process
        /// @return true if bootstrap is finished
        bool correctPoseAndBootstrap(const SRef<datastructure::Image> & image, const datastructure::Transform3Df & pose);

        /// @brief Process to bundle adjustment, map pruning
        /// and update global map
        void globalBundleAdjustment();

        /// @brief method that implementes the full maping processing
        void processMapping();

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


XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::MAPPING::PipelineMappingMonoProcessing,
                             "54b93f91-1628-4e63-b4d9-c8735b768b8b",
                             "PipelineMappingMonoProcessing",
                             "PipelineMappingMonoProcessing implements api::pipeline::IMappingPipeline interface");

#endif // PipelineMappingMonoProcessing_H
