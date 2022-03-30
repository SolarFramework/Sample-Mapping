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
#define SOLARPIPELINE_MAPPING_MONO_EXPORT_API __declspec(dllexport)
#else // SolARPipelineMappingMono_API_DLLEXPORT
#define SOLARPIPELINE_MAPPING_MONO_EXPORT_API __declspec(dllimport)
#endif // SolARPipelineMappingMono_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINE_MAPPING_MONO_EXPORT_API
#endif //_WIN32

#include "xpcf/component/ConfigurableBase.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include <mutex>  // For std::unique_lock
#include <shared_mutex>
#include <atomic>

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
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/geom/I3DTransform.h"

namespace SolAR {
using namespace api::pipeline;
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

    class SOLARPIPELINE_MAPPING_MONO_EXPORT_API PipelineMappingMonoProcessing : public org::bcom::xpcf::ConfigurableBase,
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
        /// @param[in] image the input image to process
        /// @param[in] pose the input pose in the device coordinate system
        /// @param[in] transform the transformation matrix from the device coordinate system to the world coordinate system
        /// @param[out] updatedTransform the refined transformation by a loop closure detection
        /// @param[out] status the current status of the mapping pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode mappingProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                  const SolAR::datastructure::Transform3Df & pose,
                                                  const SolAR::datastructure::Transform3Df & transform,
                                                  SolAR::datastructure::Transform3Df & updatedTransform,
                                                  MappingStatus & status) override;

        /// @brief Provide the current data from the mapping pipeline context for visualization
        /// (resulting from all mapping processing since the start of the pipeline)
        /// @param[out] outputPointClouds: pipeline current point clouds
        /// @param[out] keyframePoses: pipeline current keyframe poses
        /// @return FrameworkReturnCode::_SUCCESS if data are available, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode getDataForVisualization(std::vector<SRef<datastructure::CloudPoint>> & outputPointClouds,
                                                    std::vector<datastructure::Transform3Df> & keyframePoses) const override;

	private:
		/// @brief Correct pose and do bootstrap using an image and the associated pose
		/// This method must be called with successive pairs of (image, pose)
		/// until the bootstrap process is finished (i.e. m_isBootstrapFinished is True)
		/// @param[in] frame the input frame to process
		/// @return true if bootstrap is finished
		bool correctPoseAndBootstrap(const SRef<datastructure::Frame> & frame);

		/// @brief Process to bundle adjustment, map pruning
		/// and update global map
		void globalBundleAdjustment();

		/// @brief method that implementes the full maping processing
		void processMapping();

        /// @brief get map data
        void getMapData();

        /// @brief drift correction
        void driftCorrection(datastructure::Transform3Df driftTransform);

    private:
        datastructure::CameraParameters                 m_cameraParams;     // camera parameters
        bool                                            m_isDetectedLoop;   // if a loop closure is detected and optimized
        std::atomic<MappingStatus>                      m_status;           // current status of mapping pipeline
        datastructure::Transform3Df                     m_lastTransform;    // the last transformation matrix from device to world
        datastructure::Transform3Df                     m_loopTransform;    // the correction transformation matrix detected by loop closure
        bool                                            m_init = false;     // Indicate if initialization has been made
        bool                                            m_cameraOK = false; // Indicate if camera parameters has been set
        mutable std::mutex                              m_mutexMapData;     // Mutex for map data
        std::vector<SRef<datastructure::CloudPoint>>    m_allPointClouds;   // all current point cloud
        std::vector<datastructure::Transform3Df>        m_allKeyframePoses; // all current keyframe poses
        uint32_t                                        m_lastKeyframeId;   // the last keyframe using the the last transformation
        uint32_t                                        m_curKeyframeId;    // the current keyframe will be corrected by using the new transformation

        // Components used
        SRef<api::slam::IBootstrapper> m_bootstrapper;
        SRef<api::solver::map::IBundler> m_bundler, m_globalBundler;        
        SRef<api::slam::ITracking> m_tracking;
        SRef<api::slam::IMapping> m_mapping;
        SRef<api::storage::IKeyframesManager> m_keyframesManager;
        SRef<api::storage::IPointCloudManager> m_pointCloudManager;
		SRef<api::storage::ICovisibilityGraphManager> m_covisibilityGraphManager;
		SRef<api::storage::IMapManager> m_mapManager;
        SRef<api::features::IDescriptorsExtractorFromImage> m_descriptorExtractor;
        SRef<api::loop::ILoopClosureDetector> m_loopDetector;
        SRef<api::loop::ILoopCorrector> m_loopCorrector;
		SRef<api::geom::IUndistortPoints> m_undistortKeypoints;
        SRef<api::geom::I3DTransform> m_transform3D;

        float m_minWeightNeighbor;
        int m_countNewKeyframes;

        // Delegate task dedicated to asynchronous mapping processing
        xpcf::DelegateTask * m_mappingTask = nullptr;

        // Drop buffer containing (image,pose) pairs sent by client
        xpcf::SharedFifo<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_inputImagePoseBuffer;       
    };

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE


XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::MAPPING::PipelineMappingMonoProcessing,
                             "54b93f91-1628-4e63-b4d9-c8735b768b8b",
                             "PipelineMappingMonoProcessing",
                             "PipelineMappingMonoProcessing implements api::pipeline::IMappingPipeline interface");

#endif // PipelineMappingMonoProcessing_H
