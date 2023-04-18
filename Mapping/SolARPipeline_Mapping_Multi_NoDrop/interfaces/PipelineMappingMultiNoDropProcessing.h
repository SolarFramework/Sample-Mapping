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

#ifndef PIPELINEMAPPINGMULTINODROPPROCESSING_H
#define PIPELINEMAPPINGMULTINODROPPROCESSING_H

#if _WIN32
#ifdef SolARPipelineMappingMultiNoDrop_API_DLLEXPORT
#define SOLARPIPELINE_MAPPING_MULTI_NODROP_EXPORT_API __declspec(dllexport)
#else // SolARPipelineMappingMultiNoDrop_API_DLLEXPORT
#define SOLARPIPELINE_MAPPING_MULTI_NODROP_EXPORT_API __declspec(dllimport)
#endif //SolARPipelineMappingMultiNoDrop_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINE_MAPPING_MULTI_NODROP_EXPORT_API
#endif //_WIN32

#include "xpcf/component/ConfigurableBase.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/BaseTask.h"

#include <mutex>  // For std::unique_lock

#include "base/pipeline/AMappingPipeline.h"
#include "api/slam/IBootstrapper.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/ICameraParametersManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IMapManager.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/pipeline/IMapUpdatePipeline.h"
#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/geom/I3DTransform.h"

namespace SolAR {
using namespace api::pipeline;
namespace PIPELINES {
namespace MAPPING {

#define BUFFER_SIZE_IMAGE 50
#define BUFFER_SIZE_FRAME 50

    /**
     * @class PipelineMappingMultiNoDropProcessing
     * @brief Implementation of a mapping vision pipeline multithreading no dropping images
     * <TT>UUID: c1a30bbf-57a8-4ea3-83f5-28cc1d983f57</TT>
     *
     * @SolARComponentInjectablesBegin
     * @SolARComponentInjectable{SolAR::api::slam::IBootstrapper}
     * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}     
     * @SolARComponentInjectable{SolAR::api::slam::IMapping}
     * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
     * @SolARComponentInjectable{SolAR::api::storage::ICameraParametersManager}
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

    class SOLARPIPELINE_MAPPING_MULTI_NODROP_EXPORT_API PipelineMappingMultiNoDropProcessing : public base::pipeline::AMappingPipeline
    {
    public:
        PipelineMappingMultiNoDropProcessing();
        ~PipelineMappingMultiNoDropProcessing() override;

        /// @brief Method called when all component injections have been done
        void onInjected() override;

        void unloadComponent() override final {}

        /// @brief Initialization of the pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode init() override;

        /// @brief Initialization of the pipeline with the URL of an available Relocalization Service
        /// @param[in] relocalizationServiceURL the URL of an available Relocalization Service
        /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode init(const std::string relocalizationServiceURL) override;

        /// @brief Set the camera parameters
        /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
        /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setCameraParameters(const datastructure::CameraParameters & cameraParams) override;

        /// @brief Set the rectification parameters (use for stereo camera)
        /// @param[in] rectCam1 the rectification parameters of the first camera
        /// @param[in] rectCam2 the rectification parameters of the second camera
        /// @return FrameworkReturnCode::_SUCCESS if the rectification parameters are correctly set, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode setRectificationParameters(const SolAR::datastructure::RectificationParameters & rectCam1,
                                                       const SolAR::datastructure::RectificationParameters & rectCam2) override;

        /// @brief Start the pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the stard succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode start() override;

        /// @brief Stop the pipeline.
        /// @return FrameworkReturnCode::_SUCCESS if the stop succeed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode stop() override;

        /// @brief Request to the mapping pipeline to process a new image/pose
        /// Retrieve the new image (and pose) to process, in the current pipeline context
        /// (camera configuration, fiducial marker, point cloud, key frames, key points)
        /// @param[in] images the input image to process
        /// @param[in] poses the input pose in the device coordinate system
        /// @param[in] fixedPose the input poses are considered as ground truth
        /// @param[in] transform the transformation matrix from the device coordinate system to the world coordinate system
        /// @param[out] updatedTransform the refined transformation by a loop closure detection
        /// @param[out] status the current status of the mapping pipeline
        /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
        FrameworkReturnCode mappingProcessRequest(const std::vector<SRef<SolAR::datastructure::Image>> & images,
                                                  const std::vector<SolAR::datastructure::Transform3Df> & poses,
                                                  bool fixedPose,
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
		void correctPoseAndBootstrap();

		/// @brief Feature extraction
		void featureExtraction();

		/// @bried Update visibility
		void updateVisibility();

		/// @bried Mapping
		void mapping();

		/// @bried Loop closure detection
		void loopClosure();

		/// @brief Process to bundle adjustment, map pruning
		/// and update global map
		void globalBundleAdjustment();

        /// @brief drift correction
        void driftCorrection();

        /// @brief get map data
        void getMapData();

    private:

        mutable std::mutex									m_mutexMapData;         // Mutex for map data
        std::mutex                                          m_mutexMapping;         // Mutex for mapping
        datastructure::CameraParameters						m_cameraParams;         // camera parameters
        uint32_t                                            m_cameraParamsID;       // camera parameters ID
        std::vector<SRef<datastructure::CloudPoint>>        m_allPointClouds;       // all current point cloud
        std::vector<datastructure::Transform3Df>            m_allKeyframePoses;     // all current keyframe poses

        // Components used
        SRef<api::slam::IBootstrapper>						m_bootstrapper;
        SRef<api::solver::map::IBundler>					m_bundler, m_globalBundler;        
        SRef<api::slam::ITracking>							m_tracking;
        SRef<api::slam::IMapping>							m_mapping;
        SRef<api::storage::IKeyframesManager>				m_keyframesManager;
        SRef<api::storage::ICameraParametersManager>        m_cameraParametersManager;
        SRef<api::storage::IPointCloudManager>				m_pointCloudManager;
		SRef<api::storage::ICovisibilityGraphManager>		m_covisibilityGraphManager;
		SRef<api::storage::IMapManager>						m_mapManager;
        SRef<api::pipeline::IMapUpdatePipeline>				m_mapUpdatePipeline;
		SRef<api::pipeline::IRelocalizationPipeline>		m_relocPipeline;
        SRef<api::features::IDescriptorsExtractorFromImage>	m_descriptorExtractor;
        SRef<api::loop::ILoopClosureDetector>				m_loopDetector;
        SRef<api::loop::ILoopCorrector>						m_loopCorrector;
		SRef<api::geom::IUndistortPoints>					m_undistortKeypoints;
        SRef<api::geom::I3DTransform>                       m_transform3D;

        std::atomic_bool                                    m_isMappingIdle;		// indicates if the mapping task is idle
        std::atomic_bool                                    m_isLoopIdle;			// indicates if the loop closure task is idle
        std::atomic_bool                                    m_isDetectedLoop;       // indicates if a loop is detected
        std::atomic_bool                                    m_isDetectedDrift;      // indicates if a drift is detected by relocalization
        std::atomic<MappingStatus>                          m_status;               // current status of mapping pipeline
        datastructure::Transform3Df                         m_lastTransform;        // the last transformation matrix from device to world
        datastructure::Transform3Df                         m_loopTransform;        // the correction transformation matrix detected by loop closure
        uint32_t                                            m_lastKeyframeId;       // the last keyframe using the the last transformation
        uint32_t                                            m_curKeyframeId;        // the current keyframe will be corrected by using the new transformation
        float												m_minWeightNeighbor;
        int													m_countNewKeyframes;

        bool m_init = false;            // Indicate if initialization has been made
        bool m_cameraOK = false;        // Indicate if camera parameters has been set
        bool m_started = false;         // Indicate if pipeline il started
        bool m_tasksStarted = false;    // Indicate if tasks are started
        uint32_t m_nbTrackingLost = 0;   // Nb successive tracking lost events

        // Delegate task dedicated to asynchronous mapping processing
        xpcf::DelegateTask * m_bootstrapTask = nullptr;
        xpcf::DelegateTask * m_featureExtractionTask = nullptr;
        xpcf::DelegateTask * m_updateVisibilityTask = nullptr;
        xpcf::DelegateTask * m_mappingTask = nullptr;
        xpcf::DelegateTask * m_loopClosureTask = nullptr;
        xpcf::DelegateTask * m_driftCorrectionTask = nullptr;

        // Buffers used by mapping processing
        struct SharedBufferImagePoseElement
        {
            SRef<datastructure::Image> image;
            datastructure::Transform3Df pose;
            bool fixedPose;
        };
        xpcf::SharedBuffer<SharedBufferImagePoseElement>                        m_sharedBufferCamImagePoseCapture{ BUFFER_SIZE_IMAGE };
		xpcf::SharedBuffer<SRef<datastructure::Frame>>                          m_sharedBufferFrame{ BUFFER_SIZE_FRAME };
		xpcf::SharedBuffer<SRef<datastructure::Frame>>                          m_sharedBufferFrameBootstrap{ BUFFER_SIZE_FRAME };
		xpcf::DropBuffer<SRef<datastructure::Frame>>							m_dropBufferAddKeyframe;
        xpcf::DropBuffer<SRef<datastructure::Keyframe>>							m_dropBufferNewKeyframeLoop; 
        xpcf::DropBuffer<datastructure::Transform3Df>                           m_dropBufferDriftTransform;
    };

}
}
} // SolAR::PIPELINES::MAPPINGPIPELINE

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::MAPPING::PipelineMappingMultiNoDropProcessing,
                             "c1a30bbf-57a8-4ea3-83f5-28cc1d983f57",
                             "PipelineMappingMultiNoDropProcessing",
                             "PipelineMappingMultiNoDropProcessing implements api::pipeline::IMappingPipeline interface");

#endif // PIPELINEMAPPINGMULTINODROPPROCESSING_H
