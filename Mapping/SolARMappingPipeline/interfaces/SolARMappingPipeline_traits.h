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

#ifndef SOLARMAPPINGPIPELINE_TRAITS_H
#define SOLARMAPPINGPIPELINE_TRAITS_H

#include "xpcf/core/traits.h"

namespace SolAR {
namespace PIPELINES {
/**
 * @namespace SolAR::PIPELINES::MAPPINGPIPELINE
 * @brief <B>Provides a full mapping vision pipeline</B>
 * <TT>UUID: 43a3f181-99ba-4cc4-a926-8db6fab74454</TT>
 *
 */
namespace MAPPINGPIPELINE {
    class SolARMappingPipelineProcessing;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::MAPPINGPIPELINE::SolARMappingPipelineProcessing,
                             "54b93f91-1628-4e63-b4d9-c8735b768b8b",
                             "SolARMappingPipelineProcessing",
                             "SolARMappingPipelineProcessing implements api::pipeline::IMappingPipeline interface");

#endif // SOLARMAPPINGPIPELINE_TRAITS_H
