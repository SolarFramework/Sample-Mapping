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

#include "xpcf/module/ModuleFactory.h"
#include "PipelineMappingMultiProcessing.h"

namespace xpcf=org::bcom::xpcf;

/**
 *  @ingroup xpcfmodule
 */
/**
  * Declare module.
  */
XPCF_DECLARE_MODULE("{e7b8c768-a14a-4841-8700-6112697cf599}","PipelineMappingMulti",
                    "Multi threads mapping vision pipeline based on SolAR Framework");

/**
 * This method is the module entry point.
 * XPCF uses this method to create components available in the module.
 *
 * Each component exposed must be declared inside a xpcf::tryCreateComponent<ComponentType>() call.
 */
extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const xpcf::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::MAPPING::PipelineMappingMultiProcessing>(componentUUID,interfaceRef);

    return errCode;
}

/**
  * The declarations below populate list of the components available in the module (it represents the module index).
  * XPCF uses this index to introspect the components available in a module, providing the ability to generate the configuration file skeleton from the code.
  */
XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::MAPPING::PipelineMappingMultiProcessing)
XPCF_END_COMPONENTS_DECLARATION
