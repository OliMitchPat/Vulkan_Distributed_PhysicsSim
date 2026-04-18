#pragma once
#include <vulkan/vulkan.h>

struct PipelineCommonStates
{
    VkPipelineInputAssemblyStateCreateInfo inputAsm{};
    VkPipelineViewportStateCreateInfo viewportState{};
    VkPipelineDynamicStateCreateInfo dynamic{};
    VkPipelineRasterizationStateCreateInfo raster{};
    VkPipelineMultisampleStateCreateInfo msaa{};
    VkPipelineDepthStencilStateCreateInfo depth{};

    // must stay alive while creating the pipeline
    VkDynamicState dynStates[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
};

inline PipelineCommonStates MakeCommonPipelineStates(
    VkCullModeFlags cullMode,
    VkCompareOp depthCompareOp,
    VkBool32 depthTestEnable,
    VkBool32 depthWriteEnable)
{
    PipelineCommonStates s{};

    s.inputAsm.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    s.inputAsm.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    s.inputAsm.primitiveRestartEnable = VK_FALSE;

    s.viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    s.viewportState.viewportCount = 1;
    s.viewportState.scissorCount = 1;

    s.dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    s.dynamic.dynamicStateCount = 2;
    s.dynamic.pDynamicStates = s.dynStates;

    s.raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    s.raster.depthClampEnable = VK_FALSE;
    s.raster.rasterizerDiscardEnable = VK_FALSE;
    s.raster.polygonMode = VK_POLYGON_MODE_FILL;
    s.raster.lineWidth = 1.0f;
    s.raster.cullMode = cullMode;
    s.raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    s.raster.depthBiasEnable = VK_FALSE;

    s.msaa.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    s.msaa.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    s.msaa.sampleShadingEnable = VK_FALSE;

    s.depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    s.depth.depthTestEnable = depthTestEnable;
    s.depth.depthWriteEnable = depthWriteEnable;
    s.depth.depthCompareOp = depthCompareOp;
    s.depth.depthBoundsTestEnable = VK_FALSE;
    s.depth.stencilTestEnable = VK_FALSE;

    return s;
}

inline PipelineCommonStates MakeGlobePipelineStates()
{
    return MakeCommonPipelineStates(
        VK_CULL_MODE_NONE,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        VK_TRUE,
        VK_FALSE);
}

inline PipelineCommonStates MakeParticlePipelineStates()
{
    return MakeCommonPipelineStates(
        VK_CULL_MODE_NONE,
        VK_COMPARE_OP_LESS,
        VK_TRUE,
        VK_FALSE);
}

struct PipelineBlendAndPipe
{
    VkPipelineColorBlendAttachmentState blendAtt{};
    VkPipelineColorBlendStateCreateInfo blending{};
    VkGraphicsPipelineCreateInfo pipe{};
};

inline PipelineBlendAndPipe MakeAlphaBlendPipeBase(
    const VkPipelineShaderStageCreateInfo* stages,
    uint32_t stageCount,
    const VkPipelineVertexInputStateCreateInfo* vertexInput,
    const PipelineCommonStates& common)
{
    PipelineBlendAndPipe out{};

    // Alpha blending
    out.blendAtt.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    out.blendAtt.blendEnable = VK_TRUE;
    out.blendAtt.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    out.blendAtt.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    out.blendAtt.colorBlendOp = VK_BLEND_OP_ADD;
    out.blendAtt.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    out.blendAtt.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    out.blendAtt.alphaBlendOp = VK_BLEND_OP_ADD;

    out.blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    out.blending.logicOpEnable = VK_FALSE;
    out.blending.attachmentCount = 1;
    out.blending.pAttachments = &out.blendAtt;

    // Base pipeline create info
    out.pipe.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    out.pipe.stageCount = stageCount;
    out.pipe.pStages = stages;

    out.pipe.pVertexInputState = vertexInput;
    out.pipe.pInputAssemblyState = &common.inputAsm;
    out.pipe.pViewportState = &common.viewportState;
    out.pipe.pRasterizationState = &common.raster;
    out.pipe.pMultisampleState = &common.msaa;
    out.pipe.pDepthStencilState = &common.depth;
    out.pipe.pDynamicState = &common.dynamic;
    out.pipe.pColorBlendState = &out.blending;

    return out;
}

