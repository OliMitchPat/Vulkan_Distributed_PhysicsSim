#include "Renderer.h"
#include <stdexcept>
#include <array>
#include "PipelineCommon.h"

void Renderer::createGlobePipeline()
{
    auto const vertShaderCode = readFile("shaders/shader.vert.spv");
    auto const fragShaderCode = readFile("shaders/globe.frag.spv");

    VkShaderModule const vertShaderModule = createShaderModule(vertShaderCode);
    VkShaderModule const fragShaderModule = createShaderModule(fragShaderCode);

    VkPipelineShaderStageCreateInfo vertStage{};
    vertStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertShaderModule;
    vertStage.pName = kShaderEntryPoint;

    VkPipelineShaderStageCreateInfo fragStage{};
    fragStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragShaderModule;
    fragStage.pName = kShaderEntryPoint;

    VkPipelineShaderStageCreateInfo stages[] = { vertStage, fragStage };

    // ----- Vertex input: your normal mesh Vertex -----
    auto bindingDescription = Vertex::getBindingDescription();
    auto attributeDescriptions = Vertex::getAttributeDescriptions();

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &bindingDescription;
    vertexInput.vertexAttributeDescriptionCount =
        static_cast<uint32_t>(attributeDescriptions.size());
    vertexInput.pVertexAttributeDescriptions = attributeDescriptions.data();

    // Common fixed-function states + alpha blend base pipe
    auto const common = MakeParticlePipelineStates();

    auto const base = MakeAlphaBlendPipeBase(
        stages,
        2,
        &vertexInput,
        common
    );

    VkGraphicsPipelineCreateInfo pipe = base.pipe;

    // Reuse your existing pipelineLayout (set0 UBO + set1 texture + push constants)
    pipe.layout = globePipelineLayout;
    pipe.renderPass = renderPass;
    pipe.subpass = 0;

    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipe, nullptr, &globePipeline) != VK_SUCCESS)
        throw std::runtime_error("failed to create globe pipeline!");

    vkDestroyShaderModule(device, fragShaderModule, nullptr);
    vkDestroyShaderModule(device, vertShaderModule, nullptr);
}

void Renderer::destroyGlobePipeline()
{
    if (globePipeline)
    {
        vkDestroyPipeline(device, globePipeline, nullptr);
        globePipeline = VK_NULL_HANDLE;
    }
}

void Renderer::createGlobeSkySetLayout()
{
    VkDescriptorSetLayoutBinding dayBinding{};
    dayBinding.binding = 0;
    dayBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    dayBinding.descriptorCount = 1;
    dayBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutBinding nightBinding{};
    nightBinding.binding = 1;
    nightBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    nightBinding.descriptorCount = 1;
    nightBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::array<VkDescriptorSetLayoutBinding, 2> bindings = { dayBinding, nightBinding };

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = static_cast<uint32_t>(bindings.size());
    info.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(device, &info, nullptr, &globeSkySetLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create globeSkySetLayout!");
}

void Renderer::createGlobePipelineLayout()
{
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(PushConstantObject1);

    VkDescriptorSetLayout setLayouts[] = {
        uboSetLayout,        // set 0
        globeSkySetLayout    // set 1 (day + night)
    };

    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 2;
    info.pSetLayouts = setLayouts;
    info.pushConstantRangeCount = 1;
    info.pPushConstantRanges = &pushRange;

    if (vkCreatePipelineLayout(device, &info, nullptr, &globePipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create globePipelineLayout!");
}

void Renderer::createGlobeSkyDescriptorSet()
{
    // Load both textures through your normal loader
    const GpuTexture& dayTex = getOrLoadTexture("sky3.png");
    const GpuTexture& nightTex = getOrLoadTexture("sky5.png");

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc.descriptorPool = descriptorPool;
    alloc.descriptorSetCount = 1;
    alloc.pSetLayouts = &globeSkySetLayout;

    if (vkAllocateDescriptorSets(device, &alloc, &globeSkyDescriptorSet) != VK_SUCCESS)
        throw std::runtime_error("failed to allocate globeSkyDescriptorSet");

    // Binding 0 = day texture
    VkDescriptorImageInfo dayInfo{};
    dayInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    dayInfo.imageView = dayTex.view;
    dayInfo.sampler = textureSampler;

    // Binding 1 = night texture
    VkDescriptorImageInfo nightInfo{};
    nightInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    nightInfo.imageView = nightTex.view;
    nightInfo.sampler = textureSampler;

    VkWriteDescriptorSet writes[2]{};

    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = globeSkyDescriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &dayInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = globeSkyDescriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].pImageInfo = &nightInfo;

    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
}
