// RendererParticles.cpp
#include "Renderer.h"

#include <algorithm>
#include <stdexcept>
#include <cstddef> // offsetof
#include "PipelineCommon.h" 
#include <iostream>
// -------------------------
// Task 3.2: Create buffers
// -------------------------
void Renderer::createParticleInstanceBuffers()
{
    VkDeviceSize const bufferSize = sizeof(GpuParticleInstance) * static_cast<VkDeviceSize>(maxParticles);

    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i)
    {
        createBuffer(
            bufferSize,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            particleInstanceBuffers[i],
            particleInstanceMemories[i]
        );

        void* mapped = nullptr;
        VkResult const res = vkMapMemory(device, particleInstanceMemories[i], 0, bufferSize, 0, &mapped);
        if (res != VK_SUCCESS)
            throw std::runtime_error("Failed to map particle instance buffer memory!");

        particleInstanceMapped[i] = mapped;
    }
}

void Renderer::destroyParticleInstanceBuffers()
{
    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i)
    {
        if (particleInstanceMapped[i])
        {
            vkUnmapMemory(device, particleInstanceMemories[i]);
            particleInstanceMapped[i] = nullptr;
        }

        if (particleInstanceBuffers[i])
        {
            vkDestroyBuffer(device, particleInstanceBuffers[i], nullptr);
            particleInstanceBuffers[i] = VK_NULL_HANDLE;
        }

        if (particleInstanceMemories[i])
        {
            vkFreeMemory(device, particleInstanceMemories[i], nullptr);
            particleInstanceMemories[i] = VK_NULL_HANDLE;
        }
    }
}

// -------------------------
// Task 3.3: Upload per frame
// -------------------------
void Renderer::updateParticleInstanceBuffer(uint32_t frameIndex)
{
    particleCount = 0;

    if (!currentScene)
        return;

    const auto& parts = currentScene->particles;
    if (parts.empty())
        return;

    const size_t count = std::min(parts.size(), static_cast<size_t>(maxParticles));
    particleCount = static_cast<uint32_t>(count);

    auto* const dst = reinterpret_cast<GpuParticleInstance*>(particleInstanceMapped[frameIndex]);
    for (uint32_t i = 0; i < particleCount; ++i)
    {
        dst[i].pos_size[0] = parts[i].position.x;
        dst[i].pos_size[1] = parts[i].position.y;
        dst[i].pos_size[2] = parts[i].position.z;
        dst[i].pos_size[3] = parts[i].size;

        dst[i].color[0] = parts[i].color.r;
        dst[i].color[1] = parts[i].color.g;
        dst[i].color[2] = parts[i].color.b;
        dst[i].color[3] = parts[i].color.a;
    }
}

// -------------------------
// Task 4: Pipeline layout
// -------------------------
void Renderer::createParticlePipelineLayout()
{
    // Particles only need set 0 (UBO) so they can billboard using view/proj.
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &uboSetLayout;
    layoutInfo.pushConstantRangeCount = 0;
    layoutInfo.pPushConstantRanges = nullptr;

    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &particlePipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create particle pipeline layout!");
}

// -------------------------
// Task 5: Particle pipeline
// -------------------------
void Renderer::createParticlePipeline()
{
    // You need to compile these to SPIR-V:
    // shaders/particle.vert -> shaders/particle.vert.spv
    // shaders/particle.frag -> shaders/particle.frag.spv
    //
    // Vertex shader expectation:
    //  - set 0 binding 0 = UBO (same UniformBufferObject)
    //  - location 0: vec4 inPosSize (xyz pos, w size)
    //  - location 1: vec4 inColor

    auto const vertCode = readFile("shaders/particle.vert.spv");
    auto const fragCode = readFile("shaders/particle.frag.spv");

    VkShaderModule const vertModule = createShaderModule(vertCode);
    VkShaderModule const fragModule = createShaderModule(fragCode);

    VkPipelineShaderStageCreateInfo vertStage{};
    vertStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertModule;
    vertStage.pName = kShaderEntryPoint;

    VkPipelineShaderStageCreateInfo fragStage{};
    fragStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragModule;
    fragStage.pName = kShaderEntryPoint;

    VkPipelineShaderStageCreateInfo stages[] = { vertStage, fragStage };

    // ---- Vertex input: INSTANCE DATA ONLY ----
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(GpuParticleInstance);
    binding.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;

    VkVertexInputAttributeDescription attrs[2]{};
    attrs[0].location = 0;
    attrs[0].binding = 0;
    attrs[0].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    attrs[0].offset = offsetof(GpuParticleInstance, pos_size); // should be 0

    attrs[1].location = 1;
    attrs[1].binding = 0;
    attrs[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
    attrs[1].offset = offsetof(GpuParticleInstance, color);

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = 2;
    vertexInput.pVertexAttributeDescriptions = attrs;

    auto const common = MakeGlobePipelineStates();   // or MakeParticlePipelineStates()

    auto const base = MakeAlphaBlendPipeBase(
        stages,
        2,
        &vertexInput,
        common
    );

    VkGraphicsPipelineCreateInfo pipe = base.pipe;

    pipe.layout = particlePipelineLayout;
    pipe.renderPass = renderPass;
    pipe.subpass = 0;
    std::cout << "[PipelineDebug] Creating particle pipeline\n";
    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipe, nullptr, &particlePipeline) != VK_SUCCESS)
        throw std::runtime_error("failed to create particle pipeline!");

    vkDestroyShaderModule(device, fragModule, nullptr);
    vkDestroyShaderModule(device, vertModule, nullptr);
}

void Renderer::destroyParticlePipeline()
{
    if (particlePipeline)
    {
        vkDestroyPipeline(device, particlePipeline, nullptr);
        particlePipeline = VK_NULL_HANDLE;
    }

    if (particlePipelineLayout)
    {
        vkDestroyPipelineLayout(device, particlePipelineLayout, nullptr);
        particlePipelineLayout = VK_NULL_HANDLE;
    }
}

// -------------------------
// Task 6: Record draw
// -------------------------
void Renderer::recordParticles(VkCommandBuffer cmd, uint32_t /*imageIndex*/)
{
    if (!currentScene) return;
    if (particleCount == 0) return;
    if (!particlePipeline) return;

    // Bind pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, particlePipeline);

    // Bind set 0 (UBO) - indexed by currentFrame in your renderer
    VkDescriptorSet const set0 = uboDescriptorSets[currentFrame];
    vkCmdBindDescriptorSets(
        cmd,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        particlePipelineLayout,
        0,
        1,
        &set0,
        0,
        nullptr
    );

    // Bind instance buffer for this frame
    VkBuffer vb[] = { particleInstanceBuffers[currentFrame] };
    VkDeviceSize off[] = { 0 };
    vkCmdBindVertexBuffers(cmd, 0, 1, vb, off);

    // Draw 6 verts per instance (vertex shader uses gl_VertexIndex for quad)
    vkCmdDraw(cmd, 6, particleCount, 0, 0);
}
