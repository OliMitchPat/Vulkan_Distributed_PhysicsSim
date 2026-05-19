#include "GpuFlockingCompute.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace
{
    constexpr uint32_t kWorkGroupSize = 256;

    float MsBetween(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
    {
        return std::chrono::duration<float, std::milli>(b - a).count();
    }
}

bool GpuFlockingCompute::Initialise(
    VkDevice device,
    VkPhysicalDevice physicalDevice,
    VkQueue computeQueue,
    uint32_t computeQueueFamily,
    std::mutex* queueMutex)
{
    m_device = device;
    m_physicalDevice = physicalDevice;
    m_computeQueue = computeQueue;
    m_computeQueueFamily = computeQueueFamily;
    m_queueMutex = queueMutex;

    try
    {
        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = m_computeQueueFamily;
        if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool) != VK_SUCCESS)
            throw std::runtime_error("failed to create GPU flocking command pool");

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_commandPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        if (vkAllocateCommandBuffers(m_device, &allocInfo, &m_commandBuffer) != VK_SUCCESS)
            throw std::runtime_error("failed to allocate GPU flocking command buffer");

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        if (vkCreateFence(m_device, &fenceInfo, nullptr, &m_fence) != VK_SUCCESS)
            throw std::runtime_error("failed to create GPU flocking fence");

        CreatePipeline();
        m_available = true;
        m_lastError.clear();
        return true;
    }
    catch (const std::exception& e)
    {
        m_lastError = e.what();
        Shutdown();
        return false;
    }
}

void GpuFlockingCompute::CreatePipeline()
{
    VkDescriptorSetLayoutBinding inputBinding{};
    inputBinding.binding = 0;
    inputBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    inputBinding.descriptorCount = 1;
    inputBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutBinding outputBinding{};
    outputBinding.binding = 1;
    outputBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    outputBinding.descriptorCount = 1;
    outputBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutBinding obstacleBinding{};
    obstacleBinding.binding = 2;
    obstacleBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    obstacleBinding.descriptorCount = 1;
    obstacleBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    const VkDescriptorSetLayoutBinding bindings[] = { inputBinding, outputBinding, obstacleBinding };
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = bindings;
    if (vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &m_descriptorSetLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create GPU flocking descriptor set layout");

    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(GpuFlockingParams);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;
    if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create GPU flocking pipeline layout");

    const auto code = ReadFile("shaders/flocking_compute.comp.spv");
    VkShaderModule shaderModule = CreateShaderModule(code);

    VkPipelineShaderStageCreateInfo shaderStage{};
    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.module = shaderModule;
    shaderStage.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = shaderStage;
    pipelineInfo.layout = m_pipelineLayout;

    const VkResult result = vkCreateComputePipelines(
        m_device,
        VK_NULL_HANDLE,
        1,
        &pipelineInfo,
        nullptr,
        &m_computePipeline);

    vkDestroyShaderModule(m_device, shaderModule, nullptr);
    if (result != VK_SUCCESS)
        throw std::runtime_error("failed to create GPU flocking compute pipeline");
}

void GpuFlockingCompute::Resize(uint32_t boidCount)
{
    Resize(boidCount, 0);
}

void GpuFlockingCompute::Resize(uint32_t boidCount, uint32_t obstacleCount)
{
    if (!m_available)
        return;

    if (boidCount == 0)
        return;

    if (boidCount <= m_boidCapacity && std::max<uint32_t>(obstacleCount, 1) <= m_obstacleCapacity)
        return;

    CreateBuffers(boidCount, obstacleCount);
}

void GpuFlockingCompute::CreateBuffers(uint32_t boidCount)
{
    CreateBuffers(boidCount, 0);
}

void GpuFlockingCompute::CreateBuffers(uint32_t boidCount, uint32_t obstacleCount)
{
    if (m_queueMutex)
    {
        std::lock_guard<std::mutex> lk(*m_queueMutex);
        vkDeviceWaitIdle(m_device);
    }
    else
    {
        vkDeviceWaitIdle(m_device);
    }
    DestroyBuffers();

    m_boidCapacity = std::max<uint32_t>(boidCount, 256);
    m_bufferSize = sizeof(GpuBoid) * static_cast<VkDeviceSize>(m_boidCapacity);
    m_obstacleCapacity = std::max<uint32_t>(obstacleCount, 1);
    m_obstacleBufferSize = sizeof(GpuFlockingObstacle) * static_cast<VkDeviceSize>(m_obstacleCapacity);

    const VkBufferUsageFlags gpuUsage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    CreateBuffer(m_bufferSize, gpuUsage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_boidBufferA, m_boidMemoryA);
    CreateBuffer(m_bufferSize, gpuUsage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_boidBufferB, m_boidMemoryB);
    CreateBuffer(
        m_obstacleBufferSize,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        m_obstacleBuffer,
        m_obstacleMemory);

    CreateBuffer(
        m_bufferSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        m_uploadStagingBuffer,
        m_uploadStagingMemory);

    CreateBuffer(
        m_bufferSize,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        m_readbackStagingBuffer,
        m_readbackStagingMemory);
    CreateBuffer(
        m_obstacleBufferSize,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        m_obstacleStagingBuffer,
        m_obstacleStagingMemory);

    if (!m_descriptorPool)
    {
        VkDescriptorPoolSize poolSize{};
        poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        poolSize.descriptorCount = 6;

        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        poolInfo.maxSets = 2;
        if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS)
            throw std::runtime_error("failed to create GPU flocking descriptor pool");
    }
    else
    {
        vkResetDescriptorPool(m_device, m_descriptorPool, 0);
    }

    VkDescriptorSetLayout layouts[] = { m_descriptorSetLayout, m_descriptorSetLayout };
    VkDescriptorSet sets[] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = m_descriptorPool;
    allocInfo.descriptorSetCount = 2;
    allocInfo.pSetLayouts = layouts;
    if (vkAllocateDescriptorSets(m_device, &allocInfo, sets) != VK_SUCCESS)
        throw std::runtime_error("failed to allocate GPU flocking descriptor sets");

    m_descriptorSetA = sets[0];
    m_descriptorSetB = sets[1];

    auto writeSet = [&](VkDescriptorSet set, VkBuffer input, VkBuffer output)
    {
        VkDescriptorBufferInfo inputInfo{};
        inputInfo.buffer = input;
        inputInfo.offset = 0;
        inputInfo.range = m_bufferSize;

        VkDescriptorBufferInfo outputInfo{};
        outputInfo.buffer = output;
        outputInfo.offset = 0;
        outputInfo.range = m_bufferSize;

        VkDescriptorBufferInfo obstacleInfo{};
        obstacleInfo.buffer = m_obstacleBuffer;
        obstacleInfo.offset = 0;
        obstacleInfo.range = m_obstacleBufferSize;

        VkWriteDescriptorSet writes[3]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = set;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &inputInfo;

        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = set;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &outputInfo;

        writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet = set;
        writes[2].dstBinding = 2;
        writes[2].descriptorCount = 1;
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[2].pBufferInfo = &obstacleInfo;

        vkUpdateDescriptorSets(m_device, 3, writes, 0, nullptr);
    };

    writeSet(m_descriptorSetA, m_boidBufferA, m_boidBufferB);
    writeSet(m_descriptorSetB, m_boidBufferB, m_boidBufferA);
    m_currentInputIndex = 0;
}

void GpuFlockingCompute::UploadBoids(const std::vector<GpuBoid>& cpuBoids)
{
    if (!m_available || cpuBoids.empty())
        return;

    const auto start = std::chrono::steady_clock::now();
    Resize(static_cast<uint32_t>(cpuBoids.size()));

    const VkDeviceSize size = sizeof(GpuBoid) * static_cast<VkDeviceSize>(cpuBoids.size());
    void* mapped = nullptr;
    vkMapMemory(m_device, m_uploadStagingMemory, 0, size, 0, &mapped);
    std::memcpy(mapped, cpuBoids.data(), static_cast<size_t>(size));
    vkUnmapMemory(m_device, m_uploadStagingMemory);

    CopyBuffer(m_uploadStagingBuffer, m_currentInputIndex == 0 ? m_boidBufferA : m_boidBufferB, size);

    const auto end = std::chrono::steady_clock::now();
    m_timing.uploadMs = MsBetween(start, end);
}

void GpuFlockingCompute::Dispatch(const GpuFlockingParams& params)
{
    if (!m_available || params.boidCount == 0)
        return;

    const auto start = std::chrono::steady_clock::now();
    vkResetCommandBuffer(m_commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(m_commandBuffer, &beginInfo);

    vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);

    VkDescriptorSet descriptorSet = ActiveDescriptorSet();
    vkCmdBindDescriptorSets(
        m_commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        m_pipelineLayout,
        0,
        1,
        &descriptorSet,
        0,
        nullptr);

    vkCmdPushConstants(
        m_commandBuffer,
        m_pipelineLayout,
        VK_SHADER_STAGE_COMPUTE_BIT,
        0,
        sizeof(GpuFlockingParams),
        &params);

    const uint32_t groupCount = (params.boidCount + kWorkGroupSize - 1) / kWorkGroupSize;
    vkCmdDispatch(m_commandBuffer, groupCount, 1, 1);

    VkBufferMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = OutputBuffer();
    barrier.offset = 0;
    barrier.size = VK_WHOLE_SIZE;

    vkCmdPipelineBarrier(
        m_commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0,
        nullptr,
        1,
        &barrier,
        0,
        nullptr);

    VkBufferCopy copy{};
    copy.size = sizeof(GpuBoid) * static_cast<VkDeviceSize>(params.boidCount);
    vkCmdCopyBuffer(m_commandBuffer, OutputBuffer(), m_readbackStagingBuffer, 1, &copy);

    vkEndCommandBuffer(m_commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;

    vkResetFences(m_device, 1, &m_fence);
    if (m_queueMutex)
    {
        std::lock_guard<std::mutex> lk(*m_queueMutex);
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }
    else
    {
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }

    vkWaitForFences(m_device, 1, &m_fence, VK_TRUE, UINT64_MAX);
    m_currentInputIndex = 1 - m_currentInputIndex;

    const auto end = std::chrono::steady_clock::now();
    m_timing.dispatchMs = MsBetween(start, end);
}

void GpuFlockingCompute::DownloadBoids(std::vector<GpuBoid>& cpuBoids)
{
    if (!m_available || cpuBoids.empty())
        return;

    const auto start = std::chrono::steady_clock::now();
    const VkDeviceSize size = sizeof(GpuBoid) * static_cast<VkDeviceSize>(cpuBoids.size());
    void* mapped = nullptr;
    vkMapMemory(m_device, m_readbackStagingMemory, 0, size, 0, &mapped);
    std::memcpy(cpuBoids.data(), mapped, static_cast<size_t>(size));
    vkUnmapMemory(m_device, m_readbackStagingMemory);
    const auto end = std::chrono::steady_clock::now();

    m_timing.readbackMs = MsBetween(start, end);
    m_timing.totalMs = m_timing.uploadMs + m_timing.dispatchMs + m_timing.readbackMs;
}

void GpuFlockingCompute::UpdateBoids(std::vector<GpuBoid>& cpuBoids, const std::vector<GpuFlockingObstacle>& obstacles, const GpuFlockingParams& params)
{
    if (!m_available || cpuBoids.empty() || params.boidCount == 0)
        return;

    const auto totalStart = std::chrono::steady_clock::now();
    Resize(static_cast<uint32_t>(cpuBoids.size()), static_cast<uint32_t>(obstacles.size()));

    const VkDeviceSize size = sizeof(GpuBoid) * static_cast<VkDeviceSize>(cpuBoids.size());
    const VkDeviceSize obstacleSize = sizeof(GpuFlockingObstacle) * static_cast<VkDeviceSize>(obstacles.size());

    const auto uploadStart = std::chrono::steady_clock::now();
    void* uploadMapped = nullptr;
    vkMapMemory(m_device, m_uploadStagingMemory, 0, size, 0, &uploadMapped);
    std::memcpy(uploadMapped, cpuBoids.data(), static_cast<size_t>(size));
    vkUnmapMemory(m_device, m_uploadStagingMemory);

    if (!obstacles.empty())
    {
        void* obstacleMapped = nullptr;
        vkMapMemory(m_device, m_obstacleStagingMemory, 0, obstacleSize, 0, &obstacleMapped);
        std::memcpy(obstacleMapped, obstacles.data(), static_cast<size_t>(obstacleSize));
        vkUnmapMemory(m_device, m_obstacleStagingMemory);
    }
    const auto uploadEnd = std::chrono::steady_clock::now();

    const auto dispatchStart = std::chrono::steady_clock::now();
    vkResetCommandBuffer(m_commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(m_commandBuffer, &beginInfo);

    VkBuffer inputBuffer = m_currentInputIndex == 0 ? m_boidBufferA : m_boidBufferB;
    VkBuffer outputBuffer = OutputBuffer();

    VkBufferCopy uploadCopy{};
    uploadCopy.size = size;
    vkCmdCopyBuffer(m_commandBuffer, m_uploadStagingBuffer, inputBuffer, 1, &uploadCopy);
    if (!obstacles.empty())
    {
        VkBufferCopy obstacleCopy{};
        obstacleCopy.size = obstacleSize;
        vkCmdCopyBuffer(m_commandBuffer, m_obstacleStagingBuffer, m_obstacleBuffer, 1, &obstacleCopy);
    }

    VkBufferMemoryBarrier uploadBarriers[2]{};
    uploadBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    uploadBarriers[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    uploadBarriers[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    uploadBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    uploadBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    uploadBarriers[0].buffer = inputBuffer;
    uploadBarriers[0].offset = 0;
    uploadBarriers[0].size = size;

    uploadBarriers[1].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    uploadBarriers[1].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    uploadBarriers[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    uploadBarriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    uploadBarriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    uploadBarriers[1].buffer = m_obstacleBuffer;
    uploadBarriers[1].offset = 0;
    uploadBarriers[1].size = obstacles.empty() ? VK_WHOLE_SIZE : obstacleSize;

    vkCmdPipelineBarrier(
        m_commandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0,
        0,
        nullptr,
        obstacles.empty() ? 1 : 2,
        uploadBarriers,
        0,
        nullptr);

    vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);

    VkDescriptorSet descriptorSet = ActiveDescriptorSet();
    vkCmdBindDescriptorSets(
        m_commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        m_pipelineLayout,
        0,
        1,
        &descriptorSet,
        0,
        nullptr);

    vkCmdPushConstants(
        m_commandBuffer,
        m_pipelineLayout,
        VK_SHADER_STAGE_COMPUTE_BIT,
        0,
        sizeof(GpuFlockingParams),
        &params);

    const uint32_t groupCount = (params.boidCount + kWorkGroupSize - 1) / kWorkGroupSize;
    vkCmdDispatch(m_commandBuffer, groupCount, 1, 1);

    VkBufferMemoryBarrier readbackBarrier{};
    readbackBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    readbackBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    readbackBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    readbackBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    readbackBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    readbackBarrier.buffer = outputBuffer;
    readbackBarrier.offset = 0;
    readbackBarrier.size = size;

    vkCmdPipelineBarrier(
        m_commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0,
        nullptr,
        1,
        &readbackBarrier,
        0,
        nullptr);

    VkBufferCopy readbackCopy{};
    readbackCopy.size = size;
    vkCmdCopyBuffer(m_commandBuffer, outputBuffer, m_readbackStagingBuffer, 1, &readbackCopy);

    vkEndCommandBuffer(m_commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;

    vkResetFences(m_device, 1, &m_fence);
    if (m_queueMutex)
    {
        std::lock_guard<std::mutex> lk(*m_queueMutex);
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }
    else
    {
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }

    vkWaitForFences(m_device, 1, &m_fence, VK_TRUE, UINT64_MAX);
    m_currentInputIndex = 1 - m_currentInputIndex;
    const auto dispatchEnd = std::chrono::steady_clock::now();

    const auto readbackStart = std::chrono::steady_clock::now();
    void* readbackMapped = nullptr;
    vkMapMemory(m_device, m_readbackStagingMemory, 0, size, 0, &readbackMapped);
    std::memcpy(cpuBoids.data(), readbackMapped, static_cast<size_t>(size));
    vkUnmapMemory(m_device, m_readbackStagingMemory);
    const auto readbackEnd = std::chrono::steady_clock::now();

    m_timing.uploadMs = MsBetween(uploadStart, uploadEnd);
    m_timing.dispatchMs = MsBetween(dispatchStart, dispatchEnd);
    m_timing.readbackMs = MsBetween(readbackStart, readbackEnd);
    m_timing.totalMs = MsBetween(totalStart, readbackEnd);
}

void GpuFlockingCompute::Shutdown()
{
    if (!m_device)
        return;

    if (m_queueMutex)
    {
        std::lock_guard<std::mutex> lk(*m_queueMutex);
        vkDeviceWaitIdle(m_device);
    }
    else
    {
        vkDeviceWaitIdle(m_device);
    }
    DestroyBuffers();

    if (m_descriptorPool) vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
    if (m_computePipeline) vkDestroyPipeline(m_device, m_computePipeline, nullptr);
    if (m_pipelineLayout) vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
    if (m_descriptorSetLayout) vkDestroyDescriptorSetLayout(m_device, m_descriptorSetLayout, nullptr);
    if (m_fence) vkDestroyFence(m_device, m_fence, nullptr);
    if (m_commandPool) vkDestroyCommandPool(m_device, m_commandPool, nullptr);

    m_descriptorPool = VK_NULL_HANDLE;
    m_computePipeline = VK_NULL_HANDLE;
    m_pipelineLayout = VK_NULL_HANDLE;
    m_descriptorSetLayout = VK_NULL_HANDLE;
    m_fence = VK_NULL_HANDLE;
    m_commandPool = VK_NULL_HANDLE;
    m_commandBuffer = VK_NULL_HANDLE;
    m_available = false;
}

void GpuFlockingCompute::DestroyBuffers()
{
    auto destroyBuffer = [&](VkBuffer& buffer, VkDeviceMemory& memory)
    {
        if (buffer) vkDestroyBuffer(m_device, buffer, nullptr);
        if (memory) vkFreeMemory(m_device, memory, nullptr);
        buffer = VK_NULL_HANDLE;
        memory = VK_NULL_HANDLE;
    };

    destroyBuffer(m_boidBufferA, m_boidMemoryA);
    destroyBuffer(m_boidBufferB, m_boidMemoryB);
    destroyBuffer(m_uploadStagingBuffer, m_uploadStagingMemory);
    destroyBuffer(m_readbackStagingBuffer, m_readbackStagingMemory);
    destroyBuffer(m_obstacleBuffer, m_obstacleMemory);
    destroyBuffer(m_obstacleStagingBuffer, m_obstacleStagingMemory);
    m_descriptorSetA = VK_NULL_HANDLE;
    m_descriptorSetB = VK_NULL_HANDLE;
    m_boidCapacity = 0;
    m_obstacleCapacity = 0;
    m_bufferSize = 0;
    m_obstacleBufferSize = 0;
}

void GpuFlockingCompute::CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& memory)
{
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
        throw std::runtime_error("failed to create GPU flocking buffer");

    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(m_device, buffer, &requirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = requirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(requirements.memoryTypeBits, properties);

    if (vkAllocateMemory(m_device, &allocInfo, nullptr, &memory) != VK_SUCCESS)
        throw std::runtime_error("failed to allocate GPU flocking buffer memory");

    vkBindBufferMemory(m_device, buffer, memory, 0);
}

void GpuFlockingCompute::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
    vkResetCommandBuffer(m_commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(m_commandBuffer, &beginInfo);

    VkBufferCopy copy{};
    copy.size = size;
    vkCmdCopyBuffer(m_commandBuffer, src, dst, 1, &copy);

    vkEndCommandBuffer(m_commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;

    vkResetFences(m_device, 1, &m_fence);
    if (m_queueMutex)
    {
        std::lock_guard<std::mutex> lk(*m_queueMutex);
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }
    else
    {
        vkQueueSubmit(m_computeQueue, 1, &submitInfo, m_fence);
    }
    vkWaitForFences(m_device, 1, &m_fence, VK_TRUE, UINT64_MAX);
}

uint32_t GpuFlockingCompute::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
    VkPhysicalDeviceMemoryProperties memProperties{};
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; ++i)
    {
        if ((typeFilter & (1u << i)) &&
            (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
        {
            return i;
        }
    }

    throw std::runtime_error("failed to find GPU flocking memory type");
}

VkShaderModule GpuFlockingCompute::CreateShaderModule(const std::vector<char>& code) const
{
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    VkShaderModule shaderModule = VK_NULL_HANDLE;
    if (vkCreateShaderModule(m_device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
        throw std::runtime_error("failed to create GPU flocking shader module");

    return shaderModule;
}

std::vector<char> GpuFlockingCompute::ReadFile(const std::string& filename)
{
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file)
        throw std::runtime_error("failed to open " + filename);

    const size_t size = static_cast<size_t>(file.tellg());
    std::vector<char> buffer(size);
    file.seekg(0);
    file.read(buffer.data(), static_cast<std::streamsize>(size));
    return buffer;
}

VkDescriptorSet GpuFlockingCompute::ActiveDescriptorSet() const
{
    return m_currentInputIndex == 0 ? m_descriptorSetA : m_descriptorSetB;
}

VkBuffer GpuFlockingCompute::OutputBuffer() const
{
    return m_currentInputIndex == 0 ? m_boidBufferB : m_boidBufferA;
}
