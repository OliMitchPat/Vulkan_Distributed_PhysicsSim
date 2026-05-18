#pragma once

#include <vulkan/vulkan.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

struct GpuBoid
{
    glm::vec4 position{ 0.0f };
    glm::vec4 velocity{ 0.0f };
};

struct GpuFlockingParams
{
    uint32_t boidCount = 0;
    float dt = 0.0f;
    float perceptionRadius = 0.0f;
    float separationRadius = 0.0f;

    float cohesionWeight = 0.0f;
    float alignmentWeight = 0.0f;
    float separationWeight = 0.0f;
    float avoidanceWeight = 0.0f;

    float maxSpeed = 0.0f;
    float maxForce = 0.0f;
    float boundsMargin = 3.0f;
    float boundsAvoidanceStrength = 20.0f;

    glm::vec4 boundsMin{ -20.0f, 0.0f, -20.0f, 0.0f };
    glm::vec4 boundsMax{ 20.0f, 20.0f, 20.0f, 0.0f };
};

static_assert(sizeof(GpuBoid) == sizeof(float) * 8);
static_assert(sizeof(GpuFlockingParams) == 80);

struct GpuFlockingTiming
{
    float uploadMs = 0.0f;
    float dispatchMs = 0.0f;
    float readbackMs = 0.0f;
    float totalMs = 0.0f;
};

class GpuFlockingCompute
{
public:
    bool Initialise(
        VkDevice device,
        VkPhysicalDevice physicalDevice,
        VkQueue computeQueue,
        uint32_t computeQueueFamily,
        std::mutex* queueMutex);

    void Resize(uint32_t boidCount);
    void UploadBoids(const std::vector<GpuBoid>& cpuBoids);
    void Dispatch(const GpuFlockingParams& params);
    void DownloadBoids(std::vector<GpuBoid>& cpuBoids);
    void UpdateBoids(std::vector<GpuBoid>& cpuBoids, const GpuFlockingParams& params);
    void Shutdown();

    bool IsAvailable() const { return m_available; }
    const std::string& LastError() const { return m_lastError; }
    const GpuFlockingTiming& Timing() const { return m_timing; }

private:
    void CreatePipeline();
    void CreateBuffers(uint32_t boidCount);
    void DestroyBuffers();
    void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& memory);
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
    VkShaderModule CreateShaderModule(const std::vector<char>& code) const;
    static std::vector<char> ReadFile(const std::string& filename);
    VkDescriptorSet ActiveDescriptorSet() const;
    VkBuffer OutputBuffer() const;

private:
    VkDevice m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkQueue m_computeQueue = VK_NULL_HANDLE;
    uint32_t m_computeQueueFamily = 0;
    std::mutex* m_queueMutex = nullptr;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_computePipeline = VK_NULL_HANDLE;

    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSetA = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSetB = VK_NULL_HANDLE;

    VkBuffer m_boidBufferA = VK_NULL_HANDLE;
    VkBuffer m_boidBufferB = VK_NULL_HANDLE;
    VkDeviceMemory m_boidMemoryA = VK_NULL_HANDLE;
    VkDeviceMemory m_boidMemoryB = VK_NULL_HANDLE;

    VkBuffer m_uploadStagingBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_uploadStagingMemory = VK_NULL_HANDLE;
    VkBuffer m_readbackStagingBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_readbackStagingMemory = VK_NULL_HANDLE;

    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    VkCommandBuffer m_commandBuffer = VK_NULL_HANDLE;
    VkFence m_fence = VK_NULL_HANDLE;

    uint32_t m_currentInputIndex = 0;
    uint32_t m_boidCapacity = 0;
    VkDeviceSize m_bufferSize = 0;
    bool m_available = false;
    std::string m_lastError;
    GpuFlockingTiming m_timing{};
};
