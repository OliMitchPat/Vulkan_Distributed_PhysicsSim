#pragma once
#include "Vertex.h"    
// Core graphics headers
#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "RenderScene.h"
// STL
#include <vector>
#include <array>
#include <optional>
#include <string>
#include <cstdint>
#include <mutex>


#include "RenderResources.h"
#include <functional>

using UiBuildFn = std::function<void()>;
// ---------- Helper structs used by Renderer ----------
inline constexpr const char* kShaderEntryPoint = "main";

struct QueueFamilyIndices {
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete() const {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

static constexpr int MAX_SPARK_LIGHTS = 32;

struct PointLight
{
    glm::vec4 pos_radius; // xyz pos, w radius
    glm::vec4 col_int;    // rgb color, w intensity
};

struct UniformBufferObject
{
    glm::mat4 view;
    glm::mat4 proj;

    glm::vec4 cameraPos;

    glm::vec4 sunDir_Int;
    glm::vec4 moonDir_Int;

    glm::vec4 ambient;

    PointLight sparkLights[MAX_SPARK_LIGHTS];

    glm::ivec4 sparkLightCount_pad; // x = count
};

struct PushConstantObject1 {
    glm::mat4 model;
    glm::vec4 params;
    glm::vec4 objectColor;
};


// ---------- Renderer class declaration ----------

class Renderer final {
public:
    explicit Renderer(GLFWwindow* window);
    ~Renderer() noexcept;

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    void drawFrame();
    void waitIdle() const;

    void render(const RenderScene& scene);
    float getAspectRatio() const;

    void setUiBuildFn(UiBuildFn fn) { uiBuildFn = std::move(fn); }
    void clearUiBuildFn() { uiBuildFn = nullptr; }

    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

    void BeginImGuiFrame(); 
    void EndImGuiFrame();  

    VkDevice GetDevice() const { return device; }
    VkPhysicalDevice GetPhysicalDevice() const { return physicalDevice; }
    VkQueue GetGraphicsQueue() const { return graphicsQueue; }
    uint32_t GetGraphicsQueueFamily() const;
    std::mutex& GetQueueMutex() { return queueMutex; }
private:
    const RenderScene* currentScene = nullptr;

    // --- GLFW / window ---
    GLFWwindow* window = nullptr;
    bool framebufferResized = false;

    // Matrices driven by ECS / RenderScene
    glm::mat4 currentModel{ 1.0f };
    glm::mat4 currentView{ 1.0f };
    glm::mat4 currentProj{ 1.0f };

    // --- Vulkan core ---
    void initVulkan();
    void cleanupVulkan();

    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;

    // --- Physical & logical device ---
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue graphicsQueue = VK_NULL_HANDLE;
    VkQueue presentQueue = VK_NULL_HANDLE;
    mutable std::mutex queueMutex;

    // --- Swapchain ---
    VkSwapchainKHR swapChain = VK_NULL_HANDLE;
    std::vector<VkImage> swapChainImages;
    std::vector<VkImageView> swapChainImageViews;
    VkFormat swapChainImageFormat{};
    VkExtent2D swapChainExtent{};
    std::vector<VkFramebuffer> swapChainFramebuffers;

    // --- Depth resources ---
    VkImage depthImage = VK_NULL_HANDLE;
    VkDeviceMemory depthImageMemory = VK_NULL_HANDLE;
    VkImageView depthImageView = VK_NULL_HANDLE;

    // --- Pipeline & render pass ---
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout uboSetLayout = VK_NULL_HANDLE;      
    VkDescriptorSetLayout textureSetLayout = VK_NULL_HANDLE;  
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline graphicsPipeline = VK_NULL_HANDLE;
    VkPipeline transparentGraphicsPipeline = VK_NULL_HANDLE;
    // --- Texture ---
    VkImage textureImage = VK_NULL_HANDLE;
    VkDeviceMemory textureImageMemory = VK_NULL_HANDLE;
    VkImageView textureImageView = VK_NULL_HANDLE;
    VkSampler textureSampler = VK_NULL_HANDLE;

    // --- Buffers ---
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexBufferMemory = VK_NULL_HANDLE;

    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexBufferMemory = VK_NULL_HANDLE;

    std::vector<VkBuffer> uniformBuffers;
    std::vector<VkDeviceMemory> uniformBuffersMemory;
    std::vector<void*> uniformBuffersMapped;

    UiBuildFn uiBuildFn = nullptr;
    // ---------- Particle rendering (instance data + pipeline) ----------

    struct GpuParticleInstance
    {
        float pos_size[4]; // xyz pos, w size
        float color[4];    // rgba
    };

    // GPU instance buffers (one per frame-in-flight)
    static constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2; // must match your sync object count

    std::array<VkBuffer, MAX_FRAMES_IN_FLIGHT> particleInstanceBuffers{ VK_NULL_HANDLE, VK_NULL_HANDLE };
    std::array<VkDeviceMemory, MAX_FRAMES_IN_FLIGHT> particleInstanceMemories{ VK_NULL_HANDLE, VK_NULL_HANDLE };
    std::array<void*, MAX_FRAMES_IN_FLIGHT> particleInstanceMapped{ nullptr, nullptr };

    uint32_t particleCount = 0;
    uint32_t maxParticles = 20000;

    // Pipeline for particles (separate from mesh pipeline)
    VkPipelineLayout particlePipelineLayout = VK_NULL_HANDLE;
    VkPipeline particlePipeline = VK_NULL_HANDLE;

    // Particle API (implemented in RendererParticles.cpp)
    void createParticleInstanceBuffers();
    void destroyParticleInstanceBuffers();
    void updateParticleInstanceBuffer(uint32_t frameIndex);

    void createParticlePipelineLayout();
    void createParticlePipeline();
    void destroyParticlePipeline();

    void recordParticles(VkCommandBuffer cmd, uint32_t imageIndex);

	//________GlobeStuf__________
    VkPipeline globePipeline = VK_NULL_HANDLE;
    VkDescriptorSetLayout globeSkySetLayout = VK_NULL_HANDLE;
    VkPipelineLayout globePipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSet globeSkyDescriptorSet = VK_NULL_HANDLE;

    void createGlobePipeline();
    void destroyGlobePipeline();
    void createGlobeSkySetLayout();
    void createGlobePipelineLayout();
    void createGlobeSkyDescriptorSet();
    // --- Descriptors ---
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    // Set 0 (per-frame UBO)
    std::vector<VkDescriptorSet> uboDescriptorSets;

    //IMGUI
    void createImGuiPool();
    void initImGui();
    VkDescriptorPool imguiPool = VK_NULL_HANDLE;
    bool imguiInitialized = false;

    // --- Commands & sync ---
    VkCommandPool commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
    uint32_t currentFrame = 0;

	// --- Resource management ---
    RenderResources resources;
    const GpuTexture& getOrLoadTexture(const std::string& textureName);

    GpuMesh fallbackMesh;
    GpuTexture fallbackTexture;

    const GpuMesh& getOrLoadMesh(const std::string& meshName);

    GpuMesh createGpuMeshFromCpu(
        const std::vector<Vertex>& verts,
        const std::vector<uint32_t>& inds
    );
    void destroyMesh(GpuMesh& m)const;

    // ---------- Creation / teardown helpers ----------

    void recreateSwapChain();
    void cleanupSwapChain();

    void createInstance();
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapChain();
    void createImageViews();
    void createRenderPass();
    void createDescriptorSetLayouts();
    void createGraphicsPipeline();
    void createDepthResources();
    void createFramebuffers();
    void createCommandPool();
    void createTextureImage();
    void createTextureImageView();
    void createTextureSampler();
    void createVertexBuffer();
    void createIndexBuffer();
    void createUniformBuffers();
    void createDescriptorPool();
    void initFallbackTextureMagenta();
    void createCommandBuffers();
    void createSyncObjects();
	void createUboDescriptorSets();
	void createTerrain();

    // ---------- Per-frame / drawing ----------

    void updateUniformBuffer(uint32_t currentImage);
    void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);

    // ---------- Low-level utilities ----------

    VkShaderModule createShaderModule(const std::vector<char>& code) const;

    VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags) const;
    void createImage(uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling,
        VkImageUsageFlags usage, VkMemoryPropertyFlags properties,
        VkImage& image, VkDeviceMemory& imageMemory)const;

    void transitionImageLayout(VkImage image, VkFormat format,
        VkImageLayout oldLayout, VkImageLayout newLayout)const;
    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height)const;

    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties,
        VkBuffer& buffer, VkDeviceMemory& bufferMemory);

    VkCommandBuffer beginSingleTimeCommands() const;
    void endSingleTimeCommands(VkCommandBuffer commandBuffer) const;

    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)const;

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

    VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates,
        VkImageTiling tiling,
        VkFormatFeatureFlags features)const;
    VkFormat findDepthFormat()const;
    bool hasStencilComponent(VkFormat format)const;

    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device)const;
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device)const;

    bool isDeviceSuitable(VkPhysicalDevice device)const;
    bool checkDeviceExtensionSupport(VkPhysicalDevice device)const;
    bool checkValidationLayerSupport()const;

    std::vector<const char*> getRequiredExtensions()const;
    static std::vector<char> readFile(const std::string& filename);

    // debug callback is only used internally
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
        void* pUserData);

    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo)const;

    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats)const;
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes)const;
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities)const;

    GpuMesh makeNonOwningMeshRef(const GpuMesh& m);
    GpuTexture makeNonOwningTextureRef(const GpuTexture& t);
};
