#pragma once
#include <vulkan/vulkan.h>
#include <string>
#include <unordered_map>

struct GpuMesh
{
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;

    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;
    VkIndexType indexType = VK_INDEX_TYPE_UINT16;

    uint32_t indexCount = 0;
    bool valid = false;
    bool owned = true;

    GpuMesh() = default;
    ~GpuMesh() = default;

    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    GpuMesh(GpuMesh&&) noexcept = default;
    GpuMesh& operator=(GpuMesh&&) noexcept = default;
};


struct GpuTexture
{
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;

    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;

    bool valid = false;
    bool owned = true;

    GpuTexture() = default;
    ~GpuTexture() = default;

    GpuTexture(const GpuTexture&) = delete;
    GpuTexture& operator=(const GpuTexture&) = delete;

    GpuTexture(GpuTexture&&) noexcept = default;
    GpuTexture& operator=(GpuTexture&&) noexcept = default;
};


class RenderResources final
{
public:
    const std::string& getModelsDir() const noexcept { return modelsDir; }
    const std::string& getTexturesDir() const noexcept { return texturesDir; }
    void setModelsDir(const std::string& dir) { modelsDir = dir; }
    void setTexturesDir(const std::string& dir) { texturesDir = dir; }


    const std::unordered_map<std::string, GpuMesh>& getMeshes() const noexcept { return meshes; }
    const std::unordered_map<std::string, GpuTexture>& getTextures() const noexcept { return textures; }

    GpuMesh* findMesh(const std::string& name) noexcept {
        auto const it = meshes.find(name);
        return (it != meshes.end()) ? &it->second : nullptr;
    }
    void cacheMesh(const std::string& name, GpuMesh&& mesh) {
        meshes[name] = std::move(mesh);
    }

    GpuTexture* findTexture(const std::string& name) noexcept {
        auto const it = textures.find(name);
        return (it != textures.end()) ? &it->second : nullptr;
    }
    void cacheTexture(const std::string& name, GpuTexture&& tex) {
        textures[name] = std::move(tex);
    }

    template <typename Fn>
    void forEachMesh(Fn&& fn) {
        for (auto& kv : meshes) { fn(kv.first, kv.second); }
    }
    template <typename Fn>
    void forEachTexture(Fn&& fn) {
        for (auto& kv : textures) { fn(kv.first, kv.second); }
    }

    void clearMeshes() { meshes.clear(); }
    void clearTextures() { textures.clear(); }

private:
    std::string modelsDir = "assets/models/";
    std::string texturesDir = "assets/textures/";
    std::unordered_map<std::string, GpuMesh> meshes;
    std::unordered_map<std::string, GpuTexture> textures;
};
