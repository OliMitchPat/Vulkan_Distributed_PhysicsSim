#pragma once

#include "World.h"
#include "Renderer.h"
#include "RenderScene.h"
#include "Components.h"

class RenderSystem final
{
public:
    explicit RenderSystem(Renderer& rendererRef)
        : renderer(rendererRef)
    {
    }

    // Non-copyable / non-assignable / non-movable (reference member)
    RenderSystem(const RenderSystem&) = delete;
    RenderSystem& operator=(const RenderSystem&) = delete;
    RenderSystem(RenderSystem&&) = delete;
    RenderSystem& operator=(RenderSystem&&) = delete;

    // activeCameraRole: which logical camera to render from (Overview/Nav/Cactus)
    void render(World& world, CameraRole activeCameraRole);

private:
    Renderer& renderer;

    CameraRenderData buildCameraData(World& world, CameraRole activeCameraRole) const;
    void buildLights(World& world, RenderScene& scene) const;
    void buildSparkLights(World& world, RenderScene& scene) const;
    void buildInstances(World& world, RenderScene& scene) const;
    void buildParticles(World& world, RenderScene& scene) const;
};
