#pragma once
/*
 * WorldSnapshot.h — A lightweight, renderer-ready snapshot of the simulation
 *                   world, produced by the simulation thread each tick and
 *                   consumed by the render thread.
 *
 * The camera is NOT part of the snapshot; the render / UI thread maintains
 * its own camera state independently of the ECS World.
 *
 * Publishing and consuming the snapshot is done through a mutex-protected
 * shared_ptr pair so the render thread never touches the live sim World.
 */

#include "RenderScene.h"
#include <memory>
#include <mutex>
#include <cstdint>

// ---- WorldSnapshot ----------------------------------------------------------
// Everything the renderer needs except camera data (which the render thread
// controls independently).
struct WorldSnapshot
{
    std::vector<RenderInstance>            instances;
    std::vector<ParticleRenderData>        particles;
    std::vector<DirectionalLightRenderData> directionalLights;
    std::vector<SparkLightRenderData>      sparkLights;
    glm::vec3  ambientLight{ 0.02f, 0.02f, 0.02f };
    glm::vec4  clearColor  { 0.1f,  0.1f,  0.1f,  1.0f };
    uint64_t   simTickNumber = 0;   // debugging aid
};

// ---- SnapshotBuffer ---------------------------------------------------------
// Double-pointer publish/consume buffer.  The sim thread calls publish() every
// tick; the render thread calls consume() every frame.  The lock is only held
// during a pointer swap — no copying under the lock.
class SnapshotBuffer
{
public:
    void publish(std::shared_ptr<WorldSnapshot> snap)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_snapshot = std::move(snap);
    }

    std::shared_ptr<WorldSnapshot> consume()
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        return m_snapshot;   // shared_ptr copy: cheap, ref-counted
    }

private:
    std::mutex                     m_mutex;
    std::shared_ptr<WorldSnapshot> m_snapshot;
};
