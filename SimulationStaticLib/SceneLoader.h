#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "Scene_generated.h"

namespace SimIO
{
    // ------------------------------------------------------------
    // SceneLoader
    //
    // Reads a FlatBuffers .bin file produced from Scene.fbs,
    // verifies its integrity, and provides a pointer to the root
    // Simulation::Scene object.
    //
    // The pointer returned by GetScene() is valid for the lifetime
    // of the SceneLoader instance (backed by m_bytes).
    // ------------------------------------------------------------
    class SceneLoader
    {
    public:
        SceneLoader() = default;

        // Attempt to load and verify a .bin file.
        // Returns true on success; on failure the loader is reset
        // and GetScene() returns nullptr.
        bool Load(const std::string& path)
        {
            m_bytes.clear();
            m_scene = nullptr;
            m_lastError.clear();

            std::ifstream file(path, std::ios::binary | std::ios::ate);
            if (!file.is_open())
            {
                m_lastError = "Cannot open file: " + path;
                return false;
            }

            const std::streamsize size = file.tellg();
            if (size <= 0)
            {
                m_lastError = "File is empty: " + path;
                return false;
            }

            file.seekg(0, std::ios::beg);
            m_bytes.resize(static_cast<size_t>(size));
            if (!file.read(reinterpret_cast<char*>(m_bytes.data()),
                           static_cast<std::streamsize>(m_bytes.size())))
            {
                m_lastError = "Read error: " + path;
                m_bytes.clear();
                return false;
            }

            flatbuffers::Verifier verifier(m_bytes.data(), m_bytes.size());
            if (!Simulation::VerifySceneBuffer(verifier))
            {
                m_lastError = "FlatBuffers verification failed: " + path;
                m_bytes.clear();
                return false;
            }

            m_scene = Simulation::GetScene(m_bytes.data());
            return m_scene != nullptr;
        }

        // Returns the verified scene root, or nullptr if not loaded.
        const Simulation::Scene* GetScene() const { return m_scene; }

        bool IsLoaded() const { return m_scene != nullptr; }

        const std::string& LastError() const { return m_lastError; }

    private:
        std::vector<uint8_t>     m_bytes;
        const Simulation::Scene* m_scene = nullptr;
        std::string              m_lastError;
    };

};
