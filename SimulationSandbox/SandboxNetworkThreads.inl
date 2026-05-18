struct NetworkRuntime
{
    Net::NetworkingSystem net;
    std::mutex mutex;
};

static int ClampCoreForMachine(int desiredCore, int fallbackCore)
{
    const int numCores = ThreadUtils::LogicalCoreCount();
    if (numCores <= 0)
        return 0;
    if (desiredCore < numCores)
        return desiredCore;
    if (fallbackCore < numCores)
        return fallbackCore;
    return numCores - 1;
}

// ============================================================================
// Network receive thread. Pinned to logical processor 1 where available.
// ============================================================================
static void NetworkReceiveThreadFunc(
    SimSharedState& shared,
    const Net::PeerConfig& cfg,
    NetworkRuntime& runtime)
{
    using namespace Net;

    const int assignedRecvCore =
        ClampCoreForMachine(ThreadUtils::CORE_NET_0, ThreadUtils::CORE_RENDER);
    ThreadUtils::PinCurrentThread(ThreadUtils::CoreMask(assignedRecvCore));
    ThreadUtils::SetCurrentThreadName("NetRecv");
    shared.netRecvCoreAssigned = assignedRecvCore;
    shared.netRecvThreadRunning.store(true, std::memory_order_release);

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));
        const float dt = lc.beginFrame();
        shared.measuredNetworkRecvHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        std::vector<GlobalCommandPayload> commands;
        std::vector<std::pair<uint32_t, std::vector<StateSnapshotItem>>> snapshots;
        std::vector<SpawnObjectPayload> spawns;
        NetworkStats stats{};

        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.SetCurrentSceneGeneration(
                shared.sceneGeneration.load(std::memory_order_acquire));
            runtime.net.UpdateReceive(dt);

            GlobalCommandPayload cmd{};
            while (runtime.net.PopReceivedGlobalCommand(cmd))
                commands.push_back(cmd);

            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;
            while (runtime.net.PopReceivedStateSnapshot(items, tick))
            {
                snapshots.emplace_back(tick, std::move(items));
                items.clear();
            }

            SpawnObjectPayload payload{};
            while (runtime.net.PopReceivedSpawnObject(payload))
                spawns.push_back(payload);

            stats = runtime.net.GetStats();
            {
                std::lock_guard<std::mutex> lk(shared.activePeerIdsMutex);
                shared.activePeerIds = runtime.net.GetActivePeerIds();
            }
            {
                std::lock_guard<std::mutex> lk(shared.peerDebugMutex);
                shared.peerDebugInfo = runtime.net.GetPeerDebugInfo();
            }
        }

        {
            std::lock_guard<std::mutex> lk(shared.netStatsMutex);
            shared.netStats = stats;
        }

        for (const auto& cmd : commands)
        {
            const auto type = (GlobalCommandType)cmd.commandType;

            const ULONGLONG t = GetTickCount64();
            std::cout << "[T] NET recv popped GLOBAL cmd scene=" << cmd.sceneIndex
                << " generation=" << cmd.sceneGeneration
                << " t=" << t << "ms\n";

            if (type == GlobalCommandType::SceneChange)
            {
                shared.sceneGeneration.store(
                    cmd.sceneGeneration,
                    std::memory_order_release);

                shared.pendingSceneGeneration.store(
                    cmd.sceneGeneration,
                    std::memory_order_release);

                shared.pendingSceneIndex.store(
                    cmd.sceneIndex,
                    std::memory_order_relaxed);

                shared.pendingSceneChange.store(
                    true,
                    std::memory_order_release);

                shared.sceneTransitionActive.store(true, std::memory_order_release);
                shared.sceneTransitionRemainingSec.store(0.25f, std::memory_order_release);

                {
                    std::lock_guard<std::mutex> netLock(runtime.mutex);
                    runtime.net.SetCurrentSceneGeneration(cmd.sceneGeneration);
                    runtime.net.ClearSceneObjectTraffic();
                    runtime.net.PauseSnapshotTraffic(0.25f);
                }

                {
                    std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                    shared.outOwnedItems.clear();
                    shared.outTick = 0;
                    shared.outDirty = false;
                }

                {
                    std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                    shared.outSpawnEvents.clear();
                }

                {
                    std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                    shared.inReplicaHistory.clear();
                    shared.inReplicaLatestTick.clear();
                }

                {
                    std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                    shared.inSpawnEvents.clear();
                }
            }
            else if (type == GlobalCommandType::GravityOnOff)
            {
                shared.pendingGravityEnabled.store(
                    cmd.gravityEnabled != 0,
                    std::memory_order_relaxed);

                shared.pendingGravityChange.store(
                    true,
                    std::memory_order_release);
            }
        }

        for (auto& snapshot : snapshots)
        {
            const uint32_t tick = snapshot.first;
            const double nowSec = NowSeconds();

            std::lock_guard<std::mutex> lk(shared.inSnapMutex);
            for (const auto& it : snapshot.second)
            {
                auto latestIt = shared.inReplicaLatestTick.find(it.objectId);
                if (latestIt != shared.inReplicaLatestTick.end() && tick <= latestIt->second)
                {
                    shared.staleReplicaPacketsIgnored.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                shared.inReplicaLatestTick[it.objectId] = tick;

                ReplicaState st{};
                st.objectId = it.objectId;
                st.pos = glm::vec3(it.pos.x, it.pos.y, it.pos.z);
                st.rot = glm::quat(it.rot.w, it.rot.x, it.rot.y, it.rot.z);
                st.linVel = glm::vec3(it.linVel.x, it.linVel.y, it.linVel.z);
                st.angVel = glm::vec3(it.angVel.x, it.angVel.y, it.angVel.z);
                st.tick = tick;
                st.recvTimeSec = nowSec;

                auto& ring = shared.inReplicaHistory[it.objectId];
                ring.Push(st);
            }
        }

        if (!spawns.empty())
        {
            const uint32_t currentGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            std::vector<SpawnObjectPayload> validSpawns;
            for (const auto& payload : spawns)
            {
                if (payload.sceneGeneration == currentGeneration)
                    validSpawns.push_back(payload);
            }

            if (!validSpawns.empty())
            {
                std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                shared.inSpawnEvents.insert(
                    shared.inSpawnEvents.end(),
                    validSpawns.begin(),
                    validSpawns.end());
            }
        }

        lc.endFrame();
    }

    shared.netRecvThreadRunning.store(false, std::memory_order_release);
}

// ============================================================================
// Network send thread. Pinned to logical processor 2 where available.
// ============================================================================
static void NetworkSendThreadFunc(
    SimSharedState& shared,
    const Net::PeerConfig& cfg,
    NetworkRuntime& runtime)
{
    using namespace Net;

    const int assignedSendCore =
        ClampCoreForMachine(ThreadUtils::CORE_NET_1, ThreadUtils::CORE_NET_0);
    ThreadUtils::PinCurrentThread(ThreadUtils::CoreMask(assignedSendCore));
    ThreadUtils::SetCurrentThreadName("NetSend");
    shared.netSendCoreAssigned = assignedSendCore;
    shared.netSendThreadRunning.store(true, std::memory_order_release);

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    // Snapshot send throttling. 0 Hz means every fresh simulation tick.
    float snapshotSendAccum = 0.0f;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredNetworkSendHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);
        shared.measuredNetworkHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Keep networking layer aligned with the currently valid scene generation.
        Net::SnapshotImpairmentSettings impair{};
        impair.enabled = shared.netImpairmentEnabled.load(std::memory_order_relaxed);
        impair.latencyMs = shared.netLatencyMs.load(std::memory_order_relaxed);
        impair.jitterMs = shared.netJitterMs.load(std::memory_order_relaxed);
        impair.dropPercent = shared.netDropPercent.load(std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.SetCurrentSceneGeneration(
                shared.sceneGeneration.load(std::memory_order_acquire));
            runtime.net.SetSnapshotImpairment(impair);
        }

        // ------------------------------------------------------------
        // High-priority outgoing global commands.
        // These must happen before reliable resends, so they are not stuck
        // behind snapshot/resend processing.
        // ------------------------------------------------------------

        if (shared.sendSceneChange.exchange(false, std::memory_order_acq_rel))
        {
            const int idx =
                shared.sendSceneIndex.load(std::memory_order_relaxed);

            const uint32_t generation =
                shared.sendSceneGeneration.load(std::memory_order_acquire);

            {
                std::lock_guard<std::mutex> netLock(runtime.mutex);
                // Old scene object traffic is no longer valid.
                runtime.net.ClearSceneObjectTraffic();
                runtime.net.PauseSnapshotTraffic(0.25f);
                runtime.net.SetCurrentSceneGeneration(generation);
                runtime.net.SendSceneChange(idx, generation);
            }
        }

        if (shared.sendGravityChange.exchange(false, std::memory_order_acq_rel))
        {
            const bool enabled =
                shared.sendGravityEnabled.load(std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.PauseSnapshotTraffic(0.10f);
                runtime.net.SendGravityEnabled(enabled);
            }
        }

        // ------------------------------------------------------------
        // Process reliable resends / delayed outgoing snapshots.
        // ------------------------------------------------------------

        NetworkStats stats{};
        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.UpdateSend(dt);
            stats = runtime.net.GetStats();
            {
                std::lock_guard<std::mutex> lk(shared.activePeerIdsMutex);
                shared.activePeerIds = runtime.net.GetActivePeerIds();
            }
            {
                std::lock_guard<std::mutex> lk(shared.peerDebugMutex);
                shared.peerDebugInfo = runtime.net.GetPeerDebugInfo();
            }
        }

        {
            std::lock_guard<std::mutex> lk(shared.netStatsMutex);
            shared.netStats = stats;
        }

        // ------------------------------------------------------------
        // Send SPAWN_OBJECT packets.
        //
        // Important: do not send stale spawn events from an older scene
        // generation.
        // ------------------------------------------------------------

        if (shared.sceneTransitionActive.load(std::memory_order_acquire))
        {
            std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
            shared.outSpawnEvents.clear();
        }
        else
        {
            std::vector<SpawnObjectPayload> pending;

            {
                std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                if (!shared.outSpawnEvents.empty())
                    pending.swap(shared.outSpawnEvents);
            }

            const uint32_t currentGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            for (const auto& spawn : pending)
            {
                if (spawn.sceneGeneration != currentGeneration)
                    continue;

                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.SendSpawnObject(spawn);
            }
        }

        // ------------------------------------------------------------
        // Send STATE_SNAPSHOT packets.
        // ------------------------------------------------------------

        snapshotSendAccum += dt;
        const float snapshotHz = shared.snapshotSendHz.load(std::memory_order_relaxed);
        const float snapshotInterval = (snapshotHz > 0.0f) ? (1.0f / snapshotHz) : 0.0f;

        if (shared.sceneTransitionActive.load(std::memory_order_acquire))
        {
            // Drop any old outgoing snapshot data generated during transition.
            std::lock_guard<std::mutex> lk(shared.outSnapMutex);
            shared.outOwnedItems.clear();
            shared.outTick = 0;
            shared.outDirty = false;

            // Avoid immediately sending a snapshot burst when transition ends.
            snapshotSendAccum = 0.0f;
        }
        else if (snapshotHz <= 0.0f || snapshotSendAccum >= snapshotInterval)
        {
            if (snapshotInterval > 0.0f)
                snapshotSendAccum = std::max(0.0f, snapshotSendAccum - snapshotInterval);
            else
                snapshotSendAccum = 0.0f;

            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;
            bool dirty = false;

            {
                std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                dirty = shared.outDirty;

                if (dirty)
                {
                    items = shared.outOwnedItems;
                    tick = shared.outTick;
                    shared.outDirty = false;
                }
            }

            if (dirty && !items.empty())
            {
                const uint32_t generation =
                    shared.sceneGeneration.load(std::memory_order_acquire);

                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.SendStateSnapshot(
                    tick,
                    generation,
                    items.data(),
                    (uint32_t)items.size());
            }
        }

        lc.endFrame();
    }

    shared.netSendThreadRunning.store(false, std::memory_order_release);
}

