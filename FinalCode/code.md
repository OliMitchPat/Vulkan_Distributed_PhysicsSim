### Use this folder to develop your code

## Replica smoothing robustness tools

Replica updates for non-authoritative objects now use a per-object snapshot ring buffer (timestamp/tick, transform, linear/angular velocity), then sample the state at `now - interpolationDelay`.

The smoothing pipeline is:

1. **Interpolate** between buffered snapshots (position/velocity with lerp, rotation with slerp).
2. **Short extrapolation** when snapshots are missing (uses latest velocity, limited by max extrapolation window).
3. **Damped correction** when prediction error is large (spring-like blend instead of hard snapping).

### Runtime controls (ImGui -> `Concurrency` menu)

- **Replica Smoothing**
  - Enable/disable smoothing
  - Interpolation Delay (ms)
  - Max Extrapolation (ms)
  - Large Error Threshold (m)

- **Network Impairment (Local)** (applies to STATE_SNAPSHOT traffic on this peer)
  - Enable Snapshot Impairment
  - Latency (ms)
  - Latency Jitter (ms)
  - Packet Drop (%)
  - Preset buttons:
    - `Stable Network`
    - `100ms +/-50ms, 20% drop`

### How to run impairment tests

1. Start two or more peers as normal.
2. On each peer, open **Concurrency -> Network Impairment (Local)**.
3. Click **Preset: 100ms +/-50ms, 20% drop**.
4. Ensure **Replica Smoothing** is enabled and keep interpolation delay around `120-150 ms`.
5. Observe remote/non-authoritative objects:
   - motion should remain smooth and visually stable under loss/latency
   - missing snapshots should no longer cause immediate harsh snapping

## Distributed spawners (Milestone 7)

Spawner execution and distributed creation are now handled as follows:

1. **Spawner owner simulation**
   - Only the controlling peer simulates each spawner timer.
   - Controller peer is deterministic:
     - `ONE/TWO/THREE/FOUR` => that peer controls the spawner.
     - `SEQUENTIAL` => peer `ONE` controls the spawner.

2. **Reliable spawn replication**
   - Spawns are sent with reliable `SPAWN_OBJECT` messages.
   - Payload includes deterministic `ObjectId`, shape parameters, material, initial linear/angular velocity, transform, and owner.
   - Receiving peers instantiate the same object from the payload.

3. **Sequential ownership**
   - For `SEQUENTIAL` spawners, ownership rotates deterministically:
     - `ONE -> TWO -> THREE -> FOUR -> ONE ...`
   - Rotation is driven by the spawner owner's spawn counter.

### Integration notes

- The scene loader still creates authored objects first (deterministic IDs from scene order).
- Spawned objects are then created in deterministic event order and use the event `ObjectId`.
- Physics ownership mapping remains unchanged:
  - owner-local objects are dynamic and simulated locally
  - non-owned objects are kinematic replicas updated from network state

### Manual test steps

1. Start at least two peers with the same scene containing spawners (`sphereSpawners.bin`).
2. Confirm spawned objects appear on every peer.
3. Enable **View -> Colour by Owner** to verify rotating ownership colours over successive spawns.
4. Observe that owned objects move under their initial velocities and simulation.
5. Leave peers running for repeated spawns and verify the sequential ownership cycle continues correctly.
