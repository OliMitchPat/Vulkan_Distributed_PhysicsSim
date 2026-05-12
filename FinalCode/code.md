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
