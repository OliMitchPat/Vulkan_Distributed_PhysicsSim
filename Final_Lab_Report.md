# Final Lab Report: Simulation & Concurrency

**By:** Oliver Mitchell-Paterson  
**Word Count:** 1589

## Contents

- [1. System Architecture, Threads and Networking](#1-system-architecture-threads-and-networking)
  - [1.1 Project Overview](#11-project-overview)
  - [1.2 Networking Architecture](#12-networking-architecture)
  - [1.3 Distributed Ownership and State Synchronisation](#13-distributed-ownership-and-state-synchronisation)
  - [1.4 Concurrency Architecture](#14-concurrency-architecture)
  - [1.5 Thread Frequencies and Processor Affinity](#15-thread-frequencies-and-processor-affinity)
  - [1.6 GPU Compute and Advanced Systems](#16-gpu-compute-and-advanced-systems)
  - [1.7 Research for Flocking and Spatial Partitioning](#17-research-for-flocking-and-spatial-partitioning)
  - [1.8 Architecture Reflection](#18-architecture-reflection)
- [2. Motion Physics and Collision Detection/Response](#2-motion-physics-and-collision-detectionresponse)
  - [2.1 Physics Object Representation](#21-physics-object-representation)
  - [2.2 Materials, Mass and Inertia](#22-materials-mass-and-inertia)
  - [2.3 Motion Integration](#23-motion-integration)
  - [2.4 Collision Detection Pipeline](#24-collision-detection-pipeline)
  - [2.5 Shape-Specific Collision Detection](#25-shape-specific-collision-detection)
  - [2.6 Collision Response](#26-collision-response)
  - [2.7 Animated Objects, Containers and Spawners](#27-animated-objects-containers-and-spawners)
  - [2.8 Physics Reflection](#28-physics-reflection)
- [References](#references)

---

## 1. System Architecture, Threads and Networking

### 1.1 Project Overview

The project is split between the `SimulationSandbox` executable and the `SimulationStaticLib` library. `SimulationSandbox` owns Vulkan rendering, ImGui, networking setup, scene selection and the main runtime threads. `SimulationStaticLib` contains the reusable ECS components, physics, collision, flocking and FlatBuffer loading code.

Scenes are loaded from FlatBuffer `.bin` files and converted into ECS entities for objects, cameras, materials, spawners, ownership and flocking data. Missing values are given defaults during loading so scenes remain usable even when optional fields are absent.

The world uses an ECS-style structure. Entities combine transform, velocity, physics, shape, owner, animated, spawner and flocking components, allowing rendering, networking and simulation to share object identity while each system reads only the data it needs.

This separation keeps application runtime code, physics code and distributed-state logic modular.

### 1.2 Networking Architecture

Networking uses UDP in a peer-to-peer model with separate send and receive threads. The send thread transmits local snapshots, spawns and global commands, while the receive thread decodes incoming packets and queues them for simulation-safe application.

Local UI actions, such as camera choice and material/owner colour display, remain on the current peer. Global actions, such as scene changes, are sent to all peers so each machine loads the same scene.

Network impairment controls test the required 100ms +/- 50ms latency and 20% packet loss case.

Incoming snapshots use sequence/tick checks so duplicate, stale or out-of-order updates are ignored. Remote object transforms are buffered and smoothed rather than applied directly, reducing visible jitter when packets arrive late.

### 1.3 Distributed Ownership and State Synchronisation

Each simulated object has an `OwnerComponent` identifying the peer responsible for its physics update and collision response. Other peers render the received replica state instead of simulating the same object authoritatively.

This avoids conflicting physics updates and distributes work across peers. Spawners can assign ownership to a fixed peer or rotate it sequentially, allowing multi-peer scenes to share simulation load.

State synchronisation is snapshot based: owners publish compact object state and receivers interpolate or correct their local replicas.

### 1.4 Concurrency Architecture

The runtime is split into render, simulation, network send and network receive threads. The simulation thread owns the live ECS `World`, while rendering reads a published `WorldSnapshot` so UI and graphics do not lock the simulation hot path.

Physics integration can use worker threads that process dynamic bodies in chunks before collision handling. Networking uses queues and shared buffers so packet processing does not block simulation.

This architecture keeps the major systems asynchronous, matching the coursework requirement for independently controlled graphics, networking and simulation rates.

### 1.5 Thread Frequencies and Processor Affinity

Render, network and simulation rates are independently configurable through ImGui. Physics uses a fixed timestep, so changing the simulation target changes the step size rather than the apparent speed of the world.

Processor affinity pins render to logical processor 0, networking to logical processors 1-2 and simulation to logical processor 3+. The UI displays target/measured rates and thread-core assignments for demonstration.

On smaller machines the code falls back safely if fewer logical processors are available.

### 1.6 GPU Compute and Advanced Systems

The extended concurrency work includes four-peer support, network smoothing and a Vulkan compute path for flocking. The GPU mode updates boid positions and velocities in a compute shader while the CPU modes remain available for comparison.

This keeps the rigid-body solver CPU-based, where ownership, contacts, friction and networking are easier to synchronise, while still demonstrating GPU simulation work.

Flocking also includes brute force, uniform grid and octree neighbour-search modes for extended simulation comparison.

### 1.7 Research for Flocking and Spatial Partitioning

The advanced flocking behaviour was based on Reynolds' boids model, where flocking emerges from local collision avoidance, velocity matching and flock centring (Reynolds, 1987). These map to my separation/obstacle avoidance, alignment and cohesion behaviours. Using nearby neighbours rather than one global centre allows the flock to split around obstacles and reform naturally.

The steering-force combination was based on Buckland's game AI steering methods. My system uses a weighted truncated sum, so avoidance and separation take priority before alignment and cohesion use the remaining steering budget (Buckland, 2005).

The extended flocking work treats neighbour search as the bottleneck. Brute force checks every boid against every other boid, giving O(n squared) behaviour. The uniform grid mode follows spatial hashing ideas by inserting boids into cells and searching only nearby cells (Teschner et al., 2003).

The octree mode uses hierarchical spatial subdivision: 3D space is recursively split into eight regions so empty or distant regions can be skipped (Meagher, 1982; Ericson, 2005). The UI compares brute force, grid and octree by boid count, checks, update time and memory estimate.

The flocking results meet the specification by comparing brute force against two spatial segmentation techniques: uniform grid and octree. Brute force performs all neighbour checks, so its cost increases rapidly as the boid count grows. The uniform grid and octree reduce the number of spatial candidates used for neighbour/collision avoidance checks by only searching nearby regions, improving the CPU update time at larger boid counts. The octree gives the lowest candidate count at 500 and 1000 boids, while the uniform grid has lower build overhead and slightly lower memory use. The GPU brute-force version was slower in these tests because boid data was passed between the CPU and GPU, so transfer and synchronisation overhead outweighed the benefit of parallel execution at these sample sizes.

### 1.8 Architecture Reflection

A strength of the architecture is clear ownership of mutable state: simulation owns the live ECS world, rendering uses snapshots, and networking passes data through queues or buffers.

Distributed ownership is also effective because each peer simulates only its owned objects while still displaying a shared world.

The main weakness is added synchronisation complexity. Remote objects need ownership checks, buffering and smoothing, and the network code must reject stale or duplicate data.

Overall, the architecture suits the coursework because it keeps systems responsive and demonstrates asynchronous execution.

If repeated, I would improve networking with interest management and priority-based updates so nearby or fast-moving objects are synchronised more often than low-priority objects.

---

## 2. Motion Physics and Collision Detection/Response

### 2.1 Physics Object Representation

Physics objects are represented with ECS components rather than inheritance. Entities store only the data required for their behaviour, such as transform, shape, material, velocity, ownership, animation, spawner or physics components.

This lets rendering, physics and networking share the same world representation while remaining decoupled. For example, rendering reads transforms and shapes, while physics reads rigid-body data and writes updated transforms.

`RigidBody` stores mass, inverse mass, inertia, forces, torque, linear velocity and angular velocity, keeping reusable physical behaviour inside the physics system.

This separates object identity from physical behaviour.

### 2.2 Materials, Mass and Inertia

Materials store density, restitution and friction. Density and shape volume produce mass, while material-pair interactions control bounciness and static/dynamic friction. This means two objects with the same shape can behave differently if their materials have different densities or interaction settings.

Dynamic bodies use inverse mass during integration and impulses, while static bodies have zero inverse mass and cannot be moved by collisions.

Inertia is calculated so off-centre contacts can change angular velocity as well as linear velocity. Shape-dependent inertia gives spheres, cuboids, cylinders and capsules different rotational behaviour instead of treating every object as a simple particle.

This is more realistic than treating objects as point masses.

### 2.3 Motion Integration

Dynamic bodies are integrated during the physics update. Forces produce acceleration, velocity is updated, and velocity then updates position and orientation.

The default integrator is semi-implicit Euler because it is simple, fast and more stable for real-time simulation than explicit Euler.

Static bodies are skipped, while kinematic bodies such as animated platforms are moved directly from their animation paths and can still affect dynamic bodies through collision response.

### 2.4 Collision Detection Pipeline

Collision detection runs after integration. The broad phase first rejects distant pairs using object bounds and a spatial grid, reducing the number of expensive narrow-phase tests. The system also avoids unnecessary static-static work and records profiling values so heavy scenes can be diagnosed.

Candidate pairs then enter shape-specific narrow-phase tests. The system records broad-phase rejects, narrow-phase pairs and contacts for profiling.

When a collision is found, the narrow phase outputs the contact normal, penetration depth and contact point for the solver.

This staged pipeline improves scalability as object counts increase.

### 2.5 Shape-Specific Collision Detection

Narrow-phase routines are selected by `ShapeType` and support spheres, cuboids, cylinders, capsules and planes. This includes sphere-object cases for common scenes and non-spherical moving-object tests so cuboids, capsules and cylinders can participate in dynamic collision detection.

Simple cases use direct geometric tests, such as centre distance for spheres or signed distance for planes. More complex pairs use closest-point and axis-based tests, while containers invert the collision interpretation so objects are kept inside the boundary volume.

Each successful test produces a contact manifold, keeping detection separate from response.

### 2.6 Collision Response

Collision response uses contact normal, penetration depth and contact point. The solver first applies positional correction to separate overlapping bodies, then calculates impulses using relative velocity, restitution and inverse mass. This means heavier objects are affected less than lighter ones during the same collision.

Impulses affect both linear and angular velocity. Angular response uses the contact offset from the centre of mass and the inverse inertia tensor, so off-centre impacts cause rotation.

Friction is applied with tangent impulses. Static friction resists initial sliding, while dynamic friction reduces sliding motion once it begins.

This accounts for mass, bounciness, friction and rotation.

### 2.7 Animated Objects, Containers and Spawners

Animated objects are kinematic bodies driven by waypoint paths rather than forces. Dynamic objects collide with them as moving surfaces, transferring motion from the platform into the simulated object, but the animated object's authored path is not changed.

Containers are inverted collision volumes that keep objects inside shapes such as boxes or cylinders.

Spawners create entities during simulation with transform, shape, physics and ownership components. Networked spawns use deterministic IDs so all peers create matching objects, and sequential ownership rotates new simulated objects between peers for distributed scenes.

Fixed or sequential spawner ownership lets spawned objects be distributed across peers.

### 2.8 Physics Reflection

A strength of the physics system is its clear sequence: integration, broad phase, narrow phase, impulse response and positional correction. This made it easier to optimise and debug because timings, candidate pairs and contacts can be displayed separately in ImGui.

The response model is also strong because it uses mass, restitution, friction and inertia instead of only moving objects apart.

The main weakness is that the solver is still approximate, so complex stacks or many simultaneous contacts can become unstable at high speed or high density.

If repeated, I would add continuous collision detection and a stronger iterative constraint solver for difficult stacking cases.

---

## References

Buckland, M. (2005) *Programming Game AI by Example*. Plano, TX: Wordware Publishing.

Ericson, C. (2005) *Real-Time Collision Detection*. San Francisco, CA: Morgan Kaufmann.

Meagher, D. (1982) ‘Geometric modeling using octree encoding’, *Computer Graphics and Image Processing*, 19(2), pp. 129–147.

Reynolds, C.W. (1987) ‘Flocks, herds and schools: A distributed behavioral model’, *Computer Graphics*, 21(4), pp. 25–34.

Teschner, M., Heidelberger, B., Müller, M., Pomeranets, D. and Gross, M. (2003) ‘Optimized spatial hashing for collision detection of deformable objects’, *Proceedings of Vision, Modeling, Visualization 2003*, pp. 47–54.
