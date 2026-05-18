#pragma once

#include "EcsTypes.h"
#include "ComponentPool.h"

#include <vector>
#include <bitset>
#include <unordered_map>
#include <typeindex>
#include <memory>
#include <cassert>
#include <utility>   
// Optional: track which entities are alive
class World final{
public:
    World();

    // --- Entity management ---

    Entity createEntity();
    Entity createEntityWithId(Entity e);
    void destroyEntity(Entity e);
    bool isAlive(Entity e) const;

    // --- Component management ---

    template<typename T>
    T& addComponent(Entity e, const T& component);

    template<typename T>
    bool hasComponent(Entity e) const;

    template<typename T>
    T* getComponent(Entity e);

    template<typename T>
    const T* getComponent(Entity e) const;

    template<typename T>
    void removeComponent(Entity e);

    // --- Iteration helpers ---

    // For systems that need entities with a *single* component type
    template<typename T, typename Fn>
    void forEach(Fn fn);

    template<typename A, typename B, typename Fn>
    void forEach(Fn fn);
    // Later we can add multi-component iteration if needed.
    void Clear();

    float* ClearColour() { return clearColour; }
    const float* ClearColour() const { return clearColour; }

private:
    // Entity lifecycle
    std::vector<bool> alive;
    std::vector<Entity> freeList;
    Entity nextEntity;

    // Component storage per type
    std::unordered_map<std::type_index, std::unique_ptr<void, void(*)(void*)>> pools;

    template<typename T>
    ComponentPool<T>& getPool();

    template<typename T>
    const ComponentPool<T>& getPool() const;

    template<typename T>
    ComponentPool<T>* findPool();

    template<typename T>
    const ComponentPool<T>* findPool() const;

    template<typename T>
    static void deletePool(void* ptr) {
        delete static_cast<ComponentPool<T>*>(ptr);
    }

    float clearColour[4] = { 0.1f, 0.1f, 0.1f, 1.0f };

    void ensureEntityCapacity(Entity e);
};

// --------- Inline/template implementation ----------

inline World::World()
    : alive(MAX_ENTITIES, false),
    nextEntity(0) {
}

inline void World::ensureEntityCapacity(Entity e) {
    if (e >= alive.size()) {
        const std::size_t newSize = std::max<std::size_t>(e + 1, alive.size() * 2);
        alive.resize(newSize, false);
    }
}

inline Entity World::createEntity() {
    Entity e;
    if (!freeList.empty()) {
        e = freeList.back();
        freeList.pop_back();
    }
    else {
        e = nextEntity++;
    }

    ensureEntityCapacity(e);
    alive[e] = true;
    return e;
}

inline Entity World::createEntityWithId(Entity e) {
    ensureEntityCapacity(e);

    if (alive[e])
        return e;

    auto it = std::find(freeList.begin(), freeList.end(), e);
    if (it != freeList.end())
        freeList.erase(it);

    alive[e] = true;
    if (e >= nextEntity)
        nextEntity = e + 1;

    return e;
}

inline void World::destroyEntity(Entity e) {
    if (e >= alive.size() || !alive[e]) return;
    alive[e] = false;
    freeList.push_back(e);

    // NOTE: we are *not* eagerly removing components here for simplicity.
    // Systems should check isAlive(e) if needed.
}

inline bool World::isAlive(Entity e) const {
    return e < alive.size() && alive[e];
}

inline void World::Clear()
{
    std::fill(alive.begin(), alive.end(), false);
    freeList.clear();
    pools.clear();
    nextEntity = 0;
}


template<typename T>
inline ComponentPool<T>& World::getPool()
{
    auto key = std::type_index(typeid(T));
    auto it = pools.find(key);

    if (it == pools.end())
    {
        auto* newPool = new ComponentPool<T>(MAX_ENTITIES);

        auto inserted = pools.emplace(
            key,
            std::unique_ptr<void, void(*)(void*)>(newPool, &World::deletePool<T>)
        );

        // Return through the container storage, not via the local pointer variable
        return *static_cast<ComponentPool<T>*>(inserted.first->second.get());
    }

    return *static_cast<ComponentPool<T>*>(it->second.get());
}

template<typename T>
inline const ComponentPool<T>& World::getPool() const {
    auto it = pools.find(std::type_index(typeid(T)));
    assert(it != pools.end() && "Component pool not found");
    return *static_cast<const ComponentPool<T>*>(it->second.get());
}

template<typename T>
inline ComponentPool<T>* World::findPool() {
    auto it = pools.find(std::type_index(typeid(T)));
    if (it == pools.end()) return nullptr;
    return static_cast<ComponentPool<T>*>(it->second.get());
}

template<typename T>
inline const ComponentPool<T>* World::findPool() const {
    auto it = pools.find(std::type_index(typeid(T)));
    if (it == pools.end()) return nullptr;
    return static_cast<const ComponentPool<T>*>(it->second.get());
}

template<typename T>
inline T& World::addComponent(Entity e, const T& component) {
    assert(isAlive(e) && "Cannot add component to dead entity");
    auto& pool = getPool<T>();
    return pool.add(e, component);
}

template<typename T>
inline bool World::hasComponent(Entity e) const {
    if (!isAlive(e)) return false;
    auto* pool = findPool<T>();
    return pool && pool->has(e);
}

template<typename T>
inline T* World::getComponent(Entity e) {
    if (!isAlive(e)) return nullptr;
    auto* pool = findPool<T>();
    return pool ? pool->get(e) : nullptr;
}

template<typename T>
inline const T* World::getComponent(Entity e) const {
    if (!isAlive(e)) return nullptr;
    auto* pool = findPool<T>();
    return pool ? pool->get(e) : nullptr;
}

template<typename T>
inline void World::removeComponent(Entity e) {
    auto* pool = findPool<T>();
    if (pool) {
        pool->remove(e);
    }
}

template<typename T, typename Fn>
inline void World::forEach(Fn fn) {
    auto* pool = findPool<T>();
    if (!pool) return;

    pool->forEach([this, &fn](Entity e, T& comp) {
        if (isAlive(e)) {
            fn(e, comp);
        }
        });
}

template<typename A, typename B, typename Fn>
inline void World::forEach(Fn fn)
{
    auto* poolA = findPool<A>();
    auto* poolB = findPool<B>();
    if (!poolA || !poolB) return;

    poolA->forEach([this, &fn, poolB](Entity e, A& a)
        {
            if (!isAlive(e)) return;

            // entity must also have B
            if (auto* b = poolB->get(e))
            {
                fn(e, a, *b);
            }
        });
}
