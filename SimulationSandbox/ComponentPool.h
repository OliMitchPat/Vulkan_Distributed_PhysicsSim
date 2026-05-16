#pragma once

#include "EcsTypes.h"
#include <vector>
#include <algorithm> // for std::max

template<typename T>
class ComponentPool final {
public:
    explicit ComponentPool(std::size_t maxEntities = MAX_ENTITIES)
        : data(maxEntities),
          present(maxEntities, false),
          activeIndex(maxEntities, INVALID_INDEX) {
    }

    bool has(Entity e) const {
        return e < present.size() && present[e];
    }

    T& add(Entity e, const T& value) {
        ensureCapacity(e);
        if (!present[e]) {
            activeIndex[e] = activeEntities.size();
            activeEntities.push_back(e);
            present[e] = true;
        }
        data[e] = value;
        return data[e];
    }

    void remove(Entity e) {
        if (e < present.size() && present[e]) {
            const std::size_t removedIndex = activeIndex[e];
            const Entity lastEntity = activeEntities.back();

            activeEntities[removedIndex] = lastEntity;
            activeIndex[lastEntity] = removedIndex;
            activeEntities.pop_back();

            present[e] = false;
            activeIndex[e] = INVALID_INDEX;
        }
    }

    T* get(Entity e) {
        if (!has(e)) return nullptr;
        return &data[e];
    }

    const T* get(Entity e) const {
        if (!has(e)) return nullptr;
        return &data[e];
    }

    template<typename Fn>
    void forEach(Fn fn)
    {
        forEachRaw(&callFn<Fn>, &fn);
    }

private:
    static constexpr std::size_t INVALID_INDEX = static_cast<std::size_t>(-1);

    std::vector<T> data;
    std::vector<bool> present;
    std::vector<Entity> activeEntities;
    std::vector<std::size_t> activeIndex;

    void ensureCapacity(Entity e) {
        if (e >= data.size()) {
            const std::size_t newSize = std::max<std::size_t>(e + 1, data.size() * 2);
            data.resize(newSize);
            present.resize(newSize, false);
            activeIndex.resize(newSize, INVALID_INDEX);
        }
    }

    template<typename Fn>
    static void callFn(void* fnPtr, Entity e, T& comp)
    {
        (*static_cast<Fn*>(fnPtr))(e, comp);
    }

    inline void forEachRaw(void (*cb)(void*, Entity, T&), void* fnPtr)
    {
        // Iterate only dense component entries; most scenes use far fewer entities
        // than the pool capacity, and high-Hz systems call this heavily.
        for (Entity e : activeEntities)
        {
            if (e < present.size() && present[e])
            {
                cb(fnPtr, e, data[e]);
            }
        }
    }
};
