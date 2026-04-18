#pragma once

#include "EcsTypes.h"
#include <vector>
#include <algorithm> // for std::max

template<typename T>
class ComponentPool final {
public:
    explicit ComponentPool(std::size_t maxEntities = MAX_ENTITIES)
        : data(maxEntities), present(maxEntities, false) {
    }

    bool has(Entity e) const {
        return e < present.size() && present[e];
    }

    T& add(Entity e, const T& value) {
        ensureCapacity(e);
        data[e] = value;
        present[e] = true;
        return data[e];
    }

    void remove(Entity e) {
        if (e < present.size()) {
            present[e] = false;
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
    std::vector<T> data;
    std::vector<bool> present;

    void ensureCapacity(Entity e) {
        if (e >= data.size()) {
            const std::size_t newSize = std::max<std::size_t>(e + 1, data.size() * 2);
            data.resize(newSize);
            present.resize(newSize, false);
        }
    }

    template<typename Fn>
    static void callFn(void* fnPtr, Entity e, T& comp)
    {
        (*static_cast<Fn*>(fnPtr))(e, comp);
    }

    inline void forEachRaw(void (*cb)(void*, Entity, T&), void* fnPtr)
    {
        // Iterate through all slots; invoke callback for components that are present.
        for (Entity e = 0; e < static_cast<Entity>(present.size()); ++e)
        {
            if (present[e])
            {
                cb(fnPtr, e, data[e]);
            }
        }
    }
};
