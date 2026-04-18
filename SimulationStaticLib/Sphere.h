#pragma once

struct SphereShape
{
    float radius = 0.5f;
    SphereShape() = default;
    explicit SphereShape(float r) : radius(r) {}
};