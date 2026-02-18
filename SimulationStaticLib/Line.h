#pragma once
#include "Vector3.h"

struct Line
{
    Vector3 Start{};
    Vector3 End{};

    Line() = default;
    Line(const Vector3& start, const Vector3& end) : Start(start), End(end) {}

    Vector3 Direction() const { return End - Start; }
    Vector3 PointAt(double t) const { return Start + (End - Start) * t; }
};
