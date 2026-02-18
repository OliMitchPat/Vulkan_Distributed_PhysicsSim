#pragma once
#include "Vector3.h"
#include "Line.h"

class Collider
{
public:
    explicit Collider(const Vector3& position = {}) : _Position(position) {}
    virtual ~Collider() = default;

    const Vector3& GetPosition() const { return _Position; }
    void SetPosition(const Vector3& p) { _Position = p; }

    virtual bool IsInside(const Vector3& point) const = 0;
    virtual bool Intersects(const Line& line) const = 0;

protected:
    Vector3 _Position{};
};
