#pragma once

struct Line
{
    glm::vec3 Start{};
    glm::vec3 End{};

    Line() = default;
    Line(const glm::vec3& start, const glm::vec3& end) : Start(start), End(end) {}

    glm::vec3 Direction() const { return End - Start; }
    glm::vec3 PointAt(float t) const { return Start + (End - Start) * t; }
};