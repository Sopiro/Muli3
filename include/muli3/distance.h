#pragma once

#include "collision.h"

namespace muli3
{

// Closest features in world space.
struct ClosestFeatures
{
    Point featuresA[max_simplex_vertex_count - 1];
    Point featuresB[max_simplex_vertex_count - 1];
    int32 count;
};

// clang-format off
float GetClosestFeatures(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB,
    ClosestFeatures* features
);

float ComputeDistance(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB,
    Vec3* pointA, Vec3* pointB
);
// clang-format on

Vec3 ClosestPointVsSegment(const Vec3& p, const Vec3& a, const Vec3& b);
Vec3 ClosestPointVsTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c);
Vec3 ClosestPointVsTetrahedron(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);

// Returns s,t coordinate
Vec2 ClosestSegmentVsSegment(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1);

} // namespace muli3
