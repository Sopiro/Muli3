#pragma once

#include "muli3/math.h"

namespace muli3
{

class TriangleShape;

// Jolt style ghost collision resolution: JPH::ActiveEdges::FixNormal.
Vec3 ResolveGhostNormal(
    uint8 activeEdgeBits,
    const TriangleShape& triangle,
    const Transform& transform,
    const Vec3& point,
    const Vec3& normal,
    const Vec3& translation
);

} // namespace muli3