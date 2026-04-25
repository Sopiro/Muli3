#pragma once

#include "shape.h"

namespace muli3
{

constexpr int32 max_contact_point_count = 4;

struct ContactPoint
{
    Vec3 p{ 0.0f, 0.0f, 0.0f };
    int32 id = 0;
};

struct ContactManifold
{
    ContactPoint contactPoints[max_contact_point_count];
    ContactPoint referencePoint;
    Vec3 contactNormal{ 1.0f, 0.0f, 0.0f };
    float penetrationDepth = 0.0f;
    int32 contactCount = 0;
    bool featureFlipped = false;
};

// clang-format off
typedef bool CollideFunction(const Shape*, const Transform&,
                             const Shape*, const Transform&,
                             ContactManifold*);

bool Collide(const Shape* a, const Transform& transformA,
             const Shape* b, const Transform& transformB,
             ContactManifold* manifold = nullptr);
// clang-format on

} // namespace muli3
