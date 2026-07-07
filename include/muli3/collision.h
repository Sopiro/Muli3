#pragma once

#include "growable_stack.h"
#include "simplex.h"

namespace muli3
{

class Shape;

constexpr int32 max_contact_point_count = 4;

struct ContactPoint
{
    Vec3 anchorA;
    Vec3 anchorB;
    int32 id;

    float impulse;
};

struct ContactManifold
{
    int32 contactCount;
    ContactPoint contactPoints[max_contact_point_count];
    Vec3 normal;

    Vec3 linearImpulse;
    float angularImpulse;
};

using ManifoldSet = GrowableStack<ContactManifold, 1>;
using CollideFunction = bool(const Shape*, const Transform&, const Shape*, const Transform&, ContactManifold*);
using CollideFunction2 = bool(const Shape*, const Transform&, const Shape*, const Transform&, ManifoldSet*);

// clang-format off
bool Collide(const Shape* a, const Transform& transformA,
             const Shape* b, const Transform& transformB,
             ContactManifold* manifold = nullptr,
             bool* featureFlipped = nullptr);

struct GJKResult
{
    Simplex simplex;
    Vec3 direction;
    float distance;
};

bool GJK(const Shape* a, const Transform& transformA,
         const Shape* b, const Transform& transformB,
         GJKResult* result);

struct EPAResult
{
    Vec3 contactNormal;
    float penetrationDepth;
};

void EPA(const Shape* a, const Transform& transformA,
         const Shape* b, const Transform& transformB,
         const Simplex& simplex,
         EPAResult* result);
// clang-format on

} // namespace muli3
