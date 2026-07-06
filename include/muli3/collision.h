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

    Vec3 impulse;
    float angularImpulse;
};

// clang-format off
typedef bool CollideFunction(const Shape*, const Transform&,
                             const Shape*, const Transform&,
                             ContactManifold*);
typedef bool CollideFunction2(const Shape*, const Transform&,
                              const Shape*, const Transform&,
                              GrowableStack<ContactManifold, 1>*);

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
