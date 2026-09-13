#pragma once

#include "growable_stack.h"
#include "simplex.h"

namespace muli3
{

class Shape;

constexpr int32 max_contact_point_count = 4;

struct ContactPoint
{
    int32 id;

    Vec3 anchorA;
    Vec3 anchorB;

    float impulse;
};

struct Manifold
{
    int32 id;

    int32 contactCount;
    ContactPoint contactPoints[max_contact_point_count];
    Vec3 normal;

    Vec3 linearImpulse;
    float angularImpulse;
};

using ManifoldArray = GrowableStack<Manifold, 1>;
using CollideFunctionSimple = bool(const Shape*, const Transform&, const Shape*, const Transform&, Manifold*);
using CollideFunctionComplex = bool(const Shape*, const Transform&, const Shape*, const Transform&, ManifoldArray*);

// clang-format off
bool CollideSimple(
    const Shape* a, const Transform& transformA,
    const Shape* b, const Transform& transformB,
    Manifold* manifold = nullptr,
    bool* featureFlipped = nullptr
);

bool CollideComplex(
    const Shape* a, const Transform& transformA,
    const Shape* b, const Transform& transformB,
    ManifoldArray* manifolds = nullptr,
    bool* featureFlipped = nullptr
);

struct GJKResult
{
    Simplex simplex;
    Vec3 direction;
    float distance;
};

bool GJK(
    const Shape* a, const Transform& transformA,
    const Shape* b, const Transform& transformB,
    GJKResult* result
);

struct EPAResult
{
    Vec3 contactNormal;
    float penetrationDepth;
};

void EPA(
    const Shape* a, const Transform& transformA,
    const Shape* b, const Transform& transformB,
    const Simplex& simplex,
    EPAResult* result
);
// clang-format on

} // namespace muli3
