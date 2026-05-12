#pragma once

#include "simplex.h"

/*
 *           \   A    /         ↑ <- Contact normal
 *            \      /          |
 *    ---------\----/-------------------------------  <- Reference face
 *              \  /
 *        B      \/  <- Incident point(Contact point)
 *
 *    A: Incident body
 *    B: Reference body
 */

namespace muli3
{

class Shape;

constexpr int32 max_contact_point_count = 4;

struct ContactManifold
{
    Point contactPoints[max_contact_point_count];
    Point referencePoint;
    Vec3 contactNormal;  // Contact normal is always pointing from reference body to incident body
    float penetrationDepth;
    int32 contactCount;
    bool featureFlipped; // Set to true if shape a is incident body
};

// clang-format off
typedef bool CollideFunction(const Shape*, const Transform&,
                             const Shape*, const Transform&,
                             ContactManifold*);

bool Collide(const Shape* a, const Transform& transformA,
             const Shape* b, const Transform& transformB,
             ContactManifold* manifold = nullptr);

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
