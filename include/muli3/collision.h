#pragma once

#include "shape.h"

namespace muli3
{

constexpr int32 max_contact_point_count = 4;
constexpr int32 max_simplex_vertex_count = 4;

using ContactPoint = Point;

struct SupportPoint
{
    Point pointA;
    Point pointB;
    Vec3 point{ 0.0f, 0.0f, 0.0f };
    float weight = 0.0f;
};

struct Simplex
{
    Simplex() = default;

    void AddVertex(const SupportPoint& vertex);
    void Save(Vec3* saveVertices, int32* saveCount);
    void Advance(const Vec3& q);
    Vec3 GetSearchDirection() const;
    Vec3 GetClosestPoint() const;
    void GetWitnessPoint(Vec3* pointA, Vec3* pointB) const;

    int32 count = 0;
    SupportPoint vertices[max_simplex_vertex_count];
    float divisor = 1.0f;
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

struct GJKResult
{
    Simplex simplex;
    Vec3 direction{ 1.0f, 0.0f, 0.0f };
    float distance = 0.0f;
};

bool GJK(const Shape* a, const Transform& transformA,
         const Shape* b, const Transform& transformB,
         GJKResult* result);

struct EPAResult
{
    Vec3 contactNormal{ 1.0f, 0.0f, 0.0f };
    float penetrationDepth = 0.0f;
};

void EPA(const Shape* a, const Transform& transformA,
         const Shape* b, const Transform& transformB,
         const Simplex& simplex,
         EPAResult* result);
// clang-format on

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    MuliAssert(count != max_simplex_vertex_count);

    vertices[count++] = vertex;
}

inline void Simplex::Save(Vec3* saveVertices, int32* saveCount)
{
    *saveCount = count;
    for (int32 i = 0; i < count; ++i)
    {
        saveVertices[i] = vertices[i].point;
    }
}

} // namespace muli3
