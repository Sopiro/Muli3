#pragma once

#include "shape.h"

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

constexpr int32 max_contact_point_count = 4;
constexpr int32 max_simplex_vertex_count = 4;

struct SupportPoint
{
    Point pointA;
    Point pointB;
    Vec3 point; // pointA - pointB
    float weight;
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

private:
    void SolveSegment(const Vec3& q);
    void SolveTriangle(const Vec3& q);
    void SolveTetrahedron(const Vec3& q);
};

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

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    MuliAssert(count < max_simplex_vertex_count);
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
