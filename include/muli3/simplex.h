#pragma once

#include "math.h"

namespace muli3
{

constexpr int32 max_simplex_vertex_count = 4;

struct Point
{
    Vec3 p;
    int32 id;
};

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
    bool HasSupportPoint(const Vec3& point);
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

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    MuliAssert(count < max_simplex_vertex_count);
    vertices[count++] = vertex;
}

inline bool Simplex::HasSupportPoint(const Vec3& point)
{
    for (int32 i = 0; i < count; ++i)
    {
        if (vertices[i].point == point)
        {
            return true;
        }
    }

    return false;
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