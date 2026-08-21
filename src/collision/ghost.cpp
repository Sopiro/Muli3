#include "muli3/triangle_shape.h"

#include "ghost.h"

namespace muli3
{

static Vec3 GetBarycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c)
{
    Vec3 v0 = b - a;
    Vec3 v1 = c - a;
    Vec3 v2 = p - a;

    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    if (Abs(denom) <= epsilon)
    {
        return Vec3{ 1.0f, 0.0f, 0.0f };
    }

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    return Vec3{ 1.0f - v - w, v, w };
}

Vec3 ResolveGhostNormal(
    uint8 activeEdges,
    const TriangleShape& triangle,
    const Transform& transform,
    const Vec3& point,
    const Vec3& normal,
    const Vec3& translation
)
{
    if (activeEdges == 0b111)
    {
        // Every edge is active
        return normal;
    }

    const Vec3* vertices = triangle.GetVertices();

    Vec3 a = Mul(transform, vertices[0]);
    Vec3 b = Mul(transform, vertices[1]);
    Vec3 c = Mul(transform, vertices[2]);

    Vec3 faceNormal = Cross(b - a, c - a);
    if (faceNormal.Normalize() == 0.0f)
    {
        return normal;
    }

    if (Dot(faceNormal, normal) < 0.0f)
    {
        faceNormal = -faceNormal;
    }

    // For casts, keep the original normal if it blocks the sweep less than the face normal.
    // This avoids replacing a grazing side hit with a stronger terrain-normal hit.
    if (Length2(translation) > epsilon && Dot(translation, normal) < Dot(translation, faceNormal))
    {
        return normal;
    }

    // Almost a face hit.
    if (Dot(faceNormal, normal) > 0.999848f) // cos 179
    {
        return normal;
    }

    constexpr float epsilon0 = 1.0e-4f;
    constexpr float epsilon1 = 1.0f - epsilon0;

    Vec3 bary = GetBarycentric(point, a, b, c);
    uint8 collidingEdge = 0;

    // Build edge bits for the feature nearest the collision point.
    if (bary.x > epsilon1)
    {
        // Collision is near vertex 0, so edge 0 or 2 needs to be tested.
        collidingEdge = 0b101;
    }
    else if (bary.y > epsilon1)
    {
        // Collision is near vertex 1, so edge 0 or 1 needs to be tested.
        collidingEdge = 0b011;
    }
    else if (bary.z > epsilon1)
    {
        // Collision is near vertex 2, so edge 1 or 2 needs to be tested.
        collidingEdge = 0b110;
    }
    else if (bary.x < epsilon0)
    {
        // Collision is near edge 1.
        collidingEdge = 0b010;
    }
    else if (bary.y < epsilon0)
    {
        // Collision is near edge 2.
        collidingEdge = 0b100;
    }
    else if (bary.z < epsilon0)
    {
        // Collision is near edge 0.
        collidingEdge = 0b001;
    }
    else
    {
        // Interior hit.
        return faceNormal;
    }

    // Keep the original normal if the feature includes an active edge, otherwise use the face normal.
    return (activeEdges & collidingEdge) != 0 ? normal : faceNormal;
}

} // namespace muli3