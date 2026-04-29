#include "muli3/box.h"

namespace muli3
{

constexpr static int32 boxFaceVertexIndices[6][4] = {
    { 0, 4, 6, 2 }, // -x
    { 1, 3, 7, 5 }, // +x
    { 0, 1, 5, 4 }, // -y
    { 2, 6, 7, 3 }, // +y
    { 0, 2, 3, 1 }, // -z
    { 4, 5, 7, 6 }, // +z
};

constexpr static Vec3 boxNormals[6] = {
    Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.0f, 0.0f },  Vec3{ 0.0f, -1.0f, 0.0f },
    Vec3{ 0.0f, 1.0f, 0.0f },  Vec3{ 0.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 0.0f, 1.0f },
};

Box::Box(float width, float height, float depth, float inRadius, const Transform& transform)
    : Shape{ Shape::box, inRadius }
    , halfExtents{
        width * 0.5f * Abs(transform.s.x),
        height * 0.5f * Abs(transform.s.y),
        depth * 0.5f * Abs(transform.s.z),
    }
{
    center = transform.p;

    Vec3 fullExtents = halfExtents + Vec3{ radius, radius, radius };
    volume = 8.0f * fullExtents.x * fullExtents.y * fullExtents.z;
}

Box::Box(const Box& other, const Transform& transform)
    : Shape{ Shape::box, other.radius }
    , halfExtents{
        other.halfExtents.x * Abs(transform.s.x),
        other.halfExtents.y * Abs(transform.s.y),
        other.halfExtents.z * Abs(transform.s.z),
    }
{
    center = Mul(transform, other.center);

    Vec3 fullExtents = halfExtents + Vec3{ radius, radius, radius };
    volume = 8.0f * fullExtents.x * fullExtents.y * fullExtents.z;
}

void Box::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;

    Vec3 size = (halfExtents + Vec3{ radius, radius, radius }) * 2.0f;
    float x2 = size.x * size.x;
    float y2 = size.y * size.y;
    float z2 = size.z * size.z;
    float s = outMassData->mass / 12.0f;

    Mat3 inertiaCenter{
        Vec3{ s * (y2 + z2), 0.0f, 0.0f },
        Vec3{ 0.0f, s * (x2 + z2), 0.0f },
        Vec3{ 0.0f, 0.0f, s * (x2 + y2) },
    };

    float x = center.x;
    float y = center.y;
    float z = center.z;
    float m = outMassData->mass;

    outMassData->inertia = Mat3(
        inertiaCenter.ex + Vec3{ m * (y * y + z * z), -m * x * y, -m * x * z },
        inertiaCenter.ey + Vec3{ -m * y * x, m * (x * x + z * z), -m * y * z },
        inertiaCenter.ez + Vec3{ -m * z * x, -m * z * y, m * (x * x + y * y) }
    );
}

void Box::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Mat3 worldRotation{ transform.q };
    Vec3 worldCenter = Mul(transform, center);

    Vec3 e{
        Abs(worldRotation.ex.x) * halfExtents.x + Abs(worldRotation.ey.x) * halfExtents.y +
            Abs(worldRotation.ez.x) * halfExtents.z,
        Abs(worldRotation.ex.y) * halfExtents.x + Abs(worldRotation.ey.y) * halfExtents.y +
            Abs(worldRotation.ez.y) * halfExtents.z,
        Abs(worldRotation.ex.z) * halfExtents.x + Abs(worldRotation.ey.z) * halfExtents.y +
            Abs(worldRotation.ez.z) * halfExtents.z,
    };

    Vec3 r{ radius, radius, radius };
    *outAABB = AABB{ worldCenter - e - r, worldCenter + e + r };
}

Face Box::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Vec3 localDir = transform.q.RotateInv(dir);

    int32 axis = 0;
    float maxProjection = Abs(localDir.x);
    if (Abs(localDir.y) > maxProjection)
    {
        axis = 1;
        maxProjection = Abs(localDir.y);
    }
    if (Abs(localDir.z) > maxProjection)
    {
        axis = 2;
    }

    int32 face = axis * 2 + (localDir[axis] > 0.0f ? 1 : 0);

    Face outFace;
    outFace.count = 4;
    outFace.normal = transform.q.Rotate(boxNormals[face]);
    for (int32 i = 0; i < 4; ++i)
    {
        int32 vertexId = boxFaceVertexIndices[face][i];
        outFace.points[i].id = vertexId;
        outFace.points[i].p = Mul(transform, GetVertex(vertexId));
    }

    return outFace;
}

bool Box::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 boxQ = localQ - center;
    Vec3 clamped{
        Clamp(boxQ.x, -halfExtents.x, halfExtents.x),
        Clamp(boxQ.y, -halfExtents.y, halfExtents.y),
        Clamp(boxQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = boxQ - clamped;

    return Length2(delta) <= radius * radius;
}

Vec3 Box::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 boxQ = localQ - center;
    Vec3 clamped{
        Clamp(boxQ.x, -halfExtents.x, halfExtents.x),
        Clamp(boxQ.y, -halfExtents.y, halfExtents.y),
        Clamp(boxQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = boxQ - clamped;

    float distance = delta.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    Vec3 localClosest = center + clamped + delta * radius;
    return Mul(transform, localClosest);
}

} // namespace muli3
