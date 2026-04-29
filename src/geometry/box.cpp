#include "muli3/box.h"

namespace muli3
{

namespace
{

constexpr int32 boxFaceVertexIndices[6][4] = {
    { 0, 4, 6, 2 }, // -x
    { 1, 3, 7, 5 }, // +x
    { 0, 1, 5, 4 }, // -y
    { 2, 6, 7, 3 }, // +y
    { 0, 2, 3, 1 }, // -z
    { 4, 5, 7, 6 }, // +z
};

constexpr Vec3 boxLocalNormals[6] = {
    Vec3{ -1.0f, 0.0f, 0.0f },
    Vec3{ 1.0f, 0.0f, 0.0f },
    Vec3{ 0.0f, -1.0f, 0.0f },
    Vec3{ 0.0f, 1.0f, 0.0f },
    Vec3{ 0.0f, 0.0f, -1.0f },
    Vec3{ 0.0f, 0.0f, 1.0f },
};

} // namespace

Box::Box(float width, float height, float depth, float inRadius, const Transform& transform)
    : Shape{ ShapeType::box, inRadius }
{
    halfExtents = Vec3{ width, height, depth } * 0.5f;

    const Vec3 baseVertices[8] = {
        Vec3{ -halfExtents.x, -halfExtents.y, -halfExtents.z },
        Vec3{ halfExtents.x, -halfExtents.y, -halfExtents.z },
        Vec3{ -halfExtents.x, halfExtents.y, -halfExtents.z },
        Vec3{ halfExtents.x, halfExtents.y, -halfExtents.z },
        Vec3{ -halfExtents.x, -halfExtents.y, halfExtents.z },
        Vec3{ halfExtents.x, -halfExtents.y, halfExtents.z },
        Vec3{ -halfExtents.x, halfExtents.y, halfExtents.z },
        Vec3{ halfExtents.x, halfExtents.y, halfExtents.z },
    };

    center = transform.p;
    for (int32 i = 0; i < 8; ++i)
    {
        vertices[i] = Mul(transform, baseVertices[i]);
    }

    for (int32 i = 0; i < 6; ++i)
    {
        normals[i] = transform.q.Rotate(boxLocalNormals[i]);
    }

    Vec3 fullExtents = halfExtents + Vec3{ radius, radius, radius };
    volume = 8.0f * fullExtents.x * fullExtents.y * fullExtents.z;
}

Box::Box(const Box& other, const Transform& transform)
    : Shape{ ShapeType::box, other.radius }
{
    halfExtents = other.halfExtents * transform.s;
    center = Mul(transform, other.center);

    for (int32 i = 0; i < 8; ++i)
    {
        vertices[i] = Mul(transform, other.vertices[i]);
    }

    for (int32 i = 0; i < 6; ++i)
    {
        normals[i] = transform.q.Rotate(other.normals[i]);
    }

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

    Mat3 rotation{ normals[1], normals[3], normals[5] };
    Mat3 inertiaCenter = rotation *
                         Mat3(
                             Vec3{ s * (y2 + z2), 0.0f, 0.0f },
                             Vec3{ 0.0f, s * (x2 + z2), 0.0f },
                             Vec3{ 0.0f, 0.0f, s * (x2 + y2) }
                         ) *
                         rotation.GetTranspose();

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

    Vec3 p = Mul(transform, vertices[0]);
    Vec3 min = p;
    Vec3 max = p;
    for (int32 i = 1; i < 8; ++i)
    {
        p = Mul(transform, vertices[i]);
        min = Min(min, p);
        max = Max(max, p);
    }

    Vec3 r{ radius, radius, radius };
    *outAABB = AABB{ min - r, max + r };
}

int32 Box::GetSupport(const Vec3& localDir) const
{
    int32 best = 0;
    float bestProjection = Dot(vertices[0], localDir);
    for (int32 i = 1; i < 8; ++i)
    {
        float projection = Dot(vertices[i], localDir);
        if (projection > bestProjection)
        {
            best = i;
            bestProjection = projection;
        }
    }

    return best;
}

bool Box::GetFace(int32 id, const Transform& transform, Face* outFace) const
{
    MuliAssert(outFace != nullptr);
    MuliAssert(0 <= id && id < 6);

    outFace->normal = transform.q.Rotate(normals[id]);
    outFace->count = 4;
    outFace->id = id;
    for (int32 i = 0; i < 4; ++i)
    {
        int32 vertexId = boxFaceVertexIndices[id][i];
        outFace->points[i].id = vertexId;
        outFace->points[i].p = Mul(transform, vertices[vertexId]);
    }

    return true;
}

bool Box::GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const
{
    MuliAssert(outFace != nullptr);

    int32 index = 0;
    Vec3 normal = transform.q.Rotate(normals[0]);
    float bestProjection = Dot(normal, dir);
    for (int32 i = 1; i < 6; ++i)
    {
        Vec3 n = transform.q.Rotate(normals[i]);
        float projection = Dot(n, dir);
        if (projection > bestProjection)
        {
            index = i;
            normal = n;
            bestProjection = projection;
        }
    }

    return GetFace(index, transform, outFace);
}

bool Box::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    for (int32 i = 0; i < 6; ++i)
    {
        if (Dot(normals[i], localQ - vertices[boxFaceVertexIndices[i][0]]) > radius)
        {
            return false;
        }
    }

    return true;
}

Vec3 Box::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = localQ;

    for (int32 i = 0; i < 6; ++i)
    {
        float separation = Dot(normals[i], closest - vertices[boxFaceVertexIndices[i][0]]);
        if (separation > 0.0f)
        {
            closest -= normals[i] * separation;
        }
    }

    Vec3 d = localQ - closest;
    float distance = d.Normalize();
    if (distance > radius)
    {
        closest += d * radius;
    }

    return Mul(transform, closest);
}

} // namespace muli3
