#include "muli3/box_shape.h"

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

BoxShape::BoxShape(float width, float height, float depth, float inRadius, const Transform& transform)
    : Shape{ Shape::box, inRadius }
    , halfExtents{
        width * 0.5f * Abs(transform.s.x),
        height * 0.5f * Abs(transform.s.y),
        depth * 0.5f * Abs(transform.s.z),
    }
    , rotation{ transform.q }
{
    center = transform.p;

    float x = halfExtents.x * 2.0f;
    float y = halfExtents.y * 2.0f;
    float z = halfExtents.z * 2.0f;
    volume = x * y * z + 2.0f * (x * y + y * z + z * x) * radius + pi * (x + y + z) * radius * radius +
             4.0f / 3.0f * pi * radius * radius * radius;
}

BoxShape::BoxShape(const BoxShape& other, const Transform& transform)
    : Shape{ Shape::box, other.radius }
    , halfExtents{
        other.halfExtents.x * Abs(transform.s.x),
        other.halfExtents.y * Abs(transform.s.y),
        other.halfExtents.z * Abs(transform.s.z),
    }
    , rotation{ transform.q * other.rotation }
{
    center = Mul(transform, other.center);

    float x = halfExtents.x * 2.0f;
    float y = halfExtents.y * 2.0f;
    float z = halfExtents.z * 2.0f;
    volume = x * y * z + 2.0f * (x * y + y * z + z * x) * radius + pi * (x + y + z) * radius * radius +
             4.0f / 3.0f * pi * radius * radius * radius;
}

void BoxShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;

    // BoxShape is the Minkowski sum of the core box and a sphere of radius r.
    // The inertia is accumulated from the non-overlapping rounded-box pieces:
    // core box, 6 face slabs, 12 edge quarter-cylinders, and 8 corner octants.
    float a = halfExtents.x;
    float b = halfExtents.y;
    float c = halfExtents.z;
    float r = radius;
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r2 * r2;
    float r5 = r4 * r;

    // Second moments about the local box center: xx = int x^2 dV, etc.
    Vec3 second{
        8.0f / 3.0f * a * a * a * b * c,
        8.0f / 3.0f * a * b * b * b * c,
        8.0f / 3.0f * a * b * c * c * c,
    };

    for (int32 axis = 0; axis < 3; ++axis)
    {
        float h0 = halfExtents[axis];
        float h1 = halfExtents[(axis + 1) % 3];
        float h2 = halfExtents[(axis + 2) % 3];

        // Face slabs
        {
            float m0 = 8.0f * h1 * h2 * ((h0 + r) * (h0 + r) * (h0 + r) - h0 * h0 * h0) / 3.0f;
            float m1 = 8.0f / 3.0f * h1 * h1 * h1 * h2 * r;
            float m2 = 8.0f / 3.0f * h1 * h2 * h2 * h2 * r;

            second[axis] += m0;
            second[(axis + 1) % 3] += m1;
            second[(axis + 2) % 3] += m2;
        }

        // Edge quarter cylinders
        {
            float length = 2.0f * h0;

            float m0 = 2.0f * pi * r2 * h0 * h0 * h0 / 3.0f;
            float m1 = length * (pi * h1 * h1 * r2 + 8.0f / 3.0f * h1 * r3 + pi * r4 / 4.0f);
            float m2 = length * (pi * h2 * h2 * r2 + 8.0f / 3.0f * h2 * r3 + pi * r4 / 4.0f);
            second[axis] += m0;
            second[(axis + 1) % 3] += m1;
            second[(axis + 2) % 3] += m2;
        }
    }

    float sphereVolume = 4.0f / 3.0f * pi * r3;
    float sphereSecond = 4.0f * pi * r5 / 15.0f;
    second.x += a * a * sphereVolume + sphereSecond;
    second.y += b * b * sphereVolume + sphereSecond;
    second.z += c * c * sphereVolume + sphereSecond;

    second *= density;

    Mat3 inertiaCenter{
        Vec3{ second.y + second.z, 0.0f, 0.0f },
        Vec3{ 0.0f, second.x + second.z, 0.0f },
        Vec3{ 0.0f, 0.0f, second.x + second.y },
    };
    Mat3 localRotation{ rotation };
    inertiaCenter = localRotation * inertiaCenter * localRotation.GetTranspose();

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

void BoxShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Quat worldOrientation = transform.q * rotation;
    Mat3 worldRotation{ worldOrientation };
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

Face BoxShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Quat worldOrientation = transform.q * rotation;
    Vec3 localDir = worldOrientation.RotateInv(dir);

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
    outFace.normal = worldOrientation.Rotate(boxNormals[face]);
    for (int32 i = 0; i < 4; ++i)
    {
        int32 vertexId = boxFaceVertexIndices[face][i];
        outFace.points[i].id = vertexId;
        outFace.points[i].p = Mul(transform, GetVertex(vertexId));
    }

    return outFace;
}

bool BoxShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    Vec3 localQ = MulT(boxTransform, q);
    Vec3 clamped{
        Clamp(localQ.x, -halfExtents.x, halfExtents.x),
        Clamp(localQ.y, -halfExtents.y, halfExtents.y),
        Clamp(localQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = localQ - clamped;

    return Length2(delta) <= radius * radius;
}

Vec3 BoxShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    Vec3 localQ = MulT(boxTransform, q);
    Vec3 clamped{
        Clamp(localQ.x, -halfExtents.x, halfExtents.x),
        Clamp(localQ.y, -halfExtents.y, halfExtents.y),
        Clamp(localQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = localQ - clamped;

    float distance = delta.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(boxTransform, clamped + delta * radius);
}

bool BoxShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    RayCastInput localInput = input;
    localInput.from = MulT(boxTransform, input.from);
    localInput.to = MulT(boxTransform, input.to);

    Vec3 p1 = localInput.from;
    Vec3 p2 = localInput.to;
    Vec3 d = p2 - p1;

    float radii = radius + input.radius;
    Vec3 extents = halfExtents + radii;

    Vec3 closest{
        Clamp(p1.x, -halfExtents.x, halfExtents.x),
        Clamp(p1.y, -halfExtents.y, halfExtents.y),
        Clamp(p1.z, -halfExtents.z, halfExtents.z),
    };
    if (Dist2(p1, closest) <= radii * radii)
    {
        return false;
    }

    float near = 0.0f;
    float far = input.maxFraction;
    int32 axis = -1;
    float sign = 0.0f;

    for (int32 i = 0; i < 3; ++i)
    {
        float p = p1[i];
        float di = d[i];
        float min = -extents[i];
        float max = extents[i];

        if (Abs(di) <= epsilon)
        {
            if (p < min || p > max)
            {
                return false;
            }

            continue;
        }

        float invD = 1.0f / di;
        float t0 = (min - p) * invD;
        float t1 = (max - p) * invD;
        float normalSign = -1.0f;

        if (t1 < t0)
        {
            std::swap(t0, t1);
            normalSign = 1.0f;
        }

        if (t0 > near)
        {
            near = t0;
            axis = i;
            sign = normalSign;
        }
        far = Min(far, t1);

        if (far < near)
        {
            return false;
        }
    }

    if (axis < 0 || near < 0.0f || near > input.maxFraction)
    {
        return false;
    }

    Vec3 q = p1 + d * near;
    int32 i1 = (axis + 1) % 3;
    int32 i2 = (axis + 2) % 3;

    if (Abs(q[i1]) <= halfExtents[i1] && Abs(q[i2]) <= halfExtents[i2])
    {
        Vec3 normal = Vec3::zero;
        normal[axis] = sign;
        output->fraction = near;
        output->normal = boxTransform.q.Rotate(normal);
        return true;
    }

    if (Abs(q[i1]) > halfExtents[i1] && Abs(q[i2]) > halfExtents[i2])
    {
        Vec3 corner = Vec3::zero;
        corner[axis] = sign * halfExtents[axis];
        corner[i1] = q[i1] > 0.0f ? halfExtents[i1] : -halfExtents[i1];
        corner[i2] = q[i2] > 0.0f ? halfExtents[i2] : -halfExtents[i2];

        if (RayCastSphere(corner, radius, localInput, output) == false)
        {
            return false;
        }

        output->normal = boxTransform.q.Rotate(output->normal);
        return true;
    }

    int32 edgeAxis = Abs(q[i1]) > halfExtents[i1] ? i2 : i1;
    int32 fixedAxis = edgeAxis == i1 ? i2 : i1;

    Vec3 edgeA = Vec3::zero;
    Vec3 edgeB = Vec3::zero;
    edgeA[axis] = sign * halfExtents[axis];
    edgeB[axis] = sign * halfExtents[axis];
    edgeA[fixedAxis] = q[fixedAxis] > 0.0f ? halfExtents[fixedAxis] : -halfExtents[fixedAxis];
    edgeB[fixedAxis] = edgeA[fixedAxis];
    edgeA[edgeAxis] = -halfExtents[edgeAxis];
    edgeB[edgeAxis] = halfExtents[edgeAxis];

    if (RayCastCapsule(edgeA, edgeB, radius, localInput, output) == false)
    {
        return false;
    }

    output->normal = boxTransform.q.Rotate(output->normal);
    return true;
}

} // namespace muli3
