#include "muli3/capsule_shape.h"
#include "muli3/distance.h"
#include "muli3/frame.h"

namespace muli3
{

CapsuleShape::CapsuleShape(float height, float inRadius, const Transform& transform)
    : Shape{ Shape::capsule, inRadius * Max(Abs(transform.s.x), Abs(transform.s.z)) }
{
    float halfHeight = height * 0.5f * Abs(transform.s.y);

    va = Vec3{ 0.0f, -halfHeight, 0.0f };
    vb = Vec3{ 0.0f, halfHeight, 0.0f };
    center = transform.p;

    va = Mul(transform, va);
    vb = Mul(transform, vb);

    float length = Dist(va, vb);
    volume = pi * radius * radius * length + 4.0f / 3.0f * pi * radius * radius * radius;
}

CapsuleShape::CapsuleShape(const Vec3& p1, const Vec3& p2, float inRadius, const Transform& transform)
    : Shape{ Shape::capsule, inRadius * Max(Abs(transform.s.x), Abs(transform.s.z)) }
{
    va = p1;
    vb = p2;
    center = (p1 + p2) * 0.5f;

    va = Mul(transform, va);
    vb = Mul(transform, vb);
    center = Mul(transform, center);

    float length = Dist(va, vb);
    volume = pi * radius * radius * length + 4.0f / 3.0f * pi * radius * radius * radius;
}

void CapsuleShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    float height = Dist(va, vb);
    float radius2 = radius * radius;

    float cylinderMass = pi * radius2 * height * density;
    float hemisphereMass = 2.0f * pi / 3.0f * radius2 * radius * density;

    float inertiaAxis = cylinderMass * radius2 * 0.5f;
    float inertiaSide = inertiaAxis * 0.5f + cylinderMass * height * height / 12.0f;

    float hemisphereInertia = hemisphereMass * 4.0f * radius2 / 5.0f;
    inertiaAxis += hemisphereInertia;
    inertiaSide += hemisphereInertia + hemisphereMass * (0.5f * height * height + 0.75f * height * radius);

    Vec3 axis = NormalizeSafe(vb - va);
    if (Length2(axis) <= epsilon)
    {
        axis = y_axis;
    }

    Frame frame = Frame::FromY(axis);
    Mat3 rotation{ frame.x, frame.y, frame.z };
    Mat3 inertiaCenter = rotation * Mat3(Vec3{ inertiaSide, inertiaAxis, inertiaSide }) * rotation.GetTranspose();

    outMassData->mass = cylinderMass + hemisphereMass * 2.0f;
    outMassData->centerOfMass = center;

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

void CapsuleShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 a = Mul(transform, va);
    Vec3 b = Mul(transform, vb);
    float scaledRadius = radius * Max(Abs(transform.s.x), Max(Abs(transform.s.y), Abs(transform.s.z)));
    Vec3 r{ scaledRadius, scaledRadius, scaledRadius };

    *outAABB = AABB{ Min(a, b) - r, Max(a, b) + r };
}

Face CapsuleShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(dir);

    Face face;
    face.count = 2;
    face.points[0].id = 0;
    face.points[0].p = Mul(transform, va);
    face.points[1].id = 1;
    face.points[1].p = Mul(transform, vb);
    face.normal = dir;
    return face;
}

bool CapsuleShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = ClosestPointVsSegment(va, vb, localQ);
    return Dist2(localQ, closest) <= Sqr(radius);
}

Vec3 CapsuleShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = ClosestPointVsSegment(va, vb, localQ);
    Vec3 delta = localQ - closest;

    float distance = delta.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, closest + delta * radius);
}

bool CapsuleShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    RayCastInput localInput = input;
    localInput.from = MulT(transform, input.from);
    localInput.to = MulT(transform, input.to);

    if (RayCastCapsule(va, vb, radius, localInput, output) == false)
    {
        return false;
    }

    output->normal = transform.q.Rotate(output->normal);
    return true;
}

} // namespace muli3
