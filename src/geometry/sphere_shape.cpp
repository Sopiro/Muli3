#include "muli3/sphere_shape.h"

namespace muli3
{

SphereShape::SphereShape(float inRadius, const Transform& transform)
    : Shape(Shape::sphere, inRadius)
{
    center = transform.p;
    volume = 4.0f / 3.0f * pi * radius * radius * radius;
}

SphereShape::SphereShape(const SphereShape& other, const Transform& transform)
    : SphereShape(other.radius, Transform{ Mul(transform, other.center) })
{
}

void SphereShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;
    float i = 0.4f * outMassData->mass * radius * radius;
    float x = center.x;
    float y = center.y;
    float z = center.z;
    float m = outMassData->mass;

    outMassData->inertia = Mat3(
        Vec3{ i + m * (y * y + z * z), -m * x * y, -m * x * z }, Vec3{ -m * y * x, i + m * (x * x + z * z), -m * y * z },
        Vec3{ -m * z * x, -m * z * y, i + m * (x * x + y * y) }
    );
}

void SphereShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 c = Mul(transform, center);
    *outAABB = AABB{ c - radius, c + radius };
}

bool SphereShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    const Vec3 localQ = MulT(transform, q);
    return Length2(localQ - center) <= radius * radius;
}

Vec3 SphereShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    const Vec3 localQ = MulT(transform, q);
    Vec3 d = localQ - center;
    const float distance = d.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, center + d * radius);
}

bool SphereShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    return RayCastSphere(Mul(transform, center), radius, input, output);
}

} // namespace muli3
