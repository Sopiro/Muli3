#include "muli3/sphere.h"

namespace muli3
{

Sphere::Sphere(float radius, const Transform& transform)
    : Shape{ Shape::sphere, radius }
{
    center = transform.p;
    volume = 4.0f / 3.0f * pi * radius * radius * radius;
}

void Sphere::ComputeMass(float density, MassData* outMassData) const
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

void Sphere::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    const Vec3 transformedCenter = Mul(transform, center);
    const Vec3 r{
        radius * Abs(transform.s.x),
        radius * Abs(transform.s.y),
        radius * Abs(transform.s.z),
    };
    *outAABB = AABB{ transformedCenter - r, transformedCenter + r };
}

bool Sphere::TestPoint(const Transform& transform, const Vec3& q) const
{
    const Vec3 localQ = MulT(transform, q);
    return Length2(localQ - center) <= radius * radius;
}

Vec3 Sphere::GetClosestPoint(const Transform& transform, const Vec3& q) const
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

} // namespace muli3
