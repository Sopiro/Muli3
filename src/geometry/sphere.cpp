#include <muli3/sphere.h>

namespace muli3
{

Sphere::Sphere(float radius, const Transform& transform)
    : Shape{ ShapeType::sphere, radius }
{
    center = transform.p;
    volume = 4.0f / 3.0f * pi * radius * radius * radius;
}

void Sphere::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;
    outMassData->inertia = ComputeLocalInertiaTensor(outMassData->mass);
}

Mat3 Sphere::ComputeLocalInertiaTensor(float mass) const
{
    const float value = 0.4f * mass * radius * radius;
    return Mat3(Vec3(value, 0, 0), Vec3(0, value, 0), Vec3(0, 0, value));
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
