#include <muli3/sphere.h>

namespace muli3
{

Sphere::Sphere(float radius)
    : Shape{ ShapeType::sphere }
    , radius{ radius }
{
}

Mat3 Sphere::ComputeLocalInertiaTensor(float mass) const
{
    const float value = 0.4f * mass * radius * radius;
    return Mat3::Diagonal(value, value, value);
}

AABB Sphere::ComputeAABB(const Transform& transform) const
{
    const Vec3 r{
        radius * Abs(transform.scale.x),
        radius * Abs(transform.scale.y),
        radius * Abs(transform.scale.z),
    };
    return AABB{ transform.position - r, transform.position + r };
}

} // namespace muli3
