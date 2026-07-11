#include "shape.h"

#include "box_shape.h"          // IWYU pragma: export
#include "capsule_shape.h"      // IWYU pragma: export
#include "convex_shape.h"       // IWYU pragma: export
#include "height_field_shape.h" // IWYU pragma: export
#include "polygon_shape.h"      // IWYU pragma: export
#include "sphere_shape.h"       // IWYU pragma: export
#include "triangle_shape.h"     // IWYU pragma: export

namespace muli3
{

inline void Shape::ComputeMass(float density, MassData* outMassData) const
{
    Dispatch([&](auto shape) { shape->ComputeMass(density, outMassData); });
}

inline void Shape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Dispatch([&](auto shape) { shape->ComputeAABB(transform, outAABB); });
}

inline int32 Shape::GetVertexCount() const
{
    return Dispatch([&](auto shape) { return shape->GetVertexCount(); });
}

inline Vec3 Shape::GetVertex(int32 index) const
{
    return Dispatch([&](auto shape) { return shape->GetVertex(index); });
}

inline int32 Shape::GetVertexIndex(int32 index) const
{
    return Dispatch([&](auto shape) { return shape->GetVertexIndex(index); });
}

inline int32 Shape::GetSupport(const Vec3& localDir) const
{
    return Dispatch([&](auto shape) { return shape->GetSupport(localDir); });
}

inline Face Shape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    return Dispatch([&](auto shape) { return shape->GetFeaturedFace(transform, dir); });
}

inline bool Shape::TestPoint(const Transform& transform, const Vec3& q) const
{
    return Dispatch([&](auto shape) { return shape->TestPoint(transform, q); });
}

inline Vec3 Shape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    return Dispatch([&](auto shape) { return shape->GetClosestPoint(transform, q); });
}

inline bool Shape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    return Dispatch([&](auto shape) { return shape->RayCast(transform, input, output); });
}

} // namespace muli3
