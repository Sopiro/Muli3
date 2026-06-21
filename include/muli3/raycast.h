#pragma once

#include "math.h"

namespace muli3
{

struct RayCastInput
{
    Vec3 from;
    Vec3 to;
    float maxFraction;
    float radius;
};

struct RayCastOutput
{
    Vec3 normal;
    float fraction;
};

struct ShapeCastInput
{
    class Shape* shapeA;
    class Shape* shapeB;
    Transform* tfA;
    Transform* tfB;
    Vec3 translationA;
    Vec3 translationB;
};

struct ShapeCastOutput
{
    Vec3 point;
    Vec3 normal;
    float t;
};

bool RayCastSphere(const Vec3& p, float r, const RayCastInput& input, RayCastOutput* output);
bool RayCastCapsule(const Vec3& va, const Vec3& vb, float radius, const RayCastInput& input, RayCastOutput* output);
bool RayCastTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const RayCastInput& input, RayCastOutput* output);

bool ShapeCast(
    const Shape* a,
    const Transform& tfA,
    const Shape* b,
    const Transform& tfB,
    const Vec3& translationA,
    const Vec3& translationB,
    ShapeCastOutput* output
);

struct AABBCastInput
{
    Vec3 from;
    Vec3 to;
    float maxFraction;
    Vec3 halfExtents;
};

} // namespace muli3
