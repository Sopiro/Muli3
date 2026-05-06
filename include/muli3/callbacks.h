#pragma once

#include "math.h"

namespace muli3
{

class RigidBody;

class RayCastAnyCallback
{
public:
    virtual ~RayCastAnyCallback() {}
    virtual float OnHitAny(RigidBody* body, Vec3 point, Vec3 normal, float fraction) = 0;
};

class RayCastClosestCallback
{
public:
    virtual ~RayCastClosestCallback() {}
    virtual void OnHitClosest(RigidBody* body, Vec3 point, Vec3 normal, float fraction) = 0;
};

class ShapeCastAnyCallback
{
public:
    virtual ~ShapeCastAnyCallback() {}
    virtual float OnHitAny(RigidBody* body, Vec3 point, Vec3 normal, float t) = 0;
};

class ShapeCastClosestCallback
{
public:
    virtual ~ShapeCastClosestCallback() {}
    virtual void OnHitClosest(RigidBody* body, Vec3 point, Vec3 normal, float t) = 0;
};

} // namespace muli3
