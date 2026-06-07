#pragma once

#include "body.h"
#include "collision_filter.h"
#include "material.h"
#include "shape.h"

namespace muli3
{

class ColliderDestroyCallback;
class ContactListener;

class Collider
{
public:
    Collider(const Collider&) = delete;
    Collider& operator=(const Collider&) = delete;

    Body* GetBody();
    const Body* GetBody() const;

    Collider* GetNext();
    const Collider* GetNext() const;

    Shape::Type GetType() const;

    Shape* GetShape();
    const Shape* GetShape() const;

    float GetMass() const;
    void SetMass(float mass);
    float GetDensity() const;
    void SetDensity(float density);

    const Material& GetMaterial() const;
    void SetMaterial(const Material& material);

    float GetFriction() const;
    void SetFriction(float friction);
    float GetRestitution() const;
    void SetRestitution(float restitution);
    float GetRestitutionTreshold() const;
    void SetRestitutionTreshold(float treshold);
    Vec2 GetSurfaceSpeed() const;
    void SetSurfaceSpeed(const Vec2& surfaceSpeed);

    const CollisionFilter& GetFilter() const;
    void SetFilter(const CollisionFilter& filter);

    bool IsEnabled() const;
    void SetEnabled(bool enabled);

    AABB GetAABB() const;
    MassData GetMassData() const;

    bool TestPoint(const Vec3& q) const;
    Vec3 GetClosestPoint(const Vec3& q) const;
    bool RayCast(const RayCastInput& input, RayCastOutput* output) const;

    ColliderDestroyCallback* OnDestroy;
    muli3::ContactListener* ContactListener;
    void* UserData;

private:
    friend class AABBTree;
    friend class BroadPhase;
    friend class Contact;
    friend class ConstraintGraph;
    friend class Body;
    friend class World;

    Collider();
    ~Collider();

    void Create(Body* body, Shape* shape, const Transform& transform, float density, const Material& material);
    void Destroy(World* world);

    Body* body;
    Collider* next;
    Shape* shape;

    float density;
    Material material;
    CollisionFilter filter;

    int32 node;
    bool enabled;
};

inline Body* Collider::GetBody()
{
    return body;
}

inline const Body* Collider::GetBody() const
{
    return body;
}

inline Collider* Collider::GetNext()
{
    return next;
}

inline const Collider* Collider::GetNext() const
{
    return next;
}

inline Shape::Type Collider::GetType() const
{
    return shape->GetType();
}

inline Shape* Collider::GetShape()
{
    return shape;
}

inline const Shape* Collider::GetShape() const
{
    return shape;
}

inline float Collider::GetMass() const
{
    return shape->GetVolume() * density;
}

inline void Collider::SetMass(float newMass)
{
    density = newMass / shape->GetVolume();
    body->ResetMassData();
}

inline float Collider::GetDensity() const
{
    return density;
}

inline void Collider::SetDensity(float newDensity)
{
    density = newDensity;
    body->ResetMassData();
}

inline const Material& Collider::GetMaterial() const
{
    return material;
}

inline void Collider::SetMaterial(const Material& newMaterial)
{
    material = newMaterial;
}

inline float Collider::GetFriction() const
{
    return material.friction;
}

inline void Collider::SetFriction(float newFriction)
{
    material.friction = newFriction;
}

inline float Collider::GetRestitution() const
{
    return material.restitution;
}

inline void Collider::SetRestitution(float newRestitution)
{
    material.restitution = newRestitution;
}

inline float Collider::GetRestitutionTreshold() const
{
    return material.restitutionTreshold;
}

inline void Collider::SetRestitutionTreshold(float newRestitutionTreshold)
{
    material.restitutionTreshold = newRestitutionTreshold;
}

inline Vec2 Collider::GetSurfaceSpeed() const
{
    return material.surfaceSpeed;
}

inline void Collider::SetSurfaceSpeed(const Vec2& newSurfaceSpeed)
{
    material.surfaceSpeed = newSurfaceSpeed;
}

inline const CollisionFilter& Collider::GetFilter() const
{
    return filter;
}

inline void Collider::SetFilter(const CollisionFilter& newFilter)
{
    filter = newFilter;
}

inline bool Collider::IsEnabled() const
{
    return enabled;
}

inline void Collider::SetEnabled(bool newEnabled)
{
    enabled = newEnabled;
}

inline AABB Collider::GetAABB() const
{
    AABB aabb;
    shape->ComputeAABB(body->transform, &aabb);
    return aabb;
}

inline MassData Collider::GetMassData() const
{
    MassData massData;
    shape->ComputeMass(density, &massData);
    return massData;
}

inline bool Collider::TestPoint(const Vec3& q) const
{
    return shape->TestPoint(body->transform, q);
}

inline Vec3 Collider::GetClosestPoint(const Vec3& q) const
{
    return shape->GetClosestPoint(body->transform, q);
}

inline bool Collider::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    return shape->RayCast(body->transform, input, output);
}

} // namespace muli3
