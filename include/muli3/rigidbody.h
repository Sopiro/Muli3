#pragma once

#include "transform.h"

namespace muli3
{

class Shape;

class RigidBody
{
public:
    Transform transform{};
    Vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };
    Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };
    float invMass = 0.0f;
    float restitution = 0.0f;
    float friction = 0.5f;
    Shape* shape = nullptr;

    RigidBody() = default;

    bool IsStatic() const;
    float GetMass() const;
    Vec3 GetWorldCenterOfMass() const;
    Mat3 GetInverseInertiaTensorLocal() const;
    Mat3 GetInverseInertiaTensorWorld() const;

    void SetMass(float mass);
    void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);
    void ApplyLinearImpulse(const Vec3& impulse);
    void ApplyAngularImpulse(const Vec3& impulse);
    Vec3 GetVelocityAtWorldPoint(const Vec3& point) const;
    void Integrate(float dt);
};

inline bool RigidBody::IsStatic() const
{
    return invMass <= epsilon;
}

inline float RigidBody::GetMass() const
{
    return IsStatic() ? 0.0f : 1.0f / invMass;
}

} // namespace muli3
