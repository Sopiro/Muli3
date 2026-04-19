#pragma once

#include "sphere.h"

namespace muli3
{

class RigidBody
{
public:
    Transform transform{};
    Vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };
    Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };
    float inverseMass = 0.0f;
    float restitution = 0.35f;
    float friction = 0.6f;
    bool visible = true;
    std::shared_ptr<Shape> shape{};

    RigidBody() = default;

    bool IsStatic() const;
    float GetMass() const;
    Vec3 GetWorldCenterOfMass() const;
    Mat3 GetInverseInertiaTensorLocal() const;
    Mat3 GetInverseInertiaTensorWorld() const;

    void SetMass(float mass);
    void ApplyLinearImpulse(const Vec3& impulse);
    void ApplyAngularImpulse(const Vec3& impulse);
    void Integrate(float dt);
};

inline bool RigidBody::IsStatic() const
{
    return inverseMass <= epsilon;
}

inline float RigidBody::GetMass() const
{
    return IsStatic() ? 0.0f : 1.0f / inverseMass;
}

} // namespace muli3
