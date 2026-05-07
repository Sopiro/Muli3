#pragma once

#include "joint.h"

namespace muli3
{

// Restricts the swing between two axes to stay inside a cone.
// This joint does not constrain twist around the axis.
class ConeSwingJoint : public Joint
{
public:
    ConeSwingJoint(
        RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float maxAngle, float frequency, float dampingRatio, float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;

    float GetJointAngle() const;
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    Vec3 localAxisA;
    Vec3 localAxisB;
    float maxAngle;
    float currentAngle;

    Vec3 swingAxis;
    float m;
    float bias;
    float impulseSum;
    bool activeLimit;

    void ApplyImpulse(float lambda);
};

inline const Vec3& ConeSwingJoint::GetLocalAxisA() const
{
    return localAxisA;
}

inline const Vec3& ConeSwingJoint::GetLocalAxisB() const
{
    return localAxisB;
}

inline float ConeSwingJoint::GetJointAngle() const
{
    return currentAngle;
}

inline float ConeSwingJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void ConeSwingJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = Clamp(newMaxAngle, 0.0f, pi);
    bodyA->Awake();
    bodyB->Awake();
}

} // namespace muli3
