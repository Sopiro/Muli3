#pragma once

#include "common.h"
#include "joint.h"

namespace muli3
{

class MotorJoint : public Joint
{
public:
    MotorJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        float maxJointForce,
        float maxJointTorque,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    float GetMaxForce() const;
    void SetMaxForce(float maxForce);
    float GetMaxTorque() const;
    void SetMaxTorque(float maxTorque);
    const Vec3& GetLinearOffset() const;
    void SetLinearOffset(const Vec3& linearOffset);
    const Vec3& GetAngularOffset() const;
    void SetAngularOffset(const Vec3& angularOffset);

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Quat orientationOffset;

    Vec3 linearOffset;
    Vec3 angularOffset;

    float maxForce;
    float maxTorque;

    Vec3 ra;
    Vec3 rb;
    Mat3 linearM;
    Mat3 angularM;

    Vec3 linearBias;
    Vec3 angularBias;

    Vec3 linearImpulseSum;
    Vec3 angularImpulseSum;

    void ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda);
};

inline const Vec3& MotorJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& MotorJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float MotorJoint::GetMaxForce() const
{
    return maxForce;
}

inline void MotorJoint::SetMaxForce(float newMaxForce)
{
    maxForce = newMaxForce;
}

inline float MotorJoint::GetMaxTorque() const
{
    return maxTorque;
}

inline void MotorJoint::SetMaxTorque(float newMaxTorque)
{
    maxTorque = newMaxTorque;
}

inline const Vec3& MotorJoint::GetLinearOffset() const
{
    return linearOffset;
}

inline void MotorJoint::SetLinearOffset(const Vec3& newLinearOffset)
{
    linearOffset = newLinearOffset;
}

inline const Vec3& MotorJoint::GetAngularOffset() const
{
    return angularOffset;
}

inline void MotorJoint::SetAngularOffset(const Vec3& newAngularOffset)
{
    angularOffset = newAngularOffset;
}

} // namespace muli3
