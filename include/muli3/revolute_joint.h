#pragma once

#include "joint.h"

namespace muli3
{

// Revolute(Hinge) constraint: BallSocketJoint + hinge axis alignment with twist control
// 5 DOF constraint + optional 1 DOF angular limit and motor constraint
class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency,
        float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;
    const Vec3& GetLocalNormalAxisA() const;
    const Vec3& GetLocalNormalAxisB() const;

    float GetJointAngleOffset() const;
    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    bool IsLimitEnabled() const;
    void SetLimitEnabled(bool enabled);
    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

    bool IsMotorEnabled() const;
    void SetMotorEnabled(bool enabled);
    float GetMotorSpeed() const;
    void SetMotorSpeed(float speed);
    float GetMaxMotorTorque() const;
    void SetMaxMotorTorque(float torque);

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;
    bool limitEnabled;

    Vec3 ra;
    Vec3 rb;
    Mat3 linearM;
    Vec3 linearBias;
    Vec3 linearImpulseSum;
    float linearBeta;
    float linearGamma;

    Vec3 swingAxis1;
    Vec3 swingAxis2;
    Mat2 swingM;
    Vec2 swingBias;
    Vec2 swingImpulseSum;
    float swingBeta;
    float swingGamma;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    float angleBeta;
    float angleGamma;
    int32 limitState;

    bool motorEnabled;
    float motorSpeed;
    float maxMotorTorque;
    float motorM;
    float motorImpulseSum;

    void ApplyLinearImpulse(const Vec3& lambda);
    void ApplySwingImpulse(const Vec2& lambda);
    void ApplyTwistImpulse(float lambda);
};

inline const Vec3& RevoluteJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& RevoluteJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline const Vec3& RevoluteJoint::GetLocalAxisA() const
{
    return localAxisA;
}

inline const Vec3& RevoluteJoint::GetLocalAxisB() const
{
    return localAxisB;
}

inline const Vec3& RevoluteJoint::GetLocalNormalAxisA() const
{
    return localNormalAxisA;
}

inline const Vec3& RevoluteJoint::GetLocalNormalAxisB() const
{
    return localNormalAxisB;
}

inline float RevoluteJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float RevoluteJoint::GetJointAngle() const
{
    return currentAngle;
}

inline void RevoluteJoint::SetJointAngle(float newAngle)
{
    minAngle = newAngle;
    maxAngle = newAngle;
}

inline bool RevoluteJoint::IsLimitEnabled() const
{
    return limitEnabled;
}

inline void RevoluteJoint::SetLimitEnabled(bool enabled)
{
    limitEnabled = enabled;
    if (!limitEnabled)
    {
        angleImpulseSum = 0.0f;
    }
}

inline float RevoluteJoint::GetJointMinAngle() const
{
    return minAngle;
}

inline void RevoluteJoint::SetJointMinAngle(float newMinAngle)
{
    minAngle = newMinAngle;
    maxAngle = Max(minAngle, maxAngle);
}

inline float RevoluteJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void RevoluteJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = newMaxAngle;
    minAngle = Min(minAngle, maxAngle);
}

inline bool RevoluteJoint::IsMotorEnabled() const
{
    return motorEnabled;
}

inline void RevoluteJoint::SetMotorEnabled(bool enabled)
{
    motorEnabled = enabled;
    if (!motorEnabled)
    {
        motorImpulseSum = 0.0f;
    }
}

inline float RevoluteJoint::GetMotorSpeed() const
{
    return motorSpeed;
}

inline void RevoluteJoint::SetMotorSpeed(float speed)
{
    motorSpeed = speed;
}

inline float RevoluteJoint::GetMaxMotorTorque() const
{
    return maxMotorTorque;
}

inline void RevoluteJoint::SetMaxMotorTorque(float torque)
{
    maxMotorTorque = torque < 0.0f ? max_float : torque;
}

} // namespace muli3
