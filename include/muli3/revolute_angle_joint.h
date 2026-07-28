#pragma once

#include "joint.h"

namespace muli3
{

// Angular part of a revolute joint: keeps hinge axes aligned and controls twist around the axis
// 2 DOF angular alignment constraint + optional 1 DOF angular limit and motor constraint
class RevoluteAngleJoint : public Joint
{
public:
    RevoluteAngleJoint(
        Body* bodyA, Body* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;

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
    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;
    bool limitEnabled;

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

    void ApplySwingImpulse(const Vec2& lambda);
    void ApplyTwistImpulse(float lambda);
};

inline const Vec3& RevoluteAngleJoint::GetLocalAxisA() const
{
    return localAxisA;
}

inline const Vec3& RevoluteAngleJoint::GetLocalAxisB() const
{
    return localAxisB;
}

inline float RevoluteAngleJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float RevoluteAngleJoint::GetJointAngle() const
{
    return currentAngle;
}

inline void RevoluteAngleJoint::SetJointAngle(float newAngle)
{
    minAngle = newAngle;
    maxAngle = newAngle;
}

inline bool RevoluteAngleJoint::IsLimitEnabled() const
{
    return limitEnabled;
}

inline void RevoluteAngleJoint::SetLimitEnabled(bool enabled)
{
    limitEnabled = enabled;
    if (!limitEnabled)
    {
        angleImpulseSum = 0.0f;
    }
}

inline float RevoluteAngleJoint::GetJointMinAngle() const
{
    return minAngle;
}

inline void RevoluteAngleJoint::SetJointMinAngle(float newMinAngle)
{
    minAngle = newMinAngle;
    maxAngle = Max(minAngle, maxAngle);
}

inline float RevoluteAngleJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void RevoluteAngleJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = newMaxAngle;
    minAngle = Min(minAngle, maxAngle);
}

inline bool RevoluteAngleJoint::IsMotorEnabled() const
{
    return motorEnabled;
}

inline void RevoluteAngleJoint::SetMotorEnabled(bool enabled)
{
    motorEnabled = enabled;
    if (!motorEnabled)
    {
        motorImpulseSum = 0.0f;
    }
}

inline float RevoluteAngleJoint::GetMotorSpeed() const
{
    return motorSpeed;
}

inline void RevoluteAngleJoint::SetMotorSpeed(float speed)
{
    motorSpeed = speed;
}

inline float RevoluteAngleJoint::GetMaxMotorTorque() const
{
    return maxMotorTorque;
}

inline void RevoluteAngleJoint::SetMaxMotorTorque(float torque)
{
    maxMotorTorque = torque < 0.0f ? max_float : torque;
}

} // namespace muli3
