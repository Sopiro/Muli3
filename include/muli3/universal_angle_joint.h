#pragma once

#include "joint.h"

namespace muli3
{

// Angular part of a universal joint.
// Keeps body-fixed axes A and B perpendicular, leaving steering around A and spin around B free.
class UniversalAngleJoint : public Joint
{
public:
    UniversalAngleJoint(Body* bodyA, Body* bodyB, const Vec3& axisA, const Vec3& axisB, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;
    const Vec3& GetLocalReferenceAxisA() const;
    const Vec3& GetLocalReferenceAxisB() const;

    float GetSteeringAngle() const;
    bool IsSteeringMotorEnabled() const;
    void SetSteeringMotorEnabled(bool enabled);
    float GetTargetSteeringAngle() const;
    void SetTargetSteeringAngle(float angle);
    float GetSteeringFrequency() const;
    void SetSteeringFrequency(float frequency);
    float GetSteeringDampingRatio() const;
    void SetSteeringDampingRatio(float dampingRatio);
    float GetMaxSteeringTorque() const;
    void SetMaxSteeringTorque(float torque);

    bool IsSteeringLimitEnabled() const;
    void SetSteeringLimitEnabled(bool enabled);
    float GetSteeringMinAngle() const;
    void SetSteeringMinAngle(float angle);
    float GetSteeringMaxAngle() const;
    void SetSteeringMaxAngle(float angle);

    bool IsSpinMotorEnabled() const;
    void SetSpinMotorEnabled(bool enabled);
    float GetSpinAngle() const;
    float GetSpinSpeed() const;
    void SetSpinSpeed(float speed);
    float GetMaxSpinTorque() const;
    void SetMaxSpinTorque(float torque);

    bool IsSpinLimitEnabled() const;
    void SetSpinLimitEnabled(bool enabled);
    float GetSpinMinAngle() const;
    void SetSpinMinAngle(float angle);
    float GetSpinMaxAngle() const;
    void SetSpinMaxAngle(float angle);

private:
    Vec3 localAxisA;     // Steering axis fixed to body A
    Vec3 localAxisB;     // Spin axis fixed to body B
    Vec3 localRefAxisA;  // Zero-steering direction fixed to body A
    Vec3 localRefAxisB;  // Zero-spin direction fixed to body B

    // Perpendicular constraint removes rotation around cross(axisB, axisA).
    Vec3 perpAxis;
    float perpM;
    float perpBias;
    float perpImpulseSum;
    float perpBeta;
    float perpGamma;

    // Steering servo and limit act around the angular gradient of the measured steering angle.
    Vec3 steeringAxis;
    float steeringAngle;
    bool steeringMotorEnabled;
    float targetSteeringAngle;
    float steeringFrequency;
    float steeringDampingRatio;
    float maxSteeringTorque;
    float steeringM;
    float steeringBias;
    float steeringImpulseSum;
    float steeringBeta;
    float steeringGamma;

    bool steeringLimitEnabled;
    float steeringMinAngle;
    float steeringMaxAngle;
    float steeringLimitBias;
    float steeringLimitImpulseSum;
    int32 steeringLimitState;

    // Spin motor and limit act around body B's spin axis.
    Vec3 spinAxis;
    float spinAngle;
    bool spinMotorEnabled;
    float spinSpeed;
    float maxSpinTorque;
    float spinM;
    float spinImpulseSum;

    bool spinLimitEnabled;
    float spinMinAngle;
    float spinMaxAngle;
    float spinLimitM;
    float spinLimitBias;
    float spinLimitImpulseSum;
    float spinBeta;
    float spinGamma;
    int32 spinLimitState;

    void ApplyAngularImpulse(const Vec3& axis, float lambda);
};

inline const Vec3& UniversalAngleJoint::GetLocalAxisA() const
{
    return localAxisA;
}

inline const Vec3& UniversalAngleJoint::GetLocalAxisB() const
{
    return localAxisB;
}

inline const Vec3& UniversalAngleJoint::GetLocalReferenceAxisA() const
{
    return localRefAxisA;
}

inline const Vec3& UniversalAngleJoint::GetLocalReferenceAxisB() const
{
    return localRefAxisB;
}

inline float UniversalAngleJoint::GetSteeringAngle() const
{
    return steeringAngle;
}

inline bool UniversalAngleJoint::IsSteeringMotorEnabled() const
{
    return steeringMotorEnabled;
}

inline void UniversalAngleJoint::SetSteeringMotorEnabled(bool enabled)
{
    steeringMotorEnabled = enabled;
    if (!steeringMotorEnabled)
    {
        steeringImpulseSum = 0.0f;
    }
}

inline float UniversalAngleJoint::GetTargetSteeringAngle() const
{
    return targetSteeringAngle;
}

inline void UniversalAngleJoint::SetTargetSteeringAngle(float angle)
{
    targetSteeringAngle = angle;
}

inline float UniversalAngleJoint::GetSteeringFrequency() const
{
    return steeringFrequency;
}

inline void UniversalAngleJoint::SetSteeringFrequency(float frequency)
{
    steeringFrequency = frequency;
}

inline float UniversalAngleJoint::GetSteeringDampingRatio() const
{
    return steeringDampingRatio;
}

inline void UniversalAngleJoint::SetSteeringDampingRatio(float dampingRatio)
{
    steeringDampingRatio = Clamp(dampingRatio, 0.0f, 1.0f);
}

inline float UniversalAngleJoint::GetMaxSteeringTorque() const
{
    return maxSteeringTorque;
}

inline void UniversalAngleJoint::SetMaxSteeringTorque(float torque)
{
    maxSteeringTorque = torque < 0.0f ? max_float : torque;
}

inline bool UniversalAngleJoint::IsSteeringLimitEnabled() const
{
    return steeringLimitEnabled;
}

inline void UniversalAngleJoint::SetSteeringLimitEnabled(bool enabled)
{
    steeringLimitEnabled = enabled;
    if (!steeringLimitEnabled)
    {
        steeringLimitImpulseSum = 0.0f;
    }
}

inline float UniversalAngleJoint::GetSteeringMinAngle() const
{
    return steeringMinAngle;
}

inline void UniversalAngleJoint::SetSteeringMinAngle(float angle)
{
    steeringMinAngle = angle;
    steeringMaxAngle = Max(steeringMinAngle, steeringMaxAngle);
}

inline float UniversalAngleJoint::GetSteeringMaxAngle() const
{
    return steeringMaxAngle;
}

inline void UniversalAngleJoint::SetSteeringMaxAngle(float angle)
{
    steeringMaxAngle = angle;
    steeringMinAngle = Min(steeringMinAngle, steeringMaxAngle);
}

inline bool UniversalAngleJoint::IsSpinMotorEnabled() const
{
    return spinMotorEnabled;
}

inline void UniversalAngleJoint::SetSpinMotorEnabled(bool enabled)
{
    spinMotorEnabled = enabled;
    if (!spinMotorEnabled)
    {
        spinImpulseSum = 0.0f;
    }
}

inline float UniversalAngleJoint::GetSpinAngle() const
{
    return spinAngle;
}

inline float UniversalAngleJoint::GetSpinSpeed() const
{
    return spinSpeed;
}

inline void UniversalAngleJoint::SetSpinSpeed(float speed)
{
    spinSpeed = speed;
}

inline float UniversalAngleJoint::GetMaxSpinTorque() const
{
    return maxSpinTorque;
}

inline void UniversalAngleJoint::SetMaxSpinTorque(float torque)
{
    maxSpinTorque = torque < 0.0f ? max_float : torque;
}

inline bool UniversalAngleJoint::IsSpinLimitEnabled() const
{
    return spinLimitEnabled;
}

inline void UniversalAngleJoint::SetSpinLimitEnabled(bool enabled)
{
    spinLimitEnabled = enabled;
    if (!spinLimitEnabled)
    {
        spinLimitImpulseSum = 0.0f;
    }
}

inline float UniversalAngleJoint::GetSpinMinAngle() const
{
    return spinMinAngle;
}

inline void UniversalAngleJoint::SetSpinMinAngle(float angle)
{
    spinMinAngle = angle;
    spinMaxAngle = Max(spinMinAngle, spinMaxAngle);
}

inline float UniversalAngleJoint::GetSpinMaxAngle() const
{
    return spinMaxAngle;
}

inline void UniversalAngleJoint::SetSpinMaxAngle(float angle)
{
    spinMaxAngle = angle;
    spinMinAngle = Min(spinMinAngle, spinMaxAngle);
}

} // namespace muli3
