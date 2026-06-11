#pragma once

#include "joint.h"

namespace muli3
{

// Twist limit constraint: constrains the relative angle around a shared reference axis
// 1 DOF angular limit constraint
class TwistAngleJoint : public Joint
{
public:
    TwistAngleJoint(
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

    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    float beta;
    float gamma;
    int32 limitState;

    void ApplyAngleImpulse(float lambda);
};

inline const Vec3& TwistAngleJoint::GetLocalAxisA() const
{
    return localAxisA;
}

inline const Vec3& TwistAngleJoint::GetLocalAxisB() const
{
    return localAxisB;
}

inline float TwistAngleJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float TwistAngleJoint::GetJointAngle() const
{
    return currentAngle;
}

inline void TwistAngleJoint::SetJointAngle(float newAngle)
{
    minAngle = newAngle;
    maxAngle = newAngle;
}

inline float TwistAngleJoint::GetJointMinAngle() const
{
    return minAngle;
}

inline void TwistAngleJoint::SetJointMinAngle(float newMinAngle)
{
    minAngle = newMinAngle;
    maxAngle = Max(minAngle, maxAngle);
}

inline float TwistAngleJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void TwistAngleJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = newMaxAngle;
    minAngle = Min(minAngle, maxAngle);
}

} // namespace muli3
