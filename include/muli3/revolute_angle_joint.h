#pragma once

#include "joint.h"

namespace muli3
{

// Angular part of a revolute joint: keeps hinge axes aligned and limits twist around the axis
// 2 DOF angular alignment constraint + optional 1 DOF angular limit constraint
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

    Vec3 swingAxis;
    float swingM;
    float swingBias;
    float swingImpulseSum;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    int32 limitState;

    void ApplySwingImpulse(float lambda);
    void ApplyAngleImpulse(float lambda);
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

} // namespace muli3
