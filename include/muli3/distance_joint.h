#pragma once

#include "joint.h"

namespace muli3
{

// Distance constraint: constrains the separation between two anchor points
// 1 DOF constraint (equal distance or min/max distance limit)
class DistanceJoint : public Joint
{
public:
    DistanceJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float minLength,
        float maxLength,
        float frequency,
        float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

    float GetJointLength() const;
    void SetJointLength(float newLength);

    float GetJointMinLength() const;
    void SetJointMinLength(float newMinLength);
    float GetJointMaxLength() const;
    void SetJointMaxLength(float newMaxLength);

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    float minLength, maxLength;

    Vec3 ra;
    Vec3 rb;
    Vec3 d;
    float m;

    float bias;
    float impulseSum;
    int32 limitState;

    void ApplyImpulse(float lambda);
};

inline const Vec3& DistanceJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& DistanceJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float DistanceJoint::GetJointLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointLength(float newLength)
{
    minLength = Max(newLength, 0.0f);
    maxLength = minLength;
}

inline float DistanceJoint::GetJointMinLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointMinLength(float newMinLength)
{
    minLength = Max(newMinLength, 0.0f);
    maxLength = Max(minLength, maxLength);
}

inline float DistanceJoint::GetJointMaxLength() const
{
    return maxLength;
}

inline void DistanceJoint::SetJointMaxLength(float newMaxLength)
{
    maxLength = Max(newMaxLength, 0.0f);
    minLength = Min(minLength, maxLength);
}

} // namespace muli3
