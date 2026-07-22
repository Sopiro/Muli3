#pragma once

#include "joint.h"

namespace muli3
{

// Pulley constraint: constrains the combined rope length between two body anchors and two ground anchors
// 1 DOF constraint (constrains the scalar pulley length)
class PulleyJoint : public Joint
{
public:
    PulleyJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio,
        float frequency,
        float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetGroundAnchorA() const;
    const Vec3& GetGroundAnchorB() const;
    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    float GetPulleyLength() const;
    void SetPulleyLength(float newLength);

private:
    Vec3 groundAnchorA;
    Vec3 groundAnchorB;
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    float length;
    float ratio;

    Vec3 ra;
    Vec3 rb;
    Vec3 ua;
    Vec3 ub;
    float m;

    float bias;
    float impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(float lambda);
};

inline const Vec3& PulleyJoint::GetGroundAnchorA() const
{
    return groundAnchorA;
}

inline const Vec3& PulleyJoint::GetGroundAnchorB() const
{
    return groundAnchorB;
}

inline const Vec3& PulleyJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& PulleyJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float PulleyJoint::GetPulleyLength() const
{
    return length;
}

inline void PulleyJoint::SetPulleyLength(float newLength)
{
    length = Max(newLength, 0.0f);
}

} // namespace muli3
