#pragma once

#include "joint.h"

namespace muli3
{

class PulleyJoint : public Joint
{
public:
    PulleyJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

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
    length = newLength;
}

} // namespace muli3
