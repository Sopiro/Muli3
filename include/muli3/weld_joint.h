#pragma once

#include "joint.h"

namespace muli3
{

// Rigid attachment constraint: BallSocketJoint + relative orientation constraint
// 6 DOF constraint (constrains 3 translational DOFs and 3 rotational DOFs)
class WeldJoint : public Joint
{
public:
    WeldJoint(RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

    const Quat& GetOrientationOffset() const;

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;

    Quat orientationOffset;

    Vec3 ra;
    Vec3 rb;

    // Effective mass for linear part (3x3) and angular part (3x3) solved separately
    Mat3 linearM;
    Mat3 angularM;

    Vec3 linearBias;
    Vec3 angularBias;

    Vec3 linearImpulseSum;
    Vec3 angularImpulseSum;

    void ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda);
};

inline const Vec3& WeldJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& WeldJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline const Quat& WeldJoint::GetOrientationOffset() const
{
    return orientationOffset;
}

} // namespace muli3
