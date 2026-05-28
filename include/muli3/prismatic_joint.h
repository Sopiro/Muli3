#pragma once

#include "joint.h"

namespace muli3
{

// Prismatic(Slider) constraint: line constraint + relative orientation constraint
// 5 DOF constraint (constrains 2 translational DOFs perpendicular to the axis and 3 rotational DOFs)
class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    const Quat& GetOrientationOffset() const;

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxis;
    Quat orientationOffset;

    // For linear part (2 DOF perpendicular to axis)
    Vec3 t1, t2;
    Vec3 sa1, sa2;
    Vec3 sb1, sb2;
    Mat2 linearM;
    Vec2 linearBias;
    Vec2 linearImpulseSum;

    // For angular part (3 DOF)
    Mat3 angularM;
    Vec3 angularBias;
    Vec3 angularImpulseSum;

    void ApplyImpulse(const Vec2& linearLambda, const Vec3& angularLambda);
};

inline const Vec3& PrismaticJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& PrismaticJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline const Quat& PrismaticJoint::GetOrientationOffset() const
{
    return orientationOffset;
}

} // namespace muli3
