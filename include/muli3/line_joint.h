#pragma once

#include "common.h"
#include "joint.h"

namespace muli3
{

// Line joint in 3D: constrains one body to slide along an axis defined on the other body
// 2 DOF constraint (constrains 2 translational DOFs perpendicular to the axis)
class LineJoint : public Joint
{
public:
    LineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxis;

    Vec3 t1, t2; // Two perpendicular vectors to the sliding axis
    Vec3 sa1, sa2;
    Vec3 sb1, sb2;

    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(const Vec2& lambda);
};

inline const Vec3& LineJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& LineJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

} // namespace muli3
