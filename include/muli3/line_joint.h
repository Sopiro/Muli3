#pragma once

#include "joint.h"

namespace muli3
{

// Line constraint: constrains one anchor point to slide along an axis
// 2 DOF constraint (constrains 2 translational DOFs perpendicular to the axis)
class LineJoint : public Joint
{
public:
    LineJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

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
    float beta;
    float gamma;

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
