#pragma once

#include "joint.h"

namespace muli3
{

// Point-to-point constraint: constrains two anchor points to coincide
// 3 DOF constraint (constrains 3 translational DOFs)
class BallSocketJoint : public Joint
{
public:
    BallSocketJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

private:
    Vec3 localAnchorA;
    Vec3 localAnchorB;

    Vec3 ra;
    Vec3 rb;
    Mat3 m;

    Vec3 bias;
    Vec3 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec3& lambda);
};

inline const Vec3& BallSocketJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec3& BallSocketJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

} // namespace muli3
