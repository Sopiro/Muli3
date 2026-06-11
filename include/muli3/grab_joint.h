#pragma once

#include "joint.h"

namespace muli3
{

// Point-to-point constraint against a world-space target
// 3 DOF constraint (constrains 3 translational DOFs)
class GrabJoint : public Joint
{
public:
    GrabJoint(Body* body, const Vec3& anchor, const Vec3& target, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    const Vec3& GetLocalAnchor() const;

    const Vec3& GetTarget() const;
    void SetTarget(const Vec3& newTarget);

private:
    Vec3 localAnchor;
    Vec3 target;

    Vec3 r;
    Mat3 m;

    Vec3 bias;
    Vec3 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec3& lambda);
};

inline const Vec3& GrabJoint::GetLocalAnchor() const
{
    return localAnchor;
}

inline const Vec3& GrabJoint::GetTarget() const
{
    return target;
}

inline void GrabJoint::SetTarget(const Vec3& newTarget)
{
    target = newTarget;
}

} // namespace muli3
