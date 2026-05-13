#pragma once

#include "joint.h"

namespace muli3
{

// Orientation constraint against a world-space target orientation
// 3 DOF constraint (constrains 3 rotational DOFs)
class FixedRotationJoint : public Joint
{
public:
    FixedRotationJoint(RigidBody* body, float frequency, float dampingRatio, float jointMass);

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Quat& GetTargetOrientation() const;
    void SetTargetOrientation(const Quat& newTargetOrientation);

private:
    Quat targetOrientation;

    Mat3 m;
    Vec3 bias;
    Vec3 impulseSum;

    void ApplyImpulse(const Vec3& lambda);
};

inline const Quat& FixedRotationJoint::GetTargetOrientation() const
{
    return targetOrientation;
}

inline void FixedRotationJoint::SetTargetOrientation(const Quat& newTargetOrientation)
{
    targetOrientation = newTargetOrientation;
}

} // namespace muli3
