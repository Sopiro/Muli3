#include "muli3/fixed_rotation_joint.h"

namespace muli3
{

FixedRotationJoint::FixedRotationJoint(RigidBody* body, float jointFrequency, float jointDampingRatio, float jointMass)
    : Joint(fixed_rotation_joint, body, body, jointFrequency, jointDampingRatio, jointMass)
    , targetOrientation{ body->GetRotation() }
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
}

void FixedRotationJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = invIA;
    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Quat qError = bodyA->motion.q * targetOrientation.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    bias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void FixedRotationJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    Vec3 jv = bodyA->angularVelocity;
    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void FixedRotationJoint::ApplyImpulse(const Vec3& lambda)
{
    bodyA->angularVelocity += invIA * lambda;
}

} // namespace muli3
