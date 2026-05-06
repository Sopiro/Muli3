#include "muli3/grab_joint.h"

namespace muli3
{

GrabJoint::GrabJoint(
    RigidBody* body,
    const Vec3& anchor,
    const Vec3& targetPosition,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(grab_joint, body, body, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchor = MulT(body->GetTransform(), anchor);
    target = targetPosition;
}

void GrabJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [I, skew(r)]
    // W = (J · M^-1 · J^t)^-1

    r = bodyA->GetRotation().Rotate(localAnchor - bodyA->GetLocalCenter());
    Vec3 p = bodyA->motion.c + r;

    Mat3 skewR = Skew(r);
    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = Mat3(bodyA->invMass) + skewR.GetTranspose() * invIA * skewR;

    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Vec3 error = p - target;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void GrabJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec3 jv = bodyA->linearVelocity + Cross(bodyA->angularVelocity, r);

    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void GrabJoint::ApplyImpulse(const Vec3& lambda)
{
    bodyA->linearVelocity += lambda * bodyA->invMass;
    bodyA->angularVelocity += bodyA->GetWorldInverseInertiaTensor() * Cross(r, lambda);
}

} // namespace muli3
