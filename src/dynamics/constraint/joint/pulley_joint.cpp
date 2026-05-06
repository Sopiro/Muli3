#include "muli3/pulley_joint.h"

namespace muli3
{

PulleyJoint::PulleyJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchorA, const Vec3& anchorB,
    const Vec3& inGroundAnchorA, const Vec3& inGroundAnchorB, float pulleyRatio,
    float jointFrequency, float jointDampingRatio, float jointMass
)
    : Joint(pulley_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    groundAnchorA = inGroundAnchorA;
    groundAnchorB = inGroundAnchorB;

    ratio = pulleyRatio;
    length = Length(anchorA - groundAnchorA) + Length(anchorB - groundAnchorB);
}

void PulleyJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    ua = (bodyA->motion.c + ra) - groundAnchorA;
    ub = (bodyB->motion.c + rb) - groundAnchorB;

    float lengthA = Length(ua);
    float lengthB = Length(ub);

    if (lengthA > linear_slop)
    {
        ua *= 1.0f / lengthA;
    }
    else
    {
        ua = Vec3::zero;
    }

    if (lengthB > linear_slop)
    {
        ub *= 1.0f / lengthB;
    }
    else
    {
        ub = Vec3::zero;
    }

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    Vec3 rua = Cross(ra, ua);
    Vec3 rub = Cross(rb, ub);

    // clang-format off
    float k = bodyA->invMass + Dot(rua, invIA * rua)
            + (bodyB->invMass + Dot(rub, invIB * rub)) * ratio * ratio
            + gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = length - (lengthA + lengthB);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void PulleyJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    float jv =
        -(ratio * Dot(ub, bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) +
          Dot(ua, bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)));

    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void PulleyJoint::ApplyImpulse(float lambda)
{
    Vec3 pa = -lambda * ua;
    Vec3 pb = -ratio * lambda * ub;

    bodyA->linearVelocity += pa * bodyA->invMass;
    bodyA->angularVelocity += bodyA->GetWorldInverseInertiaTensor() * Cross(ra, pa);
    bodyB->linearVelocity += pb * bodyB->invMass;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * Cross(rb, pb);
}

} // namespace muli3
