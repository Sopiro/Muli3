#include "muli3/pulley_joint.h"

namespace muli3
{

PulleyJoint::PulleyJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    const Vec3& inGroundAnchorA,
    const Vec3& inGroundAnchorB,
    float pulleyRatio,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
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

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    ua = (sA->motion.c + ra) - groundAnchorA;
    ub = (sB->motion.c + rb) - groundAnchorB;

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

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    Vec3 rua = Cross(ra, ua);
    Vec3 rub = Cross(rb, ub);

    // clang-format off
    float k = sA->invMass + Dot(rua, s->invIA * rua)
            + (sB->invMass + Dot(rub, s->invIB * rub)) * ratio * ratio
            + s->gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = length - (lengthA + lengthB);
    bias = error * s->beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void PulleyJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    float jv =
        -(ratio * Dot(ub, sB->linearVelocity + Cross(sB->angularVelocity, rb)) +
          Dot(ua, sA->linearVelocity + Cross(sA->angularVelocity, ra)));

    float lambda = m * -(jv + bias + impulseSum * s->gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void PulleyJoint::ApplyImpulse(float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 pa = -lambda * ua;
    Vec3 pb = -ratio * lambda * ub;

    sA->linearVelocity += pa * sA->invMass;
    sA->angularVelocity += s->invIA * Cross(ra, pa);
    sB->linearVelocity += pb * sB->invMass;
    sB->angularVelocity += s->invIB * Cross(rb, pb);
}

} // namespace muli3
