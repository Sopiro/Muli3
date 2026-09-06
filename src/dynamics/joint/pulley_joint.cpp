#include "muli3/joints.h"

namespace muli3
{

PulleyJoint::PulleyJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    const Vec3& inGroundAnchorA,
    const Vec3& inGroundAnchorB,
    float pulleyRatio,
    float frequency,
    float dampingRatio
)
    : Joint(pulley_joint, bodyA, bodyB)
    , frequency{ Max(frequency, 0.0f) }
    , dampingRatio{ Max(dampingRatio, 0.0f) }
    , m{ 0.0f }
    , impulseSum{ 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    groundAnchorA = inGroundAnchorA;
    groundAnchorB = inGroundAnchorB;

    ratio = Max(pulleyRatio, 1e-3f);
    length = Length(anchorA - groundAnchorA) + ratio * Length(anchorB - groundAnchorB);
}

void PulleyJoint::Prepare(const Timestep& step)
{
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

    // C = L - (lengthA + ratio * lengthB).
    // Differentiating the segment lengths gives
    // J = [-ua, -(ra x ua), -ratio * ub, -ratio * (rb x ub)].
    // The ratio therefore enters K quadratically for body B.

    float k = sA->invMass + Dot(rua, s->invIA * rua) + (sB->invMass + Dot(rub, s->invIB * rub)) * ratio * ratio;

    ComputeBetaAndGamma(&beta, &gamma, frequency, dampingRatio, k > 0.0f ? 1.0f / k : 0.0f, step.dt);
    k += gamma;

    m = k != 0.0f ? 1.0f / k : 0.0f;

    float error = length - (lengthA + ratio * lengthB);
    bias = error * beta * step.inv_dt;
}

void PulleyJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void PulleyJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // J * V is the negative rate of change of the weighted rope length.
    float jv =
        -(ratio * Dot(ub, sB->linearVelocity + Cross(sB->angularVelocity, rb)) +
          Dot(ua, sA->linearVelocity + Cross(sA->angularVelocity, ra)));

    float lambda = m * -(jv + bias + impulseSum * gamma);

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

    // pa and pb are the linear parts of J^T * lambda. Their moments about
    // each center of mass produce the corresponding angular impulses.
    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity += pa * sA->invMass;
        sA->angularVelocity += s->invIA * Cross(ra, pa);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += pb * sB->invMass;
        sB->angularVelocity += s->invIB * Cross(rb, pb);
    }
}

const Vec3& PulleyJoint::GetGroundAnchorA() const
{
    return groundAnchorA;
}

const Vec3& PulleyJoint::GetGroundAnchorB() const
{
    return groundAnchorB;
}

const Vec3& PulleyJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const Vec3& PulleyJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

float PulleyJoint::GetPulleyLength() const
{
    return length;
}

void PulleyJoint::SetPulleyLength(float newLength)
{
    length = Max(newLength, 0.0f);
}

float PulleyJoint::GetFrequency() const
{
    return frequency;
}

void PulleyJoint::SetFrequency(float newFrequency)
{
    frequency = Max(newFrequency, 0.0f);
}

float PulleyJoint::GetDampingRatio() const
{
    return dampingRatio;
}

void PulleyJoint::SetDampingRatio(float newDampingRatio)
{
    dampingRatio = Max(newDampingRatio, 0.0f);
}

} // namespace muli3
