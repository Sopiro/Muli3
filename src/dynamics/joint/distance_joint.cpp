#include "muli3/joints.h"

namespace muli3
{

enum
{
    distance_limit_inactive,
    distance_limit_at_lower,
    distance_limit_at_upper,
    distance_limit_equal,
};

static float ClampImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case distance_limit_at_lower:
        return Max(impulse, 0.0f);
    case distance_limit_at_upper:
        return Min(impulse, 0.0f);
    case distance_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

DistanceJoint::DistanceJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float jointMinLength,
    float jointMaxLength,
    float frequency,
    float dampingRatio
)
    : Joint(distance_joint, bodyA, bodyB)
    , frequency{ Max(frequency, 0.0f) }
    , dampingRatio{ Max(dampingRatio, 0.0f) }
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
    , limitState{ distance_limit_inactive }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    minLength = jointMinLength < 0 ? Length(anchorB - anchorA) : jointMinLength;
    maxLength = jointMaxLength < 0 ? Length(anchorB - anchorA) : jointMaxLength;
    maxLength = Max(minLength, maxLength);
}

void DistanceJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;

    d = pb - pa;
    float currentLength = Length(d);
    if (currentLength > epsilon)
    {
        d *= 1.0f / currentLength;
    }
    else
    {
        d = x_axis;
    }

    // C = |pb - pa| - L. Its gradient uses d = (pb - pa) / |pb - pa|,
    // so Cdot = dot(d, vb + wb x rb - va - wa x ra) and
    // J = [-d, d x ra, d, -(d x rb)].

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    Vec3 crossDA = Cross(d, ra);
    Vec3 crossDB = Cross(d, rb);

    // clang-format off
    float k = sA->invMass + sB->invMass
            + Dot(crossDA, s->invIA * crossDA)
            + Dot(crossDB, s->invIB * crossDB);
    // clang-format on

    ComputeBetaAndGamma(&beta, &gamma, frequency, dampingRatio, k != 0.0f ? 1.0f / k : 0.0f, step.dt);

    k += gamma;
    m = k != 0.0f ? 1.0f / k : 0.0f;

    if (minLength == maxLength)
    {
        limitState = distance_limit_equal;
        bias = (currentLength - minLength) * beta * step.inv_dt;
    }
    else if (currentLength < minLength)
    {
        limitState = distance_limit_at_lower;
        bias = (currentLength - minLength) * beta * step.inv_dt;
    }
    else if (currentLength > maxLength)
    {
        limitState = distance_limit_at_upper;
        bias = (currentLength - maxLength) * beta * step.inv_dt;
    }
    else
    {
        limitState = distance_limit_inactive;
        bias = 0.0f;
    }

    impulseSum = ClampImpulse(impulseSum, limitState);
}

void DistanceJoint::WarmStart()
{
    if (limitState != distance_limit_inactive)
    {
        ApplyImpulse(impulseSum);
    }
}

void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (limitState == distance_limit_inactive)
    {
        return;
    }

    // Solve the scalar constraint along d. Limit states clamp the accumulated
    // impulse so a lower limit can only push and an upper limit can only pull.

    float jv =
        Dot((sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra)), d);

    float lambda = m * -(jv + bias + impulseSum * gamma);

    float newImpulseSum;
    if (limitState == distance_limit_equal)
    {
        newImpulseSum = impulseSum + lambda;
    }
    else
    {
        newImpulseSum = ClampImpulse(impulseSum + lambda, limitState);
    }

    lambda = newImpulseSum - impulseSum;
    impulseSum = newImpulseSum;

    ApplyImpulse(lambda);
}

void DistanceJoint::ApplyImpulse(float lambda)
{
    // Apply the generalized impulse J^T * lambda.

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = d * lambda;

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity -= p * sA->invMass;
        sA->angularVelocity -= s->invIA * Cross(ra, p);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += p * sB->invMass;
        sB->angularVelocity += s->invIB * Cross(rb, p);
    }
}

const Vec3& DistanceJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const Vec3& DistanceJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

float DistanceJoint::GetJointLength() const
{
    return minLength;
}

void DistanceJoint::SetJointLength(float newLength)
{
    minLength = Max(newLength, 0.0f);
    maxLength = minLength;
}

float DistanceJoint::GetJointMinLength() const
{
    return minLength;
}

void DistanceJoint::SetJointMinLength(float newMinLength)
{
    minLength = Max(newMinLength, 0.0f);
    maxLength = Max(minLength, maxLength);
}

float DistanceJoint::GetJointMaxLength() const
{
    return maxLength;
}

void DistanceJoint::SetJointMaxLength(float newMaxLength)
{
    maxLength = Max(newMaxLength, 0.0f);
    minLength = Min(minLength, maxLength);
}

float DistanceJoint::GetFrequency() const
{
    return frequency;
}

void DistanceJoint::SetFrequency(float newFrequency)
{
    frequency = Max(newFrequency, 0.0f);
}

float DistanceJoint::GetDampingRatio() const
{
    return dampingRatio;
}

void DistanceJoint::SetDampingRatio(float newDampingRatio)
{
    dampingRatio = Max(newDampingRatio, 0.0f);
}

} // namespace muli3
