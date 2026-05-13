#include "muli3/distance_joint.h"

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
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float jointMinLength,
    float jointMaxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(distance_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
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
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-d, -d×ra, d, d×rb] ( d = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // W = (J · M^-1 · J^t)^-1

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    Vec3 pa = bodyA->motion.c + ra;
    Vec3 pb = bodyB->motion.c + rb;

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

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    Vec3 crossDA = Cross(d, ra);
    Vec3 crossDB = Cross(d, rb);

    // clang-format off
    float k = bodyA->invMass + bodyB->invMass
            + Dot(crossDA, invIA * crossDA)
            + Dot(crossDB, invIB * crossDB)
            + gamma;
    // clang-format on

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

    if (step.warm_starting && limitState != distance_limit_inactive)
    {
        ApplyImpulse(impulseSum);
    }
}

void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    if (limitState == distance_limit_inactive)
    {
        return;
    }

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
                (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)),
            d);

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
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec3 p = d * lambda;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->GetWorldInverseInertiaTensor() * Cross(ra, p);
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * Cross(rb, p);
}

} // namespace muli3
