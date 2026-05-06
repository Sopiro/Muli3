#include "muli3/distance_joint.h"

namespace muli3
{

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
    , impulseSum{ 0.0f }
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
        d = Vec3{ 0.0f, 1.0f, 0.0f };
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

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    Vec2 error(currentLength - minLength, currentLength - maxLength);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        if (minLength == maxLength)
        {
            ApplyImpulse(impulseSum[0]);
        }
        else
        {
            if (bias[0] < 0)
            {
                ApplyImpulse(impulseSum[0]);
            }
            if (bias[1] > 0)
            {
                ApplyImpulse(impulseSum[1]);
            }
        }
    }
}

void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
                (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)),
            d);

    if (minLength == maxLength)
    {
        // You don't have to clamp the impulse because it's equality constraint!
        float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
        ApplyImpulse(lambda);
        impulseSum[0] += lambda;
    }
    else
    {
        if (bias[0] < 0)
        {
            float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
            ApplyImpulse(lambda);
            impulseSum[0] += lambda;
        }

        if (bias[1] > 0)
        {
            float lambda = m * -(jv + bias[1] + impulseSum[1] * gamma);
            ApplyImpulse(lambda);
            impulseSum[1] += lambda;
        }
    }
}

void DistanceJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec3 p = d * lambda;

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= invIA * Cross(ra, p);
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += invIB * Cross(rb, p);
}

} // namespace muli3
