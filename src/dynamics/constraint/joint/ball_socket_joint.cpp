#include "muli3/ball_socket_joint.h"

namespace muli3
{

BallSocketJoint::BallSocketJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
    : Joint(ball_socket_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
}

void BallSocketJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-I, -skew(ra), I, skew(rb)]
    // W = (J * M^-1 * J^t)^-1

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    // K = Ma^-1 + Mb^-1 + skew(ra)^T * Ia^-1 * skew(ra) + skew(rb)^T * Ib^-1 * skew(rb)
    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    // clang-format off
    Mat3 k = Mat3(bodyA->invMass + bodyB->invMass)
           + skewRA.GetTranspose() * invIA * skewRA
           + skewRB.GetTranspose() * invIB * skewRB;
    // clang-format on

    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Vec3 pa = bodyA->motion.c + ra;
    Vec3 pb = bodyB->motion.c + rb;

    Vec3 error = pb - pa;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void BallSocketJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t * lambda
    // lambda = (J * M^-1 * J^t)^-1 * -(J*v+b)

    Vec3 jv =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) - (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));

    // You don't have to clamp the impulse. It's equality constraint!
    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void BallSocketJoint::ApplyImpulse(const Vec3& lambda)
{
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda

    bodyA->linearVelocity -= lambda * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->GetWorldInverseInertiaTensor() * Cross(ra, lambda);
    bodyB->linearVelocity += lambda * bodyB->invMass;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * Cross(rb, lambda);
}

} // namespace muli3
