#include "muli3/ball_socket_joint.h"

namespace muli3
{

BallSocketJoint::BallSocketJoint(RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
    : Joint(ball_socket_joint, bodyA, bodyB, frequency, dampingRatio)
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
}

void BallSocketJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Compute Jacobian J and effective mass W
    // J = [-I, -skew(ra), I, skew(rb)]
    // W = (J * M^-1 * J^t)^-1

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    // K = Ma^-1 + Mb^-1 + skew(ra)^T * Ia^-1 * skew(ra) + skew(rb)^T * Ib^-1 * skew(rb)
    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // clang-format off
    Mat3 k = Mat3(sA->invMass + sB->invMass)
           + skewRA.GetTranspose() * s->invIA * skewRA
           + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += s->gamma;
    k.ey.y += s->gamma;
    k.ez.z += s->gamma;

    m = k.GetInverse();

    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;

    Vec3 error = pb - pa;
    bias = error * s->beta * step.inv_dt;
}

void BallSocketJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void BallSocketJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Compute corrective impulse: Pc
    // Pc = J^t * lambda
    // lambda = (J * M^-1 * J^t)^-1 * -(J*v+b)

    Vec3 jv = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    // You don't have to clamp the impulse. It's equality constraint!
    Vec3 lambda = m * -(jv + bias + impulseSum * s->gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void BallSocketJoint::ApplyImpulse(const Vec3& lambda)
{
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    sA->linearVelocity -= lambda * sA->invMass;
    sA->angularVelocity -= s->invIA * Cross(ra, lambda);
    sB->linearVelocity += lambda * sB->invMass;
    sB->angularVelocity += s->invIB * Cross(rb, lambda);
}

} // namespace muli3
