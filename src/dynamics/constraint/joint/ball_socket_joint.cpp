#include "muli3/ball_socket_joint.h"

namespace muli3
{

BallSocketJoint::BallSocketJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
    : Joint(ball_socket_joint, bodyA, bodyB, frequency, dampingRatio)
    , impulseSum{ 0.0f, 0.0f, 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
}

void BallSocketJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // C = pb - pa keeps the two anchor points coincident. Since the velocity
    // of an offset point is v + w x r, differentiating C gives
    // J = [-I, skew(ra), I, -skew(rb)] for V = [vA, wA, vB, wB].

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    // K = J * M^-1 * J^T is the inverse effective mass at the anchors.
    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // clang-format off
    Mat3 k = Mat3(sA->invMass + sB->invMass)
           + skewRA.GetTranspose() * s->invIA * skewRA
           + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(&beta, &gamma, k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;

    Vec3 error = pb - pa;
    bias = error * beta * step.inv_dt;
}

void BallSocketJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void BallSocketJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Solve K * lambda = -(J * V + bias + gamma * impulseSum).
    Vec3 jv = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    // This is an equality constraint, so lambda need no clamping.
    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void BallSocketJoint::ApplyImpulse(const Vec3& lambda)
{
    // Apply the generalized impulse J^T * lambda.

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (!bodyA->IsStatic())
    {
        sA->linearVelocity -= lambda * sA->invMass;
        sA->angularVelocity -= s->invIA * Cross(ra, lambda);
    }
    if (!bodyB->IsStatic())
    {
        sB->linearVelocity += lambda * sB->invMass;
        sB->angularVelocity += s->invIB * Cross(rb, lambda);
    }
}

} // namespace muli3
