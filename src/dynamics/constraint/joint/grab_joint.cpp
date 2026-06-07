#include "muli3/grab_joint.h"

namespace muli3
{

GrabJoint::GrabJoint(Body* body, const Vec3& anchor, const Vec3& targetPosition, float frequency, float dampingRatio)
    : Joint(grab_joint, body, body, frequency, dampingRatio)
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchor = MulT(body->GetTransform(), anchor);
    target = targetPosition;
}

void GrabJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    // Compute Jacobian J and effective mass W
    // J = [I, skew(r)]
    // W = (J · M^-1 · J^t)^-1

    r = bodyA->GetRotation().Rotate(localAnchor - bodyA->GetLocalCenter());
    Vec3 p = sA->motion.c + r;

    Mat3 skewR = Skew(r);
    s->invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = Mat3(sA->invMass) + skewR.GetTranspose() * s->invIA * skewR;

    ComputeBetaAndGamma(k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += s->gamma;
    k.ey.y += s->gamma;
    k.ez.z += s->gamma;

    m = k.GetInverse();

    Vec3 error = p - target;
    bias = error * s->beta * step.inv_dt;
}

void GrabJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 * -(J·v+b)

    Vec3 jv = sA->linearVelocity + Cross(sA->angularVelocity, r);

    Vec3 lambda = m * -(jv + bias + impulseSum * s->gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void GrabJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void GrabJoint::ApplyImpulse(const Vec3& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    if (!bodyA->IsStatic())
    {
        sA->linearVelocity += lambda * sA->invMass;
        sA->angularVelocity += s->invIA * Cross(r, lambda);
    }
}

} // namespace muli3
