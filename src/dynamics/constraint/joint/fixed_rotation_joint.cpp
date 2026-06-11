#include "muli3/fixed_rotation_joint.h"

namespace muli3
{

FixedRotationJoint::FixedRotationJoint(Body* body, float frequency, float dampingRatio)
    : Joint(fixed_rotation_joint, body, body, frequency, dampingRatio)
    , targetOrientation{ body->GetRotation() }
    , impulseSum{ 0.0f, 0.0f, 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
}

void FixedRotationJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    s->invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = s->invIA;
    ComputeBetaAndGamma(&beta, &gamma, k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Quat qError = sA->motion.q * targetOrientation.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    bias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * beta * step.inv_dt;
}

void FixedRotationJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void FixedRotationJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();

    Vec3 jv = sA->angularVelocity;
    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void FixedRotationJoint::ApplyImpulse(const Vec3& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    if (!bodyA->IsStatic())
    {
        sA->angularVelocity += s->invIA * lambda;
    }
}

} // namespace muli3
