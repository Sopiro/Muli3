#include "muli3/fixed_rotation_joint.h"

namespace muli3
{

FixedRotationJoint::FixedRotationJoint(RigidBody* body, float frequency, float dampingRatio)
    : Joint(fixed_rotation_joint, body, body, frequency, dampingRatio)
    , targetOrientation{ body->GetRotation() }
    , impulseSum{ 0.0f, 0.0f, 0.0f }
{
}

void FixedRotationJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    s->invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = s->invIA;
    ComputeBetaAndGamma(k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += s->gamma;
    k.ey.y += s->gamma;
    k.ez.z += s->gamma;

    m = k.GetInverse();

    Quat qError = sA->motion.q * targetOrientation.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    bias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * s->beta * step.inv_dt;
}

void FixedRotationJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void FixedRotationJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    Vec3 jv = sA->angularVelocity;
    Vec3 lambda = m * -(jv + bias + impulseSum * s->gamma);

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
