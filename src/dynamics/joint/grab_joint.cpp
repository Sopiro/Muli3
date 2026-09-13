#include "muli3/joints.h"

namespace muli3
{

FixedPositionJoint::FixedPositionJoint(
    Body* body, const Vec3& anchor, const Vec3& targetPosition, float frequency, float dampingRatio
)
    : Joint(fixed_position_joint, body, body)
    , frequency{ Max(frequency, 0.0f) }
    , dampingRatio{ Max(dampingRatio, 0.0f) }
    , impulseSum{ 0.0f, 0.0f, 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    localAnchor = MulT(body->GetTransform(), anchor);
    target = targetPosition;
}

void FixedPositionJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    // C = p - target fixes one body point in world space. From pdot = v + w x r,
    // J = [I, -skew(r)] for V = [v, w].
    r = bodyA->GetRotation().Rotate(localAnchor - bodyA->GetLocalCenter());
    Vec3 p = sA->motion.c + r;

    Mat3 skewR = Skew(r);
    s->invIA = bodyA->GetWorldInverseInertiaTensor();

    Mat3 k = Mat3(sA->invMass) + skewR.GetTranspose() * s->invIA * skewR;

    ComputeBetaAndGamma(&beta, &gamma, frequency, dampingRatio, k.TraceInverse() / 3.0f, step.dt);

    k.ex.x += gamma;
    k.ey.y += gamma;
    k.ez.z += gamma;

    m = k.GetInverse();

    Vec3 error = p - target;
    bias = error * beta * step.inv_dt;
}

void FixedPositionJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();

    // Solve K * lambda = -(J * V + bias + gamma * impulseSum).
    Vec3 jv = sA->linearVelocity + Cross(sA->angularVelocity, r);

    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void FixedPositionJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void FixedPositionJoint::ApplyImpulse(const Vec3& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity += lambda * sA->invMass;
        sA->angularVelocity += s->invIA * Cross(r, lambda);
    }
}

const Vec3& FixedPositionJoint::GetLocalAnchor() const
{
    return localAnchor;
}

const Vec3& FixedPositionJoint::GetTarget() const
{
    return target;
}

void FixedPositionJoint::SetTarget(const Vec3& newTarget)
{
    target = newTarget;
}

float FixedPositionJoint::GetFrequency() const
{
    return frequency;
}

void FixedPositionJoint::SetFrequency(float newFrequency)
{
    frequency = Max(newFrequency, 0.0f);
}

float FixedPositionJoint::GetDampingRatio() const
{
    return dampingRatio;
}

void FixedPositionJoint::SetDampingRatio(float newDampingRatio)
{
    dampingRatio = Max(newDampingRatio, 0.0f);
}

} // namespace muli3
