#include "muli3/frame.h"
#include "muli3/joints.h"

namespace muli3
{

enum
{
    swing_limit_inactive,
    swing_limit_at_upper,
};

static float ClampImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case swing_limit_at_upper:
        return Min(impulse, 0.0f);
    default:
        return 0.0f;
    }
}

SwingAngleJoint::SwingAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& worldAxis, float jointMaxAngle, float frequency, float dampingRatio
)
    : Joint(swing_angle_joint, bodyA, bodyB)
    , frequency{ Max(frequency, 0.0f) }
    , dampingRatio{ Max(dampingRatio, 0.0f) }
    , maxAngle{ Clamp(jointMaxAngle, 0.0f, pi) }
    , currentAngle{ 0.0f }
    , frictionAxis1{ 0.0f, 0.0f, 0.0f }
    , frictionAxis2{ 0.0f, 0.0f, 0.0f }
    , frictionM{ 0.0f }
    , frictionImpulseSum{ 0.0f, 0.0f }
    , maxFrictionTorque{ 0.0f }
    , m{ 0.0f }
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
    , limitState{ swing_limit_inactive }
{
    Vec3 axis = Length2(worldAxis) > epsilon ? Normalize(worldAxis) : y_axis;
    Vec3 normalAxis;
    CoordinateSystem(axis, &normalAxis);

    localAxisA = bodyA->GetRotation().RotateInv(axis);
    localAxisB = bodyB->GetRotation().RotateInv(axis);
    localNormalAxisB = bodyB->GetRotation().RotateInv(normalAxis);
}

void SwingAngleJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);

    float dot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    currentAngle = std::acos(dot);

    Vec3 axis = Cross(axisA, axisB);
    float axisLength = axis.Normalize();

    if (currentAngle > maxAngle + angular_slop)
    {
        limitState = swing_limit_at_upper;
    }
    else
    {
        limitState = swing_limit_inactive;
    }

    if (limitState == swing_limit_inactive && maxFrictionTorque == 0.0f)
    {
        bias = 0.0f;
        impulseSum = 0.0f;
        frictionImpulseSum = Vec2::zero;
        swingAxis = Vec3::zero;
        m = 0.0f;
        return;
    }

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    if (maxFrictionTorque > 0.0f)
    {
        frictionAxis1 = bodyB->GetRotation().Rotate(localNormalAxisB);
        frictionAxis2 = Cross(axisB, frictionAxis1);

        float k11 = Dot(frictionAxis1, s->invIA * frictionAxis1) + Dot(frictionAxis1, s->invIB * frictionAxis1);
        float k12 = Dot(frictionAxis1, s->invIA * frictionAxis2) + Dot(frictionAxis1, s->invIB * frictionAxis2);
        float k22 = Dot(frictionAxis2, s->invIA * frictionAxis2) + Dot(frictionAxis2, s->invIB * frictionAxis2);
        Mat2 K = Mat2{ Vec2{ k11, k12 }, Vec2{ k12, k22 } };
        frictionM = K.GetInverse();

        float maxImpulse = maxFrictionTorque * step.dt;
        if (Length2(frictionImpulseSum) > maxImpulse * maxImpulse)
        {
            frictionImpulseSum = Normalize(frictionImpulseSum) * maxImpulse;
        }
    }
    else
    {
        frictionImpulseSum = Vec2::zero;
    }

    if (limitState == swing_limit_inactive)
    {
        bias = 0.0f;
        impulseSum = 0.0f;
        swingAxis = Vec3::zero;
        m = 0.0f;
        return;
    }

    // C = angle(axisA, axisB) - maxAngle. A relative angular velocity around
    // normalize(axisA x axisB) changes this angle directly, so
    // J = [0, -swingAxis, 0, swingAxis].
    if (axisLength == 0.0f)
    {
        CoordinateSystem(axisA, &swingAxis);
    }
    else
    {
        swingAxis = axis;
    }

    // K = J * M^-1 * J^T for the one-dimensional angular constraint.
    float k = Dot(swingAxis, s->invIA * swingAxis) + Dot(swingAxis, s->invIB * swingAxis);
    ComputeBetaAndGamma(&beta, &gamma, frequency, dampingRatio, k > 0.0f ? 1.0f / k : 0.0f, step.dt);

    k += gamma;
    m = k != 0.0f ? 1.0f / k : 0.0f;

    float error = Min(currentAngle - (maxAngle + angular_slop), max_joint_angular_correction);
    bias = error * beta * step.inv_dt;
    impulseSum = ClampImpulse(impulseSum, limitState);
}

void SwingAngleJoint::WarmStart()
{
    ApplyFrictionImpulse(frictionImpulseSum);
    ApplyImpulse(impulseSum);
}

void SwingAngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (maxFrictionTorque > 0.0f)
    {
        Vec3 relativeAngularVelocity = sB->angularVelocity - sA->angularVelocity;
        Vec2 jv{ Dot(frictionAxis1, relativeAngularVelocity), Dot(frictionAxis2, relativeAngularVelocity) };
        Vec2 lambda = Mul(frictionM, -jv);
        Vec2 newImpulseSum = frictionImpulseSum + lambda;
        float maxImpulse = maxFrictionTorque * step.dt;

        if (Length2(newImpulseSum) > maxImpulse * maxImpulse)
        {
            newImpulseSum = Normalize(newImpulseSum) * maxImpulse;
        }

        lambda = newImpulseSum - frictionImpulseSum;
        frictionImpulseSum = newImpulseSum;
        ApplyFrictionImpulse(lambda);
    }

    if (limitState == swing_limit_inactive)
    {
        return;
    }

    // The upper limit is unilateral. A non-positive impulse can only reduce
    // the separation angle, never push the axes farther apart.
    float jv = Dot(swingAxis, sB->angularVelocity - sA->angularVelocity);
    float lambda = m * -(jv + bias + impulseSum * gamma);
    float newImpulseSum = ClampImpulse(impulseSum + lambda, limitState);

    lambda = newImpulseSum - impulseSum;
    impulseSum = newImpulseSum;

    ApplyImpulse(lambda);
}

void SwingAngleJoint::ApplyFrictionImpulse(const Vec2& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 impulse = frictionAxis1 * lambda.x + frictionAxis2 * lambda.y;

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * impulse;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * impulse;
    }
}

void SwingAngleJoint::ApplyImpulse(float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = swingAxis * lambda;

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * p;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * p;
    }
}

const Vec3& SwingAngleJoint::GetLocalAxisA() const
{
    return localAxisA;
}

const Vec3& SwingAngleJoint::GetLocalAxisB() const
{
    return localAxisB;
}

float SwingAngleJoint::GetJointAngle() const
{
    return currentAngle;
}

float SwingAngleJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

void SwingAngleJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = std::clamp(newMaxAngle, 0.0f, pi);
}

float SwingAngleJoint::GetMaxFrictionTorque() const
{
    return maxFrictionTorque;
}

void SwingAngleJoint::SetMaxFrictionTorque(float torque)
{
    maxFrictionTorque = Max(torque, 0.0f);
    if (maxFrictionTorque == 0.0f)
    {
        frictionImpulseSum = Vec2::zero;
    }
}

float SwingAngleJoint::GetFrequency() const
{
    return frequency;
}

void SwingAngleJoint::SetFrequency(float newFrequency)
{
    frequency = Max(newFrequency, 0.0f);
}

float SwingAngleJoint::GetDampingRatio() const
{
    return dampingRatio;
}

void SwingAngleJoint::SetDampingRatio(float newDampingRatio)
{
    dampingRatio = Max(newDampingRatio, 0.0f);
}

} // namespace muli3
