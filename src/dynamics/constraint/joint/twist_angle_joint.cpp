#include "muli3/twist_angle_joint.h"
#include "muli3/frame.h"

namespace muli3
{

enum
{
    twist_limit_inactive,
    twist_limit_at_lower,
    twist_limit_at_upper,
    twist_limit_equal,
};

static float NormalizeAngle(float angle)
{
    while (angle > pi)
    {
        angle -= two_pi;
    }

    while (angle < -pi)
    {
        angle += two_pi;
    }

    return angle;
}

static float ClampImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case twist_limit_at_lower:
        return Max(impulse, 0.0f);
    case twist_limit_at_upper:
        return Min(impulse, 0.0f);
    case twist_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

// Rotate v by the shortest rotation that takes from to to.
// This is Rodrigues' formula written directly to avoid building a temporary quaternion.
static Vec3 RotateBetweenUnitVectors(const Vec3& from, const Vec3& to, const Vec3& v)
{
    float c = Clamp(Dot(from, to), -1.0f, 1.0f);

    if (c > 1.0f - epsilon)
    {
        return v;
    }

    Vec3 axis = Cross(from, to);
    float s = axis.Normalize();

    if (s == 0.0f)
    {
        CoordinateSystem(from, &axis);
        return -v + axis * (2.0f * Dot(axis, v));
    }

    return v * c + Cross(axis, v) * s + axis * (Dot(axis, v) * (1.0f - c));
}

static float GetTwistAngle(const Vec3& frameX, const Vec3& frameY, const Vec3& frameZ, const Vec3& axisB, const Vec3& refAxisB)
{
    Vec3 alignedRefAxisB = RotateBetweenUnitVectors(axisB, frameZ, refAxisB);
    Vec3 projected = GramSchmidt(alignedRefAxisB, frameZ);
    if (projected.Normalize() == 0.0f)
    {
        projected = frameX;
    }

    float x = Dot(projected, frameX);
    float y = Dot(projected, frameY);

    return std::atan2(y, x);
}

TwistAngleJoint::TwistAngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& worldAxis,
    float jointMinAngle,
    float jointMaxAngle,
    float jointFrequency,
    float jointDampingRatio
)
    : Joint(twist_angle_joint, bodyA, bodyB, jointFrequency, jointDampingRatio)
    , angleOffset{ 0.0f }
    , minAngle{ jointMinAngle }
    , maxAngle{ jointMaxAngle }
    , currentAngle{ 0.0f }
    , angleM{ 0.0f }
    , angleBias{ 0.0f }
    , angleImpulseSum{ 0.0f }
    , limitState{ twist_limit_inactive }
{
    Vec3 axis = Length2(worldAxis) > epsilon ? Normalize(worldAxis) : y_axis;
    Vec3 normalAxis;
    CoordinateSystem(axis, &normalAxis);

    // Store one common twist axis and one perpendicular reference axis on each body.
    // The signed relative angle is measured by projecting B's reference axis onto A's twist plane.
    localAxisA = bodyA->GetRotation().RotateInv(axis);
    localAxisB = bodyB->GetRotation().RotateInv(axis);
    localNormalAxisA = bodyA->GetRotation().RotateInv(normalAxis);
    localNormalAxisB = bodyB->GetRotation().RotateInv(normalAxis);

    maxAngle = Max(minAngle, maxAngle);
}

void TwistAngleJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localNormalAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localNormalAxisB);
    Vec3 binormalA = Cross(axisA, refAxisA);

    if (binormalA.Normalize() == 0.0f)
    {
        CoordinateSystem(axisA, &binormalA);
    }

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Use the average of both transformed axes when possible.
    // This keeps the correction symmetric while still measuring the angle in A's frame.
    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0.0f)
    {
        twistAxis = axisA;
    }

    float angleK = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis);

    ComputeBetaAndGamma(angleK > 0.0f ? 1.0f / angleK : 0.0f, step.dt);

    angleK += s->gamma;
    angleM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    currentAngle = GetTwistAngle(refAxisA, binormalA, axisA, axisB, refAxisB) - angleOffset;

    if (minAngle == maxAngle)
    {
        limitState = twist_limit_equal;
        angleBias = Clamp(NormalizeAngle(currentAngle - minAngle), -max_joint_angular_correction, max_joint_angular_correction) *
                    s->beta * step.inv_dt;
    }
    else if (maxAngle - minAngle >= two_pi)
    {
        limitState = twist_limit_inactive;
        angleBias = 0.0f;
    }
    else
    {
        float center = 0.5f * (minAngle + maxAngle);
        float shift = two_pi * std::round((currentAngle - center) / two_pi);
        float lower = minAngle + shift;
        float upper = maxAngle + shift;

        if (currentAngle < lower - angular_slop)
        {
            limitState = twist_limit_at_lower;
            angleBias = Max(currentAngle - (lower - angular_slop), -max_joint_angular_correction) * s->beta * step.inv_dt;
        }
        else if (currentAngle > upper + angular_slop)
        {
            limitState = twist_limit_at_upper;
            angleBias = Min(currentAngle - (upper + angular_slop), max_joint_angular_correction) * s->beta * step.inv_dt;
        }
        else
        {
            limitState = twist_limit_inactive;
            angleBias = 0.0f;
        }
    }

    angleImpulseSum = ClampImpulse(angleImpulseSum, limitState);
}

void TwistAngleJoint::WarmStart()
{
    if (limitState != twist_limit_inactive)
    {
        ApplyAngleImpulse(angleImpulseSum);
    }
}

void TwistAngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (limitState == twist_limit_inactive)
    {
        return;
    }

    float angleJV = Dot(twistAxis, sB->angularVelocity - sA->angularVelocity);
    float lambda = angleM * -(angleJV + angleBias + angleImpulseSum * s->gamma);

    float newImpulseSum;
    if (limitState == twist_limit_equal)
    {
        newImpulseSum = angleImpulseSum + lambda;
    }
    else
    {
        newImpulseSum = ClampImpulse(angleImpulseSum + lambda, limitState);
    }

    lambda = newImpulseSum - angleImpulseSum;
    angleImpulseSum = newImpulseSum;

    ApplyAngleImpulse(lambda);
}

void TwistAngleJoint::ApplyAngleImpulse(float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = twistAxis * lambda;

    sA->angularVelocity -= s->invIA * p;
    sB->angularVelocity += s->invIB * p;
}

} // namespace muli3
