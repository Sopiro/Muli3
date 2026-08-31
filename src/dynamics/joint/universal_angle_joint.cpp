#include "muli3/frame.h"
#include "muli3/joints.h"

namespace muli3
{

enum
{
    universal_limit_inactive,
    universal_limit_at_lower,
    universal_limit_at_upper,
    universal_limit_equal,
};

static float ClampLimitImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case universal_limit_at_lower:
        return Max(impulse, 0.0f);
    case universal_limit_at_upper:
        return Min(impulse, 0.0f);
    case universal_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

UniversalAngleJoint::UniversalAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& worldAxisA, const Vec3& worldAxisB, float frequency, float dampingRatio
)
    : Joint(universal_angle_joint, bodyA, bodyB, frequency, dampingRatio)
    , perpM{ 0.0f }
    , perpBias{ 0.0f }
    , perpImpulseSum{ 0.0f }
    , perpBeta{ 0.0f }
    , perpGamma{ 0.0f }
    , steeringAngle{ 0.0f }
    , steeringMotorEnabled{ false }
    , targetSteeringAngle{ 0.0f }
    , steeringFrequency{ 5.0f }
    , steeringDampingRatio{ 1.0f }
    , maxSteeringTorque{ 0.0f }
    , steeringM{ 0.0f }
    , steeringBias{ 0.0f }
    , steeringImpulseSum{ 0.0f }
    , steeringBeta{ 0.0f }
    , steeringGamma{ 0.0f }
    , steeringLimitEnabled{ false }
    , steeringMinAngle{ -pi }
    , steeringMaxAngle{ pi }
    , steeringLimitBias{ 0.0f }
    , steeringLimitImpulseSum{ 0.0f }
    , steeringLimitState{ universal_limit_inactive }
    , spinAngle{ 0.0f }
    , spinMotorEnabled{ false }
    , spinSpeed{ 0.0f }
    , maxSpinTorque{ 0.0f }
    , spinM{ 0.0f }
    , spinImpulseSum{ 0.0f }
    , spinLimitEnabled{ false }
    , spinMinAngle{ -pi }
    , spinMaxAngle{ pi }
    , spinLimitM{ 0.0f }
    , spinLimitBias{ 0.0f }
    , spinLimitImpulseSum{ 0.0f }
    , spinBeta{ 0.0f }
    , spinGamma{ 0.0f }
    , spinLimitState{ universal_limit_inactive }
{
    Vec3 axisA = Length2(worldAxisA) > epsilon ? Normalize(worldAxisA) : y_axis;
    Vec3 axisB = GramSchmidt(worldAxisB, axisA);
    if (axisB.Normalize() == 0.0f)
    {
        CoordinateSystem(axisA, &axisB);
    }

    // Axis A is the steering axis and axis B is the spin axis.
    // Store the initial opposite axis on each body to measure zero steering and zero spin.
    localAxisA = bodyA->GetRotation().RotateInv(axisA);
    localAxisB = bodyB->GetRotation().RotateInv(axisB);
    localRefAxisA = bodyA->GetRotation().RotateInv(axisB);
    localRefAxisB = bodyB->GetRotation().RotateInv(axisA);
}

void UniversalAngleJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localRefAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localRefAxisB);

    Vec3 tangentAxisA = Cross(axisA, refAxisA);
    if (tangentAxisA.Normalize() == 0.0f)
    {
        CoordinateSystem(axisA, &refAxisA, &tangentAxisA);
    }

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();
    Mat3 invInertiaSum = s->invIA + s->invIB;

    // Perpendicular-axis constraint:
    // C = dot(axisA, axisB) and Cdot = dot(wB - wA, cross(axisB, axisA)).
    perpAxis = Cross(axisB, axisA);
    float perpK = Dot(perpAxis, invInertiaSum * perpAxis);

    ComputeBetaAndGamma(&perpBeta, &perpGamma, frequency, dampingRatio, perpK > 0.0f ? 1.0f / perpK : 0.0f, step.dt);

    perpK += perpGamma;
    perpM = perpK != 0.0f ? 1.0f / perpK : 0.0f;
    perpBias = Clamp(Dot(axisA, axisB), -max_joint_angular_correction, max_joint_angular_correction) * perpBeta * step.inv_dt;

    // Measure axis B in body A's steering plane:
    // theta = atan2(dot(axisB, tangentAxisA), dot(axisB, refAxisA)).
    // Both reference axes rotate with body A, so the angle changes with wRel = wB - wA.
    float cosTheta = Dot(axisB, refAxisA);
    float sinTheta = Dot(axisB, tangentAxisA);
    float denominator = Sqr(cosTheta) + Sqr(sinTheta);
    steeringAngle = std::atan2(sinTheta, cosTheta);

    // Differentiating atan2 gives thetaDot = dot(wRel, steeringAxis), where
    // steeringAxis = cross(axisB, cosTheta * tangentAxisA - sinTheta * refAxisA) / (cosTheta^2 + sinTheta^2).
    // This equals axisA when axisA and axisB are exactly perpendicular,
    // and remains consistent with the measured angle when solver error moves axisB out of the plane.
    if (denominator > epsilon)
    {
        steeringAxis = Cross(axisB, tangentAxisA * cosTheta - refAxisA * sinTheta) / denominator;
    }
    else
    {
        steeringAxis = axisA;
    }

    float steeringK = Dot(steeringAxis, invInertiaSum * steeringAxis);
    float steeringEffectiveMass = steeringK > 0.0f ? 1.0f / steeringK : 0.0f;
    ComputeBetaAndGamma(&steeringBeta, &steeringGamma, steeringFrequency, steeringDampingRatio, steeringEffectiveMass, step.dt);

    steeringK += steeringGamma;
    steeringM = steeringK != 0.0f ? 1.0f / steeringK : 0.0f;
    steeringBias =
        Clamp(NormalizeAngle(steeringAngle - targetSteeringAngle), -max_joint_angular_correction, max_joint_angular_correction) *
        steeringBeta * step.inv_dt;

    if (!steeringMotorEnabled)
    {
        steeringImpulseSum = 0.0f;
    }

    if (!steeringLimitEnabled || steeringMaxAngle - steeringMinAngle >= two_pi)
    {
        steeringLimitState = universal_limit_inactive;
        steeringLimitBias = 0.0f;
    }
    else if (steeringMinAngle == steeringMaxAngle)
    {
        steeringLimitState = universal_limit_equal;
        steeringLimitBias =
            Clamp(NormalizeAngle(steeringAngle - steeringMinAngle), -max_joint_angular_correction, max_joint_angular_correction) *
            steeringBeta * step.inv_dt;
    }
    else
    {
        // Move the configured interval to the same 2 pi branch as the measured angle.
        float center = 0.5f * (steeringMinAngle + steeringMaxAngle);
        float shift = two_pi * std::round((steeringAngle - center) / two_pi);
        float lower = steeringMinAngle + shift;
        float upper = steeringMaxAngle + shift;

        if (steeringAngle < lower - angular_slop)
        {
            steeringLimitState = universal_limit_at_lower;
            steeringLimitBias =
                Max(steeringAngle - (lower - angular_slop), -max_joint_angular_correction) * steeringBeta * step.inv_dt;
        }
        else if (steeringAngle > upper + angular_slop)
        {
            steeringLimitState = universal_limit_at_upper;
            steeringLimitBias =
                Min(steeringAngle - (upper + angular_slop), max_joint_angular_correction) * steeringBeta * step.inv_dt;
        }
        else
        {
            steeringLimitState = universal_limit_inactive;
            steeringLimitBias = 0.0f;
        }
    }

    steeringLimitImpulseSum = ClampLimitImpulse(steeringLimitImpulseSum, steeringLimitState);
    if (steeringLimitState == universal_limit_equal)
    {
        steeringImpulseSum = 0.0f;
    }

    // Rotation around cross(axisB, axisA) would break the right angle and is constrained.
    // The remaining relative rotations are steering around axisA and spin around axisB.
    // Since the axes are perpendicular, projecting wB - wA onto axisB selects the spin speed.
    // The spin motor and limit therefore use the angular Jacobian [-axisB, axisB].
    spinAxis = axisB;
    float spinK = Dot(spinAxis, invInertiaSum * spinAxis);
    spinM = spinK != 0.0f ? 1.0f / spinK : 0.0f;

    // axisB alone cannot measure spin because it does not move when body B rotates around it.
    // refAxisB acts like a spoke fixed to body B.
    // Project axisA onto axisB's normal plane to build a zero-spin basis that follows steering,
    // then measure the spoke in that basis so steering is not mistaken for wheel spin.
    Vec3 spinRefAxisA = GramSchmidt(axisA, spinAxis);
    if (spinRefAxisA.Normalize() == 0.0f)
    {
        CoordinateSystem(spinAxis, &spinRefAxisA);
    }

    Vec3 spinTangentAxis = Cross(spinAxis, spinRefAxisA);
    spinTangentAxis.Normalize();

    float spinCosTheta = Dot(refAxisB, spinRefAxisA);
    float spinSinTheta = Dot(refAxisB, spinTangentAxis);
    spinAngle = std::atan2(spinSinTheta, spinCosTheta);

    if (!spinMotorEnabled)
    {
        spinImpulseSum = 0.0f;
    }

    ComputeBetaAndGamma(&spinBeta, &spinGamma, frequency, dampingRatio, spinM, step.dt);
    float spinLimitK = spinK + spinGamma;
    spinLimitM = spinLimitK != 0.0f ? 1.0f / spinLimitK : 0.0f;

    if (!spinLimitEnabled || spinMaxAngle - spinMinAngle >= two_pi)
    {
        spinLimitState = universal_limit_inactive;
        spinLimitBias = 0.0f;
    }
    else if (spinMinAngle == spinMaxAngle)
    {
        spinLimitState = universal_limit_equal;
        spinLimitBias =
            Clamp(NormalizeAngle(spinAngle - spinMinAngle), -max_joint_angular_correction, max_joint_angular_correction) *
            spinBeta * step.inv_dt;
    }
    else
    {
        // Move the configured interval to the same 2 pi branch as the measured angle.
        float center = 0.5f * (spinMinAngle + spinMaxAngle);
        float shift = two_pi * std::round((spinAngle - center) / two_pi);
        float lower = spinMinAngle + shift;
        float upper = spinMaxAngle + shift;

        if (spinAngle < lower - angular_slop)
        {
            spinLimitState = universal_limit_at_lower;
            spinLimitBias = Max(spinAngle - (lower - angular_slop), -max_joint_angular_correction) * spinBeta * step.inv_dt;
        }
        else if (spinAngle > upper + angular_slop)
        {
            spinLimitState = universal_limit_at_upper;
            spinLimitBias = Min(spinAngle - (upper + angular_slop), max_joint_angular_correction) * spinBeta * step.inv_dt;
        }
        else
        {
            spinLimitState = universal_limit_inactive;
            spinLimitBias = 0.0f;
        }
    }

    spinLimitImpulseSum = ClampLimitImpulse(spinLimitImpulseSum, spinLimitState);
    if (spinLimitState == universal_limit_equal)
    {
        spinImpulseSum = 0.0f;
    }
}

void UniversalAngleJoint::WarmStart()
{
    ApplyAngularImpulse(perpAxis, perpImpulseSum);

    if (steeringMotorEnabled && steeringLimitState != universal_limit_equal)
    {
        ApplyAngularImpulse(steeringAxis, steeringImpulseSum);
    }
    if (steeringLimitState != universal_limit_inactive)
    {
        ApplyAngularImpulse(steeringAxis, steeringLimitImpulseSum);
    }
    if (spinMotorEnabled && spinLimitState != universal_limit_equal)
    {
        ApplyAngularImpulse(spinAxis, spinImpulseSum);
    }
    if (spinLimitState != universal_limit_inactive)
    {
        ApplyAngularImpulse(spinAxis, spinLimitImpulseSum);
    }
}

void UniversalAngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Keep the steering and spin axes perpendicular while leaving rotation around either axis free.
    float perpendicularJV = Dot(perpAxis, sB->angularVelocity - sA->angularVelocity);
    float lambda = perpM * -(perpendicularJV + perpBias + perpImpulseSum * perpGamma);
    perpImpulseSum += lambda;
    ApplyAngularImpulse(perpAxis, lambda);

    if (steeringMotorEnabled && steeringLimitState != universal_limit_equal)
    {
        // Soft angular servo: drive the measured steering angle toward the target.
        float steeringJV = Dot(steeringAxis, sB->angularVelocity - sA->angularVelocity);
        lambda = steeringM * -(steeringJV + steeringBias + steeringImpulseSum * steeringGamma);
        float maxImpulse = maxSteeringTorque * step.dt;
        float newImpulseSum = Clamp(steeringImpulseSum + lambda, -maxImpulse, maxImpulse);

        lambda = newImpulseSum - steeringImpulseSum;
        steeringImpulseSum = newImpulseSum;
        ApplyAngularImpulse(steeringAxis, lambda);
    }

    if (steeringLimitState != universal_limit_inactive)
    {
        float steeringJV = Dot(steeringAxis, sB->angularVelocity - sA->angularVelocity);
        lambda = steeringM * -(steeringJV + steeringLimitBias + steeringLimitImpulseSum * steeringGamma);

        float newImpulseSum;
        if (steeringLimitState == universal_limit_equal)
        {
            newImpulseSum = steeringLimitImpulseSum + lambda;
        }
        else
        {
            newImpulseSum = ClampLimitImpulse(steeringLimitImpulseSum + lambda, steeringLimitState);
        }

        lambda = newImpulseSum - steeringLimitImpulseSum;
        steeringLimitImpulseSum = newImpulseSum;
        ApplyAngularImpulse(steeringAxis, lambda);
    }

    if (spinMotorEnabled && spinLimitState != universal_limit_equal)
    {
        // Velocity motor: drive the relative angular velocity around axisB.
        float spinJV = Dot(spinAxis, sB->angularVelocity - sA->angularVelocity) - spinSpeed;
        lambda = -spinM * spinJV;
        float maxImpulse = maxSpinTorque * step.dt;
        float newImpulseSum = Clamp(spinImpulseSum + lambda, -maxImpulse, maxImpulse);

        lambda = newImpulseSum - spinImpulseSum;
        spinImpulseSum = newImpulseSum;
        ApplyAngularImpulse(spinAxis, lambda);
    }

    if (spinLimitState != universal_limit_inactive)
    {
        float spinJV = Dot(spinAxis, sB->angularVelocity - sA->angularVelocity);
        lambda = spinLimitM * -(spinJV + spinLimitBias + spinLimitImpulseSum * spinGamma);

        float newImpulseSum;
        if (spinLimitState == universal_limit_equal)
        {
            newImpulseSum = spinLimitImpulseSum + lambda;
        }
        else
        {
            // Lower and upper limits accept impulse only toward the valid interval.
            newImpulseSum = ClampLimitImpulse(spinLimitImpulseSum + lambda, spinLimitState);
        }

        lambda = newImpulseSum - spinLimitImpulseSum;
        spinLimitImpulseSum = newImpulseSum;
        ApplyAngularImpulse(spinAxis, lambda);
    }
}

void UniversalAngleJoint::ApplyAngularImpulse(const Vec3& axis, float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 impulse = axis * lambda;

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * impulse;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * impulse;
    }
}

const Vec3& UniversalAngleJoint::GetLocalAxisA() const
{
    return localAxisA;
}

const Vec3& UniversalAngleJoint::GetLocalAxisB() const
{
    return localAxisB;
}

const Vec3& UniversalAngleJoint::GetLocalReferenceAxisA() const
{
    return localRefAxisA;
}

const Vec3& UniversalAngleJoint::GetLocalReferenceAxisB() const
{
    return localRefAxisB;
}

float UniversalAngleJoint::GetSteeringAngle() const
{
    return steeringAngle;
}

bool UniversalAngleJoint::IsSteeringMotorEnabled() const
{
    return steeringMotorEnabled;
}

void UniversalAngleJoint::SetSteeringMotorEnabled(bool enabled)
{
    steeringMotorEnabled = enabled;
    if (!steeringMotorEnabled)
    {
        steeringImpulseSum = 0.0f;
    }
}

float UniversalAngleJoint::GetTargetSteeringAngle() const
{
    return targetSteeringAngle;
}

void UniversalAngleJoint::SetTargetSteeringAngle(float newTargetSteeringAngle)
{
    targetSteeringAngle = newTargetSteeringAngle;
}

float UniversalAngleJoint::GetSteeringFrequency() const
{
    return steeringFrequency;
}

void UniversalAngleJoint::SetSteeringFrequency(float newSteeringFrequency)
{
    steeringFrequency = newSteeringFrequency;
}

float UniversalAngleJoint::GetSteeringDampingRatio() const
{
    return steeringDampingRatio;
}

void UniversalAngleJoint::SetSteeringDampingRatio(float newSteeringDampingRatio)
{
    steeringDampingRatio = Max(newSteeringDampingRatio, 0.0f);
}

float UniversalAngleJoint::GetMaxSteeringTorque() const
{
    return maxSteeringTorque;
}

void UniversalAngleJoint::SetMaxSteeringTorque(float torque)
{
    maxSteeringTorque = torque < 0.0f ? max_float : torque;
}

bool UniversalAngleJoint::IsSteeringLimitEnabled() const
{
    return steeringLimitEnabled;
}

void UniversalAngleJoint::SetSteeringLimitEnabled(bool enabled)
{
    steeringLimitEnabled = enabled;
    if (!steeringLimitEnabled)
    {
        steeringLimitImpulseSum = 0.0f;
    }
}

float UniversalAngleJoint::GetSteeringMinAngle() const
{
    return steeringMinAngle;
}

void UniversalAngleJoint::SetSteeringMinAngle(float angle)
{
    steeringMinAngle = angle;
    steeringMaxAngle = Max(steeringMinAngle, steeringMaxAngle);
}

float UniversalAngleJoint::GetSteeringMaxAngle() const
{
    return steeringMaxAngle;
}

void UniversalAngleJoint::SetSteeringMaxAngle(float angle)
{
    steeringMaxAngle = angle;
    steeringMinAngle = Min(steeringMinAngle, steeringMaxAngle);
}

bool UniversalAngleJoint::IsSpinMotorEnabled() const
{
    return spinMotorEnabled;
}

void UniversalAngleJoint::SetSpinMotorEnabled(bool enabled)
{
    spinMotorEnabled = enabled;
    if (!spinMotorEnabled)
    {
        spinImpulseSum = 0.0f;
    }
}

float UniversalAngleJoint::GetSpinAngle() const
{
    return spinAngle;
}

float UniversalAngleJoint::GetSpinSpeed() const
{
    return spinSpeed;
}

void UniversalAngleJoint::SetSpinSpeed(float speed)
{
    spinSpeed = speed;
}

float UniversalAngleJoint::GetMaxSpinTorque() const
{
    return maxSpinTorque;
}

void UniversalAngleJoint::SetMaxSpinTorque(float torque)
{
    maxSpinTorque = torque < 0.0f ? max_float : torque;
}

bool UniversalAngleJoint::IsSpinLimitEnabled() const
{
    return spinLimitEnabled;
}

void UniversalAngleJoint::SetSpinLimitEnabled(bool enabled)
{
    spinLimitEnabled = enabled;
    if (!spinLimitEnabled)
    {
        spinLimitImpulseSum = 0.0f;
    }
}

float UniversalAngleJoint::GetSpinMinAngle() const
{
    return spinMinAngle;
}

void UniversalAngleJoint::SetSpinMinAngle(float angle)
{
    spinMinAngle = angle;
    spinMaxAngle = Max(spinMinAngle, spinMaxAngle);
}

float UniversalAngleJoint::GetSpinMaxAngle() const
{
    return spinMaxAngle;
}

void UniversalAngleJoint::SetSpinMaxAngle(float angle)
{
    spinMaxAngle = angle;
    spinMinAngle = Min(spinMinAngle, spinMaxAngle);
}

} // namespace muli3
