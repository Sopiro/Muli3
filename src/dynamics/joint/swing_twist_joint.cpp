#include "muli3/frame.h"
#include "muli3/joints.h"

namespace muli3
{

enum
{
    twist_limit_inactive,
    twist_limit_at_lower,
    twist_limit_at_upper,
    twist_limit_equal,
};

static float ClampTwistImpulse(float impulse, int32 limitState)
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

SwingTwistJoint::SwingTwistJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float maxSwingAngle,
    float minTwistAngle,
    float maxTwistAngle,
    float linearFrequency,
    float linearDampingRatio,
    float swingFrequency,
    float swingDampingRatio,
    float twistFrequency,
    float twistDampingRatio
)
    : Joint(swing_twist_joint, bodyA, bodyB)
    , linearFrequency{ Max(linearFrequency, 0.0f) }
    , linearDampingRatio{ Max(linearDampingRatio, 0.0f) }
    , swingFrequency{ Max(swingFrequency, 0.0f) }
    , swingDampingRatio{ Max(swingDampingRatio, 0.0f) }
    , twistFrequency{ Max(twistFrequency, 0.0f) }
    , twistDampingRatio{ Max(twistDampingRatio, 0.0f) }
    , maxSwingAngle{ Clamp(maxSwingAngle, 0.0f, pi) }
    , minTwistAngle{ minTwistAngle }
    , maxTwistAngle{ Max(minTwistAngle, maxTwistAngle) }
    , swingAngle{ 0.0f }
    , twistAngle{ 0.0f }
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , linearBeta{ 0.0f }
    , linearGamma{ 0.0f }
    , swingAxis{ 0.0f, 0.0f, 0.0f }
    , swingM{ 0.0f }
    , swingBias{ 0.0f }
    , swingImpulseSum{ 0.0f }
    , swingBeta{ 0.0f }
    , swingGamma{ 0.0f }
    , swingLimitActive{ false }
    , twistAxis{ 0.0f, 0.0f, 0.0f }
    , twistM{ 0.0f }
    , twistBias{ 0.0f }
    , twistImpulseSum{ 0.0f }
    , twistBeta{ 0.0f }
    , twistGamma{ 0.0f }
    , twistLimitState{ twist_limit_inactive }
{
    Vec3 worldAxis = Length2(axis) > epsilon ? Normalize(axis) : y_axis;

    Vec3 normalAxis;
    CoordinateSystem(worldAxis, &normalAxis);

    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
    localAxisA = bodyA->GetRotation().RotateInv(worldAxis);
    localAxisB = bodyB->GetRotation().RotateInv(worldAxis);
    localNormalAxisA = bodyA->GetRotation().RotateInv(normalAxis);
    localNormalAxisB = bodyB->GetRotation().RotateInv(normalAxis);
}

void SwingTwistJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());
    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Point constraint: C = pb - pa, J = [-I, skew(ra), I, -skew(rb)].
    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    // clang-format off
    Mat3 linearK = Mat3(sA->invMass + sB->invMass)
                 + skewRA.GetTranspose() * s->invIA * skewRA
                 + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(&linearBeta, &linearGamma, linearFrequency, linearDampingRatio, linearK.TraceInverse() / 3.0f, step.dt);
    linearK.ex.x += linearGamma;
    linearK.ey.y += linearGamma;
    linearK.ez.z += linearGamma;
    linearM = linearK.GetInverse();
    linearBias = (sB->motion.c + rb - sA->motion.c - ra) * linearBeta * step.inv_dt;

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localNormalAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localNormalAxisB);

    float axisDot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    swingAngle = std::acos(axisDot);
    swingAxis = Cross(axisA, axisB);

    float axisLength = swingAxis.Normalize();
    if (axisLength == 0.0f)
    {
        CoordinateSystem(axisA, &swingAxis);
    }

    // Circular cone: the unilateral row can only decrease the swing angle.
    swingLimitActive = swingAngle > maxSwingAngle + angular_slop;
    if (swingLimitActive)
    {
        float k = Dot(swingAxis, s->invIA * swingAxis) + Dot(swingAxis, s->invIB * swingAxis);
        ComputeBetaAndGamma(&swingBeta, &swingGamma, swingFrequency, swingDampingRatio, k > 0.0f ? 1.0f / k : 0.0f, step.dt);
        k += swingGamma;
        swingM = k != 0.0f ? 1.0f / k : 0.0f;
        swingBias = Min(swingAngle - (maxSwingAngle + angular_slop), max_joint_angular_correction) * swingBeta * step.inv_dt;
        swingImpulseSum = Min(swingImpulseSum, 0.0f);
    }
    else
    {
        swingImpulseSum = 0.0f;
    }

    // Undo the swing before measuring twist, reusing the cone's axis and sine/cosine.
    // Rodrigues' rotation from axisB to axisA has the opposite sign to swingAxis.
    Vec3 alignedRefAxisB = refAxisB;
    if (axisDot <= 1.0f - epsilon)
    {
        if (axisLength == 0.0f)
        {
            // Twist is ambiguous at antiparallel axes; use a deterministic half-turn.
            Vec3 axis;
            CoordinateSystem(axisB, &axis);
            alignedRefAxisB = -refAxisB + axis * (2.0f * Dot(axis, refAxisB));
        }
        else
        {
            alignedRefAxisB = refAxisB * axisDot - Cross(swingAxis, refAxisB) * axisLength +
                              swingAxis * (Dot(swingAxis, refAxisB) * (1.0f - axisDot));
        }
    }
    Vec3 projected = GramSchmidt(alignedRefAxisB, axisA);
    if (projected.Normalize() == 0.0f)
    {
        projected = refAxisA;
    }
    twistAngle = std::atan2(Dot(projected, Normalize(Cross(axisA, refAxisA))), Dot(projected, refAxisA));

    // Match TwistAngleJoint's symmetric correction axis and periodic limit interval.
    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0.0f)
    {
        twistAxis = axisA;
    }

    if (minTwistAngle == maxTwistAngle)
    {
        twistLimitState = twist_limit_equal;
        twistBias =
            Clamp(NormalizeAngle(twistAngle - minTwistAngle), -max_joint_angular_correction, max_joint_angular_correction);
    }
    else if (maxTwistAngle - minTwistAngle >= two_pi)
    {
        twistLimitState = twist_limit_inactive;
    }
    else
    {
        float shift = two_pi * std::round((twistAngle - 0.5f * (minTwistAngle + maxTwistAngle)) / two_pi);
        float lower = minTwistAngle + shift;
        float upper = maxTwistAngle + shift;
        if (twistAngle < lower - angular_slop)
        {
            twistLimitState = twist_limit_at_lower;
            twistBias = Max(twistAngle - (lower - angular_slop), -max_joint_angular_correction);
        }
        else if (twistAngle > upper + angular_slop)
        {
            twistLimitState = twist_limit_at_upper;
            twistBias = Min(twistAngle - (upper + angular_slop), max_joint_angular_correction);
        }
        else
        {
            twistLimitState = twist_limit_inactive;
        }
    }

    if (twistLimitState != twist_limit_inactive)
    {
        float k = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis);
        ComputeBetaAndGamma(&twistBeta, &twistGamma, twistFrequency, twistDampingRatio, k > 0.0f ? 1.0f / k : 0.0f, step.dt);

        k += twistGamma;
        twistM = k != 0.0f ? 1.0f / k : 0.0f;
        twistBias *= twistBeta * step.inv_dt;
    }

    twistImpulseSum = ClampTwistImpulse(twistImpulseSum, twistLimitState);
}

void SwingTwistJoint::WarmStart()
{
    ApplyLinearImpulse(linearImpulseSum);
    if (swingLimitActive || twistLimitState != twist_limit_inactive)
    {
        ApplyAngularImpulse(swingAxis * swingImpulseSum + twistAxis * twistImpulseSum);
    }
}

void SwingTwistJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Sequential rows share one joint state; each row sees the preceding impulse.
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));
    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * linearGamma);
    ApplyLinearImpulse(linearLambda);
    linearImpulseSum += linearLambda;

    if (swingLimitActive)
    {
        float jv = Dot(swingAxis, sB->angularVelocity - sA->angularVelocity);
        float lambda = -swingM * (jv + swingBias + swingImpulseSum * swingGamma);
        float newImpulseSum = Min(swingImpulseSum + lambda, 0.0f);
        ApplyAngularImpulse(swingAxis * (newImpulseSum - swingImpulseSum));
        swingImpulseSum = newImpulseSum;
    }

    if (twistLimitState != twist_limit_inactive)
    {
        float jv = Dot(twistAxis, sB->angularVelocity - sA->angularVelocity);
        float lambda = -twistM * (jv + twistBias + twistImpulseSum * twistGamma);
        float newImpulseSum = ClampTwistImpulse(twistImpulseSum + lambda, twistLimitState);
        ApplyAngularImpulse(twistAxis * (newImpulseSum - twistImpulseSum));
        twistImpulseSum = newImpulseSum;
    }
}

void SwingTwistJoint::ApplyLinearImpulse(const Vec3& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity -= lambda * sA->invMass;
        sA->angularVelocity -= s->invIA * Cross(ra, lambda);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += lambda * sB->invMass;
        sB->angularVelocity += s->invIB * Cross(rb, lambda);
    }
}

void SwingTwistJoint::ApplyAngularImpulse(const Vec3& impulse)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * impulse;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * impulse;
    }
}

float SwingTwistJoint::GetLinearFrequency() const
{
    return linearFrequency;
}

void SwingTwistJoint::SetLinearFrequency(float newFrequency)
{
    linearFrequency = Max(newFrequency, 0.0f);
}

float SwingTwistJoint::GetLinearDampingRatio() const
{
    return linearDampingRatio;
}

void SwingTwistJoint::SetLinearDampingRatio(float newDampingRatio)
{
    linearDampingRatio = Max(newDampingRatio, 0.0f);
}

float SwingTwistJoint::GetSwingFrequency() const
{
    return swingFrequency;
}

void SwingTwistJoint::SetSwingFrequency(float newFrequency)
{
    swingFrequency = Max(newFrequency, 0.0f);
}

float SwingTwistJoint::GetSwingDampingRatio() const
{
    return swingDampingRatio;
}

void SwingTwistJoint::SetSwingDampingRatio(float newDampingRatio)
{
    swingDampingRatio = Max(newDampingRatio, 0.0f);
}

float SwingTwistJoint::GetTwistFrequency() const
{
    return twistFrequency;
}

void SwingTwistJoint::SetTwistFrequency(float newFrequency)
{
    twistFrequency = Max(newFrequency, 0.0f);
}

float SwingTwistJoint::GetTwistDampingRatio() const
{
    return twistDampingRatio;
}

void SwingTwistJoint::SetTwistDampingRatio(float newDampingRatio)
{
    twistDampingRatio = Max(newDampingRatio, 0.0f);
}

const Vec3& SwingTwistJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const Vec3& SwingTwistJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

const Vec3& SwingTwistJoint::GetLocalAxisA() const
{
    return localAxisA;
}

const Vec3& SwingTwistJoint::GetLocalAxisB() const
{
    return localAxisB;
}

const Vec3& SwingTwistJoint::GetLocalNormalAxisA() const
{
    return localNormalAxisA;
}

const Vec3& SwingTwistJoint::GetLocalNormalAxisB() const
{
    return localNormalAxisB;
}

float SwingTwistJoint::GetSwingAngle() const
{
    return swingAngle;
}

float SwingTwistJoint::GetMaxSwingAngle() const
{
    return maxSwingAngle;
}

float SwingTwistJoint::GetTwistAngle() const
{
    return twistAngle;
}

float SwingTwistJoint::GetMinTwistAngle() const
{
    return minTwistAngle;
}

float SwingTwistJoint::GetMaxTwistAngle() const
{
    return maxTwistAngle;
}

void SwingTwistJoint::SetMaxSwingAngle(float newMaxAngle)
{
    maxSwingAngle = Clamp(newMaxAngle, 0.0f, pi);
    swingImpulseSum = 0.0f;
}

void SwingTwistJoint::SetTwistAngle(float newAngle)
{
    minTwistAngle = newAngle;
    maxTwistAngle = newAngle;
    twistImpulseSum = 0.0f;
}

void SwingTwistJoint::SetMinTwistAngle(float newMinAngle)
{
    minTwistAngle = newMinAngle;
    maxTwistAngle = Max(minTwistAngle, maxTwistAngle);
    twistImpulseSum = 0.0f;
}

void SwingTwistJoint::SetMaxTwistAngle(float newMaxAngle)
{
    maxTwistAngle = newMaxAngle;
    minTwistAngle = Min(minTwistAngle, maxTwistAngle);
    twistImpulseSum = 0.0f;
}

} // namespace muli3
