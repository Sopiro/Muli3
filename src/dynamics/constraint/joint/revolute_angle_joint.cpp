#include "muli3/revolute_angle_joint.h"
#include "muli3/frame.h"

namespace muli3
{

enum
{
    revolute_limit_inactive,
    revolute_limit_at_lower,
    revolute_limit_at_upper,
    revolute_limit_equal,
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
    case revolute_limit_at_lower:
        return Max(impulse, 0.0f);
    case revolute_limit_at_upper:
        return Min(impulse, 0.0f);
    case revolute_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

static float GetAngle(const Vec3& frameX, const Vec3& frameY, const Vec3& frameZ, const Vec3& refAxisB)
{
    Vec3 projected = GramSchmidt(refAxisB, frameZ);
    if (projected.Normalize() == 0)
    {
        projected = frameX;
    }

    float x = Dot(projected, frameX);
    float y = Dot(projected, frameY);

    return std::atan2(y, x);
}

RevoluteAngleJoint::RevoluteAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& worldAxis, float minAngle, float maxAngle, float frequency, float dampingRatio
)
    : Joint(revolute_angle_joint, bodyA, bodyB, frequency, dampingRatio)
    , angleOffset{ 0.0f }
    , minAngle{ minAngle }
    , maxAngle{ maxAngle }
    , currentAngle{ 0.0f }
    , swingM{ 0.0f }
    , swingBias{ 0.0f }
    , swingImpulseSum{ 0.0f }
    , swingBeta{ 0.0f }
    , swingGamma{ 0.0f }
    , angleM{ 0.0f }
    , angleBias{ 0.0f }
    , angleImpulseSum{ 0.0f }
    , angleBeta{ 0.0f }
    , angleGamma{ 0.0f }
    , limitState{ revolute_limit_inactive }
{
    Vec3 axis = Length2(worldAxis) > epsilon ? Normalize(worldAxis) : y_axis;
    Vec3 normalAxis;
    CoordinateSystem(axis, &normalAxis);

    // Store the hinge axis on each body and also store one perpendicular
    // reference axis on each body so the remaining twist angle can be measured.
    localAxisA = bodyA->GetRotation().RotateInv(axis);
    localAxisB = bodyB->GetRotation().RotateInv(axis);
    localNormalAxisA = bodyA->GetRotation().RotateInv(normalAxis);
    localNormalAxisB = bodyB->GetRotation().RotateInv(normalAxis);

    maxAngle = Max(minAngle, maxAngle);
}

void RevoluteAngleJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localNormalAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localNormalAxisB);

    Vec3 binormalA = Cross(axisA, refAxisA);
    binormalA.Normalize();

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Swing part:
    // Keep the two hinge axes aligned, but do not constrain twist around them.
    // The corrective torque acts around axisA x axisB.
    float axisDot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    float swingAngle = std::acos(axisDot);

    swingAxis = Cross(axisA, axisB);
    if (swingAxis.Normalize() == 0)
    {
        if (axisDot < 0.0f)
        {
            // If the hinge axes point in opposite directions, the cross product
            // vanishes even though the configuration is maximally wrong.
            // Pick any stable perpendicular axis and rotate around it.
            CoordinateSystem(axisA, &swingAxis);
        }
        else
        {
            // The axes already match, so the swing constraint has no work to do.
            swingAxis = Vec3::zero;
        }
    }

    if (swingAxis != Vec3::zero)
    {
        float swingK = Dot(swingAxis, s->invIA * swingAxis) + Dot(swingAxis, s->invIB * swingAxis);
        ComputeBetaAndGamma(&swingBeta, &swingGamma, swingK > 0.0f ? 1.0f / swingK : 0.0f, step.dt);
        swingK += swingGamma;
        swingM = swingK != 0.0f ? 1.0f / swingK : 0.0f;
    }
    else
    {
        swingM = 0.0f;
        swingBeta = 0.0f;
        swingGamma = 0.0f;
        swingImpulseSum = 0.0f;
    }

    swingBias = Min(swingAngle, max_joint_angular_correction) * swingBeta * step.inv_dt;

    // Twist part:
    // Once the hinge axes are aligned, only the relative rotation around that axis remains.
    // Use the averaged axis when possible for better symmetry.
    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0)
    {
        // Fall back to A's hinge axis when the average becomes ill-defined.
        twistAxis = axisA;
    }

    float angleK = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis);

    ComputeBetaAndGamma(&angleBeta, &angleGamma, angleK > 0.0f ? 1.0f / angleK : 0.0f, step.dt);

    angleK += angleGamma;
    angleM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    // Measure the signed twist angle of body B around body A's hinge axis.
    // Project B's reference axis onto A's hinge plane and compare it against A's reference frame { refAxisA, binormalA }.
    currentAngle = GetAngle(refAxisA, binormalA, axisA, refAxisB) - angleOffset;

    // Twist limit around the hinge axis.
    // J = [0 -twistAxis 0 twistAxis]

    if (minAngle == maxAngle)
    {
        limitState = revolute_limit_equal;
        angleBias = Clamp(NormalizeAngle(currentAngle - minAngle), -max_joint_angular_correction, max_joint_angular_correction) *
                    angleBeta * step.inv_dt;
    }
    else if (maxAngle - minAngle >= two_pi)
    {
        limitState = revolute_limit_inactive;
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
            limitState = revolute_limit_at_lower;
            angleBias = Max(currentAngle - (lower - angular_slop), -max_joint_angular_correction) * angleBeta * step.inv_dt;
        }
        else if (currentAngle > upper + angular_slop)
        {
            limitState = revolute_limit_at_upper;
            angleBias = Min(currentAngle - (upper + angular_slop), max_joint_angular_correction) * angleBeta * step.inv_dt;
        }
        else
        {
            limitState = revolute_limit_inactive;
            angleBias = 0.0f;
        }
    }

    angleImpulseSum = ClampImpulse(angleImpulseSum, limitState);
}

void RevoluteAngleJoint::WarmStart()
{
    // Warm start the swing constraint unconditionally.
    ApplySwingImpulse(swingImpulseSum);

    if (limitState != revolute_limit_inactive)
    {
        // Only warm start the twist limit when it is active.
        ApplyAngleImpulse(angleImpulseSum);
    }
}

void RevoluteAngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // First solve the swing constraint so the hinge axes line up while leaving
    // the twist angle free.
    // Pc = J^t * lambda
    // lambda = (J * M^-1 * J^t)^-1 * -(J*v+b)

    float swingJV = Dot(swingAxis, sB->angularVelocity - sA->angularVelocity);
    float swingLambda = swingM * -(swingJV + swingBias + swingImpulseSum * swingGamma);

    ApplySwingImpulse(swingLambda);
    swingImpulseSum += swingLambda;

    if (limitState == revolute_limit_inactive)
    {
        return;
    }

    // Then solve the remaining twist limit around the aligned hinge axis.
    float angleJV = Dot(twistAxis, sB->angularVelocity - sA->angularVelocity);
    float lambda = angleM * -(angleJV + angleBias + angleImpulseSum * angleGamma);

    float newImpulseSum;
    if (limitState == revolute_limit_equal)
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

void RevoluteAngleJoint::ApplySwingImpulse(float lambda)
{
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = swingAxis * lambda;

    if (!bodyA->IsStatic())
    {
        sA->angularVelocity -= s->invIA * p;
    }
    if (!bodyB->IsStatic())
    {
        sB->angularVelocity += s->invIB * p;
    }
}

void RevoluteAngleJoint::ApplyAngleImpulse(float lambda)
{
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = twistAxis * lambda;

    if (!bodyA->IsStatic())
    {
        sA->angularVelocity -= s->invIA * p;
    }
    if (!bodyB->IsStatic())
    {
        sB->angularVelocity += s->invIB * p;
    }
}

} // namespace muli3
