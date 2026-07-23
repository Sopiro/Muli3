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
    // Lower limits accept positive twist impulses; upper limits accept negative impulses.
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
    , swingBias{ 0.0f, 0.0f }
    , swingImpulseSum{ 0.0f, 0.0f }
    , swingBeta{ 0.0f }
    , swingGamma{ 0.0f }
    , angleM{ 0.0f }
    , angleBias{ 0.0f }
    , angleImpulseSum{ 0.0f }
    , angleBeta{ 0.0f }
    , angleGamma{ 0.0f }
    , limitState{ revolute_limit_inactive }
    , motorEnabled{ false }
    , motorSpeed{ 0.0f }
    , maxMotorTorque{ 0.0f }
    , motorM{ 0.0f }
    , motorImpulseSum{ 0.0f }
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

    // Swing constraint: align the hinge axes while leaving twist free.
    // Two rows perpendicular to axisA remove both swing degrees of freedom.
    float axisDot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    float swingAngle = std::acos(axisDot);

    swingAxis1 = GramSchmidt(refAxisA, axisA);
    if (swingAxis1.Normalize() == 0.0f)
    {
        CoordinateSystem(axisA, &swingAxis1);
    }
    swingAxis2 = Cross(axisA, swingAxis1);

    Mat3 invInertiaSum = s->invIA + s->invIB;

    // J_i = [0, -swingAxis_i, 0, swingAxis_i].
    // Solve both perpendicular angular rows as one block to preserve their inertia coupling.

    Mat2 swingK;
    swingK[0][0] = Dot(swingAxis1, invInertiaSum * swingAxis1);
    swingK[1][1] = Dot(swingAxis2, invInertiaSum * swingAxis2);
    swingK[0][1] = Dot(swingAxis1, invInertiaSum * swingAxis2);
    swingK[1][0] = swingK[0][1];

    ComputeBetaAndGamma(&swingBeta, &swingGamma, swingK.TraceInverse() / 2.0f, step.dt);
    swingK[0][0] += swingGamma;
    swingK[1][1] += swingGamma;
    swingM = swingK.GetInverse();

    Vec3 swingError = Cross(axisA, axisB);
    if (swingError.Normalize() == 0.0f)
    {
        swingError = axisDot < 0.0f ? swingAxis1 : Vec3::zero;
    }

    swingError *= Min(swingAngle, max_joint_angular_correction);
    swingBias.Set(Dot(swingError, swingAxis1), Dot(swingError, swingAxis2));
    swingBias *= swingBeta * step.inv_dt;

    // Twist constraint: after swing is removed, the remaining angle changes
    // with relative angular velocity along the common hinge axis. Thus
    // J_twist = [0, -twistAxis, 0, twistAxis].
    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0)
    {
        // Fall back to A's hinge axis when the average becomes ill-defined.
        twistAxis = axisA;
    }

    float angleK = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis);

    // Effective mass without soft constraint
    motorM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    ComputeBetaAndGamma(&angleBeta, &angleGamma, angleK > 0.0f ? 1.0f / angleK : 0.0f, step.dt);

    angleK += angleGamma;
    angleM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    // Project B's reference axis onto A's hinge plane. atan2 against
    // { refAxisA, binormalA } then gives the signed twist angle.
    currentAngle = GetAngle(refAxisA, binormalA, axisA, refAxisB) - angleOffset;

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

        // Move the configured interval to the 2-pi branch nearest the measured angle.
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

    if (!motorEnabled || limitState == revolute_limit_equal)
    {
        motorImpulseSum = 0.0f;
    }
}

void RevoluteAngleJoint::WarmStart()
{
    // Warm start the swing constraint unconditionally.
    ApplySwingImpulse(swingImpulseSum);

    if (limitState != revolute_limit_inactive)
    {
        // Only warm start the twist limit when it is active.
        ApplyTwistImpulse(angleImpulseSum);
    }

    if (motorEnabled && limitState != revolute_limit_equal)
    {
        ApplyTwistImpulse(motorImpulseSum);
    }
}

void RevoluteAngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Solve swing first so the axis used by the twist limit is well defined.

    Vec3 relativeAngularVelocity = sB->angularVelocity - sA->angularVelocity;
    Vec2 swingJV{ Dot(swingAxis1, relativeAngularVelocity), Dot(swingAxis2, relativeAngularVelocity) };
    Vec2 swingLambda = Mul(swingM, -(swingJV + swingBias + swingImpulseSum * swingGamma));

    ApplySwingImpulse(swingLambda);
    swingImpulseSum += swingLambda;

    if (motorEnabled && limitState != revolute_limit_equal)
    {
        // Drive only the free twist DOF.
        // The accumulated impulse is clamped by torque * dt so the motor applies the same torque at any update rate.
        float motorJV = Dot(twistAxis, sB->angularVelocity - sA->angularVelocity) - motorSpeed;
        float lambda = -motorM * motorJV;
        float maxImpulse = maxMotorTorque * step.dt;
        float newImpulseSum = Clamp(motorImpulseSum + lambda, -maxImpulse, maxImpulse);

        lambda = newImpulseSum - motorImpulseSum;
        motorImpulseSum = newImpulseSum;

        ApplyTwistImpulse(lambda);
    }

    if (limitState == revolute_limit_inactive)
    {
        return;
    }

    // The twist limit is unilateral unless minAngle == maxAngle.
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

    ApplyTwistImpulse(lambda);
}

void RevoluteAngleJoint::ApplySwingImpulse(const Vec2& lambda)
{
    // Apply J_swing^T * lambda.

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = swingAxis1 * lambda.x + swingAxis2 * lambda.y;

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * p;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * p;
    }
}

void RevoluteAngleJoint::ApplyTwistImpulse(float lambda)
{
    // Apply J_twist^T * lambda.

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = twistAxis * lambda;

    if (sA->invMass > 0.0f)
    {
        sA->angularVelocity -= s->invIA * p;
    }
    if (sB->invMass > 0.0f)
    {
        sB->angularVelocity += s->invIB * p;
    }
}

} // namespace muli3
