#include "muli3/revolute_joint.h"
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

static constexpr float revolute_joint_max_angular_correction = 10.0f * pi / 180.0f;

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

RevoluteJoint::RevoluteJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float jointMinAngle,
    float jointMaxAngle,
    float frequency,
    float dampingRatio
)
    : Joint(revolute_joint, bodyA, bodyB, frequency, dampingRatio)
    , angleOffset{ 0.0f }
    , minAngle{ jointMinAngle }
    , maxAngle{ jointMaxAngle }
    , currentAngle{ 0.0f }
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , linearBeta{ 0.0f }
    , linearGamma{ 0.0f }
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
    Vec3 worldAxis = Length2(axis) > epsilon ? Normalize(axis) : y_axis;
    Vec3 normalAxis;
    CoordinateSystem(worldAxis, &normalAxis);

    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
    localAxisA = bodyA->GetRotation().RotateInv(worldAxis);
    localAxisB = bodyB->GetRotation().RotateInv(worldAxis);
    localNormalAxisA = bodyA->GetRotation().RotateInv(normalAxis);
    localNormalAxisB = bodyB->GetRotation().RotateInv(normalAxis);

    maxAngle = Max(minAngle, maxAngle);
}

void RevoluteJoint::Prepare(const Timestep& step)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Anchor constraint: C_linear = pb - pa.
    // Differentiating the offset point velocities gives
    // J_linear = [-I, skew(ra), I, -skew(rb)].
    // clang-format off
    Mat3 linearK = Mat3(sA->invMass + sB->invMass)
                 + skewRA.GetTranspose() * s->invIA * skewRA
                 + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(&linearBeta, &linearGamma, frequency, dampingRatio, linearK.TraceInverse() / 3.0f, step.dt);

    linearK.ex.x += linearGamma;
    linearK.ey.y += linearGamma;
    linearK.ez.z += linearGamma;

    linearM = linearK.GetInverse();

    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;

    linearBias = (pb - pa) * linearBeta * step.inv_dt;

    // Swing constraint: align the hinge axes while leaving twist free.
    // Two rows perpendicular to axisA remove both swing degrees of freedom.
    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localNormalAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localNormalAxisB);
    Vec3 binormalA = Normalize(Cross(axisA, refAxisA));

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

    ComputeBetaAndGamma(&swingBeta, &swingGamma, frequency, dampingRatio, swingK.TraceInverse() / 2.0f, step.dt);
    swingK[0][0] += swingGamma;
    swingK[1][1] += swingGamma;
    swingM = swingK.GetInverse();

    Vec3 swingError = Cross(axisA, axisB);
    if (swingError.Normalize() == 0.0f)
    {
        swingError = axisDot < 0.0f ? swingAxis1 : Vec3::zero;
    }

    swingError *= Min(swingAngle, revolute_joint_max_angular_correction);
    swingBias.Set(Dot(swingError, swingAxis1), Dot(swingError, swingAxis2));
    swingBias *= swingBeta * step.inv_dt;

    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0.0f)
    {
        twistAxis = axisA;
    }

    // Twist limit: the signed hinge angle changes with relative angular
    // velocity along twistAxis, so Jt = [0, -twistAxis, 0, twistAxis].
    float angleK = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis);

    // Effective mass without soft constraint
    motorM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    ComputeBetaAndGamma(&angleBeta, &angleGamma, frequency, dampingRatio, angleK > 0.0f ? 1.0f / angleK : 0.0f, step.dt);
    angleK += angleGamma;
    angleM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

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

void RevoluteJoint::WarmStart()
{
    ApplyLinearImpulse(linearImpulseSum);
    ApplySwingImpulse(swingImpulseSum);

    if (limitState != revolute_limit_inactive)
    {
        ApplyTwistImpulse(angleImpulseSum);
    }

    if (motorEnabled && limitState != revolute_limit_equal)
    {
        ApplyTwistImpulse(motorImpulseSum);
    }
}

void RevoluteJoint::SolveVelocityConstraints(const Timestep& step)
{
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Solve the anchor position and axis alignment before controlling twist.
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));
    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * linearGamma);
    ApplyLinearImpulse(linearLambda);
    linearImpulseSum += linearLambda;

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

void RevoluteJoint::ApplyLinearImpulse(const Vec3& lambda)
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

void RevoluteJoint::ApplySwingImpulse(const Vec2& lambda)
{
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

void RevoluteJoint::ApplyTwistImpulse(float lambda)
{
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
