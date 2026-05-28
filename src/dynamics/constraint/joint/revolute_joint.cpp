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
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float jointMinAngle,
    float jointMaxAngle,
    float jointFrequency,
    float jointDampingRatio
)
    : Joint(revolute_joint, bodyA, bodyB, jointFrequency, jointDampingRatio)
    , angleOffset{ 0.0f }
    , minAngle{ jointMinAngle }
    , maxAngle{ jointMaxAngle }
    , currentAngle{ 0.0f }
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , swingM{ 0.0f }
    , swingBias{ 0.0f }
    , swingImpulseSum{ 0.0f }
    , angleM{ 0.0f }
    , angleBias{ 0.0f }
    , angleImpulseSum{ 0.0f }
    , limitState{ revolute_limit_inactive }
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

    // Linear part: keep the world anchor points together.
    // clang-format off
    Mat3 linearK = Mat3(sA->invMass + sB->invMass)
                 + skewRA.GetTranspose() * s->invIA * skewRA
                 + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(linearK.TraceInverse() / 3.0f, step.dt);

    linearK.ex.x += s->gamma;
    linearK.ey.y += s->gamma;
    linearK.ez.z += s->gamma;

    linearM = linearK.GetInverse();

    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;

    linearBias = (pb - pa) * s->beta * step.inv_dt;

    // Angular part: align the hinge axes while leaving twist free.
    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);
    Vec3 refAxisA = bodyA->GetRotation().Rotate(localNormalAxisA);
    Vec3 refAxisB = bodyB->GetRotation().Rotate(localNormalAxisB);
    Vec3 binormalA = Cross(axisA, refAxisA);
    binormalA.Normalize();

    float axisDot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    float swingAngle = std::acos(axisDot);

    swingAxis = Cross(axisA, axisB);
    if (swingAxis.Normalize() == 0.0f)
    {
        if (axisDot < 0.0f)
        {
            CoordinateSystem(axisA, &swingAxis);
        }
        else
        {
            swingAxis = Vec3::zero;
        }
    }

    if (swingAxis != Vec3::zero)
    {
        float swingK = Dot(swingAxis, s->invIA * swingAxis) + Dot(swingAxis, s->invIB * swingAxis) + s->gamma;
        swingM = swingK != 0.0f ? 1.0f / swingK : 0.0f;
    }
    else
    {
        swingM = 0.0f;
    }

    swingBias = Min(swingAngle, revolute_joint_max_angular_correction) * s->beta * step.inv_dt;

    twistAxis = axisA + axisB;
    if (twistAxis.Normalize() == 0.0f)
    {
        twistAxis = axisA;
    }

    float angleK = Dot(twistAxis, s->invIA * twistAxis) + Dot(twistAxis, s->invIB * twistAxis) + s->gamma;
    angleM = angleK != 0.0f ? 1.0f / angleK : 0.0f;

    currentAngle = GetAngle(refAxisA, binormalA, axisA, refAxisB) - angleOffset;

    if (minAngle == maxAngle)
    {
        limitState = revolute_limit_equal;
        angleBias = Clamp(NormalizeAngle(currentAngle - minAngle), -max_joint_angular_correction, max_joint_angular_correction) *
                    s->beta * step.inv_dt;
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
            angleBias = Max(currentAngle - (lower - angular_slop), -max_joint_angular_correction) * s->beta * step.inv_dt;
        }
        else if (currentAngle > upper + angular_slop)
        {
            limitState = revolute_limit_at_upper;
            angleBias = Min(currentAngle - (upper + angular_slop), max_joint_angular_correction) * s->beta * step.inv_dt;
        }
        else
        {
            limitState = revolute_limit_inactive;
            angleBias = 0.0f;
        }
    }

    angleImpulseSum = ClampImpulse(angleImpulseSum, limitState);
}

void RevoluteJoint::WarmStart()
{
    ApplyLinearImpulse(linearImpulseSum);
    ApplySwingImpulse(swingImpulseSum);

    if (limitState != revolute_limit_inactive)
    {
        ApplyAngleImpulse(angleImpulseSum);
    }
}

void RevoluteJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));
    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * s->gamma);
    ApplyLinearImpulse(linearLambda);
    linearImpulseSum += linearLambda;

    float swingJV = Dot(swingAxis, sB->angularVelocity - sA->angularVelocity);
    float swingLambda = swingM * -(swingJV + swingBias + swingImpulseSum * s->gamma);
    ApplySwingImpulse(swingLambda);
    swingImpulseSum += swingLambda;

    if (limitState == revolute_limit_inactive)
    {
        return;
    }

    float angleJV = Dot(twistAxis, sB->angularVelocity - sA->angularVelocity);
    float lambda = angleM * -(angleJV + angleBias + angleImpulseSum * s->gamma);

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

void RevoluteJoint::ApplyLinearImpulse(const Vec3& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    sA->linearVelocity -= lambda * sA->invMass;
    sA->angularVelocity -= s->invIA * Cross(ra, lambda);
    sB->linearVelocity += lambda * sB->invMass;
    sB->angularVelocity += s->invIB * Cross(rb, lambda);
}

void RevoluteJoint::ApplySwingImpulse(float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = swingAxis * lambda;

    sA->angularVelocity -= s->invIA * p;
    sB->angularVelocity += s->invIB * p;
}

void RevoluteJoint::ApplyAngleImpulse(float lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = twistAxis * lambda;

    sA->angularVelocity -= s->invIA * p;
    sB->angularVelocity += s->invIB * p;
}

} // namespace muli3
