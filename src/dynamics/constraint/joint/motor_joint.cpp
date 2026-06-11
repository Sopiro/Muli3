#include "muli3/motor_joint.h"

namespace muli3
{

MotorJoint::MotorJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, float maxJointForce, float maxJointTorque, float frequency, float dampingRatio
)
    : Joint(motor_joint, bodyA, bodyB, frequency, dampingRatio)
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
    , linearBeta{ 0.0f }
    , linearGamma{ 0.0f }
    , angularBeta{ 0.0f }
    , angularGamma{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
    orientationOffset = bodyA->GetRotation().GetConjugate() * bodyB->GetRotation();

    linearOffset = Vec3::zero;
    angularOffset = Vec3::zero;

    maxForce = maxJointForce < 0 ? max_float : Clamp<float>(maxJointForce, 0.0f, max_float);
    maxTorque = maxJointTorque < 0 ? max_float : Clamp<float>(maxJointTorque, 0.0f, max_float);
}

void MotorJoint::Prepare(const Timestep& step)
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

    // Linear effective mass
    // clang-format off
    Mat3 linearK = Mat3(sA->invMass + sB->invMass)
                 + skewRA.GetTranspose() * s->invIA * skewRA
                 + skewRB.GetTranspose() * s->invIB * skewRB;
    // clang-format on

    ComputeBetaAndGamma(&linearBeta, &linearGamma, linearK.TraceInverse() / 3.0f, step.dt);

    linearK.ex.x += linearGamma;
    linearK.ey.y += linearGamma;
    linearK.ez.z += linearGamma;

    linearM = linearK.GetInverse();

    // Angular effective mass
    Mat3 angularK = s->invIA + s->invIB;
    ComputeBetaAndGamma(&angularBeta, &angularGamma, angularK.TraceInverse() / 3.0f, step.dt);
    angularK.ex.x += angularGamma;
    angularK.ey.y += angularGamma;
    angularK.ez.z += angularGamma;

    angularM = angularK.GetInverse();

    // Linear bias
    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;
    linearBias = (pb - pa + linearOffset) * linearBeta * step.inv_dt;

    // Angular bias
    Quat qTarget = sA->motion.q * orientationOffset;
    Quat qError = sB->motion.q * qTarget.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    angularBias = (Vec3{ qError.x, qError.y, qError.z } * 2.0f - angularOffset) * angularBeta * step.inv_dt;
}

void MotorJoint::WarmStart()
{
    ApplyImpulse(linearImpulseSum, angularImpulseSum);
}

void MotorJoint::SolveVelocityConstraints(const Timestep& step)
{
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Linear
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * linearGamma);

    // Angular
    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * angularGamma);

    // Clamp linear impulse
    {
        float maxLinearImpulse = maxForce * step.dt;
        Vec3 oldLinearImpulse = linearImpulseSum;
        linearImpulseSum += linearLambda;

        if (Length2(linearImpulseSum) > maxLinearImpulse * maxLinearImpulse)
        {
            linearImpulseSum = Normalize(linearImpulseSum) * maxLinearImpulse;
        }

        linearLambda = linearImpulseSum - oldLinearImpulse;
    }

    // Clamp angular impulse
    {
        float maxAngularImpulse = maxTorque * step.dt;
        Vec3 oldAngularImpulse = angularImpulseSum;
        angularImpulseSum += angularLambda;

        if (Length2(angularImpulseSum) > maxAngularImpulse * maxAngularImpulse)
        {
            angularImpulseSum = Normalize(angularImpulseSum) * maxAngularImpulse;
        }

        angularLambda = angularImpulseSum - oldAngularImpulse;
    }

    ApplyImpulse(linearLambda, angularLambda);
}

void MotorJoint::ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (!bodyA->IsStatic())
    {
        sA->linearVelocity -= sA->invMass * linearLambda;
        sA->angularVelocity -= s->invIA * (Cross(ra, linearLambda) + angularLambda);
    }
    if (!bodyB->IsStatic())
    {
        sB->linearVelocity += sB->invMass * linearLambda;
        sB->angularVelocity += s->invIB * (Cross(rb, linearLambda) + angularLambda);
    }
}

} // namespace muli3
