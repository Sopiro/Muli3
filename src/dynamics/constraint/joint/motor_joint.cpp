#include "muli3/motor_joint.h"

namespace muli3
{

MotorJoint::MotorJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float maxJointForce, float maxJointTorque,
    float jointFrequency, float jointDampingRatio, float jointMass
)
    : Joint(motor_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
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
    ComputeBetaAndGamma(step);

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    // Linear effective mass
    // clang-format off
    Mat3 linearK = Mat3(bodyA->invMass + bodyB->invMass)
                 + skewRA.GetTranspose() * invIA * skewRA
                 + skewRB.GetTranspose() * invIB * skewRB;
    // clang-format on

    linearK.ex.x += gamma;
    linearK.ey.y += gamma;
    linearK.ez.z += gamma;

    linearM = linearK.GetInverse();

    // Angular effective mass
    Mat3 angularK = invIA + invIB;
    angularK.ex.x += gamma;
    angularK.ey.y += gamma;
    angularK.ez.z += gamma;

    angularM = angularK.GetInverse();

    // Linear bias
    Vec3 pa = bodyA->motion.c + ra;
    Vec3 pb = bodyB->motion.c + rb;
    linearBias = (pb - pa + linearOffset) * beta * step.inv_dt;

    // Angular bias
    Quat qTarget = bodyA->motion.q * orientationOffset;
    Quat qError = bodyB->motion.q * qTarget.GetConjugate();
    if (qError.w < 0.0f) qError = -qError;
    angularBias = (Vec3{ qError.x, qError.y, qError.z } * 2.0f - angularOffset) * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(linearImpulseSum, angularImpulseSum);
    }
}

void MotorJoint::SolveVelocityConstraints(const Timestep& step)
{
    // Linear
    Vec3 linearJV =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
        (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * gamma);

    // Angular
    Vec3 angularJV = bodyB->angularVelocity - bodyA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * gamma);

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
    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    bodyA->linearVelocity -= bodyA->invMass * linearLambda;
    bodyA->angularVelocity -= invIA * (Cross(ra, linearLambda) + angularLambda);
    bodyB->linearVelocity += bodyB->invMass * linearLambda;
    bodyB->angularVelocity += invIB * (Cross(rb, linearLambda) + angularLambda);
}

} // namespace muli3
