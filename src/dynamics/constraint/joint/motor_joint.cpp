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

    // Negative limits select an unbounded servo.
    // Finite limits are converted to impulses per step.
    maxForce = maxJointForce < 0 ? max_float : Clamp(maxJointForce, 0.0f, max_float);
    maxTorque = maxJointTorque < 0 ? max_float : Clamp(maxJointTorque, 0.0f, max_float);
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

    // Linear constraint: C = pb - pa + linearOffset. Differentiating the two
    // anchor positions gives Jl = [-I, skew(ra), I, -skew(rb)].
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

    // Angular constraint: C is the rotation vector from the target relative
    // orientation to qB. Its small-angle derivative is wb - wa, hence
    // Ja = [0, -I, 0, I].
    Mat3 angularK = s->invIA + s->invIB;
    ComputeBetaAndGamma(&angularBeta, &angularGamma, angularK.TraceInverse() / 3.0f, step.dt);
    angularK.ex.x += angularGamma;
    angularK.ey.y += angularGamma;
    angularK.ez.z += angularGamma;

    angularM = angularK.GetInverse();

    // Biases are beta / dt times the position-level constraints.
    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;
    linearBias = (pb - pa + linearOffset) * linearBeta * step.inv_dt;

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

    // Solve K * lambda = -(J * V + bias + gamma * impulseSum) for each block.
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * linearGamma);

    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * angularGamma);

    // Clamp the accumulated impulse so the force limit is independent of the iteration count.
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

    // Torque is limited in the same accumulated-impulse space.
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
