#include "muli3/joints.h"

namespace muli3
{

// BallSocketJoint + orientation constraint

WeldJoint::WeldJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
    : Joint(weld_joint, bodyA, bodyB, frequency, dampingRatio)
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
    , linearBeta{ 0.0f }
    , linearGamma{ 0.0f }
    , angularBeta{ 0.0f }
    , angularGamma{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);

    // Store the relative orientation that represents C_angular = 0.
    // qOffset = qA^-1 * qB
    orientationOffset = bodyA->GetRotation().GetConjugate() * bodyB->GetRotation();
}

void WeldJoint::Prepare(const Timestep& step)
{
    // C_linear = pb - pa gives Jl = [-I, skew(ra), I, -skew(rb)].
    // C_angular is the relative rotation vector, whose small-angle derivative
    // is wb - wa and therefore Ja = [0, -I, 0, I].

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    ra = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    rb = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());

    Mat3 skewRA = Skew(ra);
    Mat3 skewRB = Skew(rb);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Kl = Jl * M^-1 * Jl^T.
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

    // Ka = Ja * M^-1 * Ja^T = invIA + invIB.
    Mat3 angularK = s->invIA + s->invIB;
    ComputeBetaAndGamma(&angularBeta, &angularGamma, frequency, dampingRatio, angularK.TraceInverse() / 3.0f, step.dt);

    angularK.ex.x += angularGamma;
    angularK.ey.y += angularGamma;
    angularK.ez.z += angularGamma;

    angularM = angularK.GetInverse();

    // Biases are beta / dt times the position-level constraints.
    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;
    linearBias = (pb - pa) * linearBeta * step.inv_dt;

    Quat qTarget = sA->motion.q * orientationOffset;
    Quat qError = sB->motion.q * qTarget.GetConjugate();

    // q and -q represent the same rotation. Choose the shorter rotation.
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    // For a small rotation, the rotation vector is 2 * qError.xyz.
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * angularBeta * step.inv_dt;
}

void WeldJoint::WarmStart()
{
    ApplyImpulse(linearImpulseSum, angularImpulseSum);
}

void WeldJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Evaluate Jl * V and Ja * V, then solve both equality constraints.
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * linearGamma);

    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;

    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * angularGamma);

    ApplyImpulse(linearLambda, angularLambda);
    linearImpulseSum += linearLambda;
    angularImpulseSum += angularLambda;
}

void WeldJoint::ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda)
{
    // Apply Jl^T * linearLambda + Ja^T * angularLambda.

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity -= linearLambda * sA->invMass;
        sA->angularVelocity -= s->invIA * (Cross(ra, linearLambda) + angularLambda);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += linearLambda * sB->invMass;
        sB->angularVelocity += s->invIB * (Cross(rb, linearLambda) + angularLambda);
    }
}

const Vec3& WeldJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const Vec3& WeldJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

const Quat& WeldJoint::GetOrientationOffset() const
{
    return orientationOffset;
}

} // namespace muli3
