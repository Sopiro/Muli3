#include "muli3/weld_joint.h"

namespace muli3
{

// BallSocketJoint + orientation constraint

WeldJoint::WeldJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
    : Joint(weld_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , linearImpulseSum{ 0.0f, 0.0f, 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);

    // Store relative orientation: qOffset = qA^-1 * qB
    orientationOffset = bodyA->GetRotation().GetConjugate() * bodyB->GetRotation();
}

void WeldJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // Linear part: J = [-I, -skew(ra), I, skew(rb)]
    // Angular part: J = [0, -I, 0, I]

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

    // Position error
    Vec3 pa = bodyA->motion.c + ra;
    Vec3 pb = bodyB->motion.c + rb;
    linearBias = (pb - pa) * beta * step.inv_dt;

    // Orientation error: compute the rotation difference
    // qError = qA * qOffset * qB^-1  (should be identity if no error)
    Quat qTarget = bodyA->motion.q * orientationOffset;
    Quat qError = bodyB->motion.q * qTarget.GetConjugate();

    // Ensure shortest path
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    // The angular error is 2 * imaginary part of qError (small angle approximation)
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(linearImpulseSum, angularImpulseSum);
    }
}

void WeldJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Linear velocity constraint
    Vec3 linearJV =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) - (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * gamma);

    // Angular velocity constraint
    Vec3 angularJV = bodyB->angularVelocity - bodyA->angularVelocity;

    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * gamma);

    ApplyImpulse(linearLambda, angularLambda);
    linearImpulseSum += linearLambda;
    angularImpulseSum += angularLambda;
}

void WeldJoint::ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= linearLambda * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->GetWorldInverseInertiaTensor() * (Cross(ra, linearLambda) + angularLambda);
    bodyB->linearVelocity += linearLambda * bodyB->invMass;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * (Cross(rb, linearLambda) + angularLambda);
}

} // namespace muli3
