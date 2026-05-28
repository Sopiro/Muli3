#include "muli3/weld_joint.h"

namespace muli3
{

// BallSocketJoint + orientation constraint

WeldJoint::WeldJoint(RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float jointFrequency, float jointDampingRatio)
    : Joint(weld_joint, bodyA, bodyB, jointFrequency, jointDampingRatio)
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
    // Compute Jacobian J and effective mass W
    // Linear part: J = [-I, -skew(ra), I, skew(rb)]
    // Angular part: J = [0, -I, 0, I]

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

    ComputeBetaAndGamma(linearK.TraceInverse() / 3.0f, step.dt);

    linearK.ex.x += s->gamma;
    linearK.ey.y += s->gamma;
    linearK.ez.z += s->gamma;

    linearM = linearK.GetInverse();

    // Angular effective mass
    Mat3 angularK = s->invIA + s->invIB;

    angularK.ex.x += s->gamma;
    angularK.ey.y += s->gamma;
    angularK.ez.z += s->gamma;

    angularM = angularK.GetInverse();

    // Position error
    Vec3 pa = sA->motion.c + ra;
    Vec3 pb = sB->motion.c + rb;
    linearBias = (pb - pa) * s->beta * step.inv_dt;

    // Orientation error: compute the rotation difference
    // qError = qA * qOffset * qB^-1  (should be identity if no error)
    Quat qTarget = sA->motion.q * orientationOffset;
    Quat qError = sB->motion.q * qTarget.GetConjugate();

    // Ensure shortest path
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }

    // The angular error is 2 * imaginary part of qError (small angle approximation)
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * s->beta * step.inv_dt;
}

void WeldJoint::WarmStart()
{
    ApplyImpulse(linearImpulseSum, angularImpulseSum);
}

void WeldJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Linear velocity constraint
    Vec3 linearJV = (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    Vec3 linearLambda = linearM * -(linearJV + linearBias + linearImpulseSum * s->gamma);

    // Angular velocity constraint
    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;

    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * s->gamma);

    ApplyImpulse(linearLambda, angularLambda);
    linearImpulseSum += linearLambda;
    angularImpulseSum += angularLambda;
}

void WeldJoint::ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda)
{
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * λ

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    sA->linearVelocity -= linearLambda * sA->invMass;
    sA->angularVelocity -= s->invIA * (Cross(ra, linearLambda) + angularLambda);
    sB->linearVelocity += linearLambda * sB->invMass;
    sB->angularVelocity += s->invIB * (Cross(rb, linearLambda) + angularLambda);
}

} // namespace muli3
