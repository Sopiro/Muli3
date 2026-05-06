#include "muli3/prismatic_joint.h"
#include "muli3/frame.h"

namespace muli3
{

PrismaticJoint::PrismaticJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(prismatic_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , linearImpulseSum{ 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);

    if (Length2(dir) < epsilon)
    {
        localAxis = bodyA->GetRotation().RotateInv(Normalize(bodyB->GetPosition() - bodyA->GetPosition()));
    }
    else
    {
        localAxis = bodyA->GetRotation().RotateInv(Normalize(dir));
    }

    orientationOffset = bodyA->GetRotation().GetConjugate() * bodyB->GetRotation();
}

void PrismaticJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    Vec3 ra0 = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    Vec3 rb0 = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());
    Vec3 pa = bodyA->motion.c + ra0;
    Vec3 pb = bodyB->motion.c + rb0;
    Vec3 d = pb - pa;

    Vec3 worldAxis = bodyA->GetRotation().Rotate(localAxis);
    CoordinateSystem(worldAxis, &t1, &t2);

    sa1 = Cross(ra0 + d, t1);
    sb1 = Cross(rb0, t1);
    sa2 = Cross(ra0 + d, t2);
    sb2 = Cross(rb0, t2);

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    // Linear part (2 DOF)
    Mat2 lk;
    lk[0][0] = bodyA->invMass + bodyB->invMass + Dot(sa1, invIA * sa1) + Dot(sb1, invIB * sb1);
    lk[1][1] = bodyA->invMass + bodyB->invMass + Dot(sa2, invIA * sa2) + Dot(sb2, invIB * sb2);
    lk[0][1] = Dot(sa1, invIA * sa2) + Dot(sb1, invIB * sb2);
    lk[1][0] = lk[0][1];

    lk[0][0] += gamma;
    lk[1][1] += gamma;

    linearM = lk.GetInverse();

    // Angular part (3 DOF)
    Mat3 ak = invIA + invIB;
    ak.ex.x += gamma;
    ak.ey.y += gamma;
    ak.ez.z += gamma;

    angularM = ak.GetInverse();

    // Linear bias
    linearBias.Set(Dot(d, t1), Dot(d, t2));
    linearBias *= beta * step.inv_dt;

    // Angular bias
    Quat qTarget = bodyA->motion.q * orientationOffset;
    Quat qError = bodyB->motion.q * qTarget.GetConjugate();
    if (qError.w < 0.0f) qError = -qError;
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(linearImpulseSum, angularImpulseSum);
    }
}

void PrismaticJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Linear
    Vec3 dv = bodyB->linearVelocity - bodyA->linearVelocity;
    Vec2 linearJV;
    linearJV.x = Dot(t1, dv) + Dot(sb1, bodyB->angularVelocity) - Dot(sa1, bodyA->angularVelocity);
    linearJV.y = Dot(t2, dv) + Dot(sb2, bodyB->angularVelocity) - Dot(sa2, bodyA->angularVelocity);

    Vec2 linearLambda = Mul(linearM, -(linearJV + linearBias + linearImpulseSum * gamma));

    // Angular
    Vec3 angularJV = bodyB->angularVelocity - bodyA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * gamma);

    ApplyImpulse(linearLambda, angularLambda);
    linearImpulseSum += linearLambda;
    angularImpulseSum += angularLambda;
}

void PrismaticJoint::ApplyImpulse(const Vec2& linearLambda, const Vec3& angularLambda)
{
    Vec3 p = t1 * linearLambda.x + t2 * linearLambda.y;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->GetWorldInverseInertiaTensor() * (sa1 * linearLambda.x + sa2 * linearLambda.y + angularLambda);
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * (sb1 * linearLambda.x + sb2 * linearLambda.y + angularLambda);
}

} // namespace muli3
