#include "muli3/prismatic_joint.h"
#include "muli3/frame.h"

namespace muli3
{

PrismaticJoint::PrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio
)
    : Joint(prismatic_joint, bodyA, bodyB, frequency, dampingRatio)
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
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 ra0 = bodyA->GetRotation().Rotate(localAnchorA - bodyA->GetLocalCenter());
    Vec3 rb0 = bodyB->GetRotation().Rotate(localAnchorB - bodyB->GetLocalCenter());
    Vec3 pa = sA->motion.c + ra0;
    Vec3 pb = sB->motion.c + rb0;
    Vec3 d = pb - pa;

    Vec3 worldAxis = bodyA->GetRotation().Rotate(localAxis);
    CoordinateSystem(worldAxis, &t1, &t2);

    sa1 = Cross(ra0 + d, t1);
    sb1 = Cross(rb0, t1);
    sa2 = Cross(ra0 + d, t2);
    sb2 = Cross(rb0, t2);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Linear part (2 DOF)
    Mat2 lk;
    lk[0][0] = sA->invMass + sB->invMass + Dot(sa1, s->invIA * sa1) + Dot(sb1, s->invIB * sb1);
    lk[1][1] = sA->invMass + sB->invMass + Dot(sa2, s->invIA * sa2) + Dot(sb2, s->invIB * sb2);
    lk[0][1] = Dot(sa1, s->invIA * sa2) + Dot(sb1, s->invIB * sb2);
    lk[1][0] = lk[0][1];

    ComputeBetaAndGamma(lk.TraceInverse() / 2.0f, step.dt);

    lk[0][0] += s->gamma;
    lk[1][1] += s->gamma;

    linearM = lk.GetInverse();

    // Angular part (3 DOF)
    Mat3 ak = s->invIA + s->invIB;
    ak.ex.x += s->gamma;
    ak.ey.y += s->gamma;
    ak.ez.z += s->gamma;

    angularM = ak.GetInverse();

    // Linear bias
    linearBias.Set(Dot(d, t1), Dot(d, t2));
    linearBias *= s->beta * step.inv_dt;

    // Angular bias
    Quat qTarget = sA->motion.q * orientationOffset;
    Quat qError = sB->motion.q * qTarget.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * s->beta * step.inv_dt;
}

void PrismaticJoint::WarmStart()
{
    ApplyImpulse(linearImpulseSum, angularImpulseSum);
}

void PrismaticJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Linear
    Vec3 dv = sB->linearVelocity - sA->linearVelocity;
    Vec2 linearJV;
    linearJV.x = Dot(t1, dv) + Dot(sb1, sB->angularVelocity) - Dot(sa1, sA->angularVelocity);
    linearJV.y = Dot(t2, dv) + Dot(sb2, sB->angularVelocity) - Dot(sa2, sA->angularVelocity);

    Vec2 linearLambda = Mul(linearM, -(linearJV + linearBias + linearImpulseSum * s->gamma));

    // Angular
    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * s->gamma);

    ApplyImpulse(linearLambda, angularLambda);
    linearImpulseSum += linearLambda;
    angularImpulseSum += angularLambda;
}

void PrismaticJoint::ApplyImpulse(const Vec2& linearLambda, const Vec3& angularLambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = t1 * linearLambda.x + t2 * linearLambda.y;

    sA->linearVelocity -= p * sA->invMass;
    sA->angularVelocity -= s->invIA * (sa1 * linearLambda.x + sa2 * linearLambda.y + angularLambda);
    sB->linearVelocity += p * sB->invMass;
    sB->angularVelocity += s->invIB * (sb1 * linearLambda.x + sb2 * linearLambda.y + angularLambda);
}

} // namespace muli3
