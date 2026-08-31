#include "muli3/frame.h"
#include "muli3/joints.h"


namespace muli3
{

PrismaticJoint::PrismaticJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio)
    : Joint(prismatic_joint, bodyA, bodyB, frequency, dampingRatio)
    , linearImpulseSum{ 0.0f }
    , linearBeta{ 0.0f }
    , linearGamma{ 0.0f }
    , angularImpulseSum{ 0.0f, 0.0f, 0.0f }
    , angularBeta{ 0.0f }
    , angularGamma{ 0.0f }
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

    // Translation along worldAxis is free. For Ci = dot(ti, pb - pa), the
    // rotating basis on A gives Ji = [-ti, -sa_i, ti, sb_i], where
    // sa_i = (ra + d) x ti and sb_i = rb x ti.
    sa1 = Cross(ra0 + d, t1);
    sb1 = Cross(rb0, t1);
    sa2 = Cross(ra0 + d, t2);
    sb2 = Cross(rb0, t2);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // The two transverse rows form the coupled 2x2 effective mass.
    Mat2 lk;
    lk[0][0] = sA->invMass + sB->invMass + Dot(sa1, s->invIA * sa1) + Dot(sb1, s->invIB * sb1);
    lk[1][1] = sA->invMass + sB->invMass + Dot(sa2, s->invIA * sa2) + Dot(sb2, s->invIB * sb2);
    lk[0][1] = Dot(sa1, s->invIA * sa2) + Dot(sb1, s->invIB * sb2);
    lk[1][0] = lk[0][1];

    ComputeBetaAndGamma(&linearBeta, &linearGamma, frequency, dampingRatio, lk.TraceInverse() / 2.0f, step.dt);

    lk[0][0] += linearGamma;
    lk[1][1] += linearGamma;

    linearM = lk.GetInverse();

    // Relative orientation is fixed: Cdot = wb - wa and
    // Ja = [0, -I, 0, I]. This removes all three rotational DOFs.
    Mat3 ak = s->invIA + s->invIB;
    ComputeBetaAndGamma(&angularBeta, &angularGamma, frequency, dampingRatio, ak.TraceInverse() / 3.0f, step.dt);
    ak.ex.x += angularGamma;
    ak.ey.y += angularGamma;
    ak.ez.z += angularGamma;

    angularM = ak.GetInverse();

    // Biases are beta / dt times the position-level constraints.
    linearBias.Set(Dot(d, t1), Dot(d, t2));
    linearBias *= linearBeta * step.inv_dt;

    Quat qTarget = sA->motion.q * orientationOffset;
    Quat qError = sB->motion.q * qTarget.GetConjugate();
    if (qError.w < 0.0f)
    {
        qError = -qError;
    }
    angularBias = Vec3{ qError.x, qError.y, qError.z } * 2.0f * angularBeta * step.inv_dt;
}

void PrismaticJoint::WarmStart()
{
    ApplyImpulse(linearImpulseSum, angularImpulseSum);
}

void PrismaticJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    // Evaluate J * V for the two transverse rows.
    Vec3 dv = sB->linearVelocity - sA->linearVelocity;
    Vec2 linearJV;
    linearJV.x = Dot(t1, dv) + Dot(sb1, sB->angularVelocity) - Dot(sa1, sA->angularVelocity);
    linearJV.y = Dot(t2, dv) + Dot(sb2, sB->angularVelocity) - Dot(sa2, sA->angularVelocity);

    Vec2 linearLambda = Mul(linearM, -(linearJV + linearBias + linearImpulseSum * linearGamma));

    // Evaluate the angular row block Ja * V.
    Vec3 angularJV = sB->angularVelocity - sA->angularVelocity;
    Vec3 angularLambda = angularM * -(angularJV + angularBias + angularImpulseSum * angularGamma);

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

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity -= p * sA->invMass;
        sA->angularVelocity -= s->invIA * (sa1 * linearLambda.x + sa2 * linearLambda.y + angularLambda);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += p * sB->invMass;
        sB->angularVelocity += s->invIB * (sb1 * linearLambda.x + sb2 * linearLambda.y + angularLambda);
    }
}

const Vec3& PrismaticJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const Vec3& PrismaticJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

const Quat& PrismaticJoint::GetOrientationOffset() const
{
    return orientationOffset;
}

} // namespace muli3
