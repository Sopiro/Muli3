#include "muli3/line_joint.h"
#include "muli3/frame.h"

namespace muli3
{

LineJoint::LineJoint(RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio)
    : Joint(line_joint, bodyA, bodyB, frequency, dampingRatio)
    , impulseSum{ 0.0f }
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
}

void LineJoint::Prepare(const Timestep& step)
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

    Mat2 k;
    k[0][0] = sA->invMass + sB->invMass + Dot(sa1, s->invIA * sa1) + Dot(sb1, s->invIB * sb1);
    k[1][1] = sA->invMass + sB->invMass + Dot(sa2, s->invIA * sa2) + Dot(sb2, s->invIB * sb2);
    k[0][1] = Dot(sa1, s->invIA * sa2) + Dot(sb1, s->invIB * sb2);
    k[1][0] = k[0][1];

    ComputeBetaAndGamma(k.TraceInverse() / 2.0f, step.dt);

    k[0][0] += s->gamma;
    k[1][1] += s->gamma;

    m = k.GetInverse();

    bias.Set(Dot(d, t1), Dot(d, t2));
    bias *= s->beta * step.inv_dt;
}

void LineJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void LineJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 dv = sB->linearVelocity - sA->linearVelocity;
    Vec2 jv;
    jv.x = Dot(t1, dv) + Dot(sb1, sB->angularVelocity) - Dot(sa1, sA->angularVelocity);
    jv.y = Dot(t2, dv) + Dot(sb2, sB->angularVelocity) - Dot(sa2, sA->angularVelocity);

    Vec2 lambda = Mul(m, -(jv + bias + impulseSum * s->gamma));

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void LineJoint::ApplyImpulse(const Vec2& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = t1 * lambda.x + t2 * lambda.y;

    if (!bodyA->IsStatic())
    {
        sA->linearVelocity -= p * sA->invMass;
        sA->angularVelocity -= s->invIA * (sa1 * lambda.x + sa2 * lambda.y);
    }
    if (!bodyB->IsStatic())
    {
        sB->linearVelocity += p * sB->invMass;
        sB->angularVelocity += s->invIB * (sb1 * lambda.x + sb2 * lambda.y);
    }
}

} // namespace muli3
