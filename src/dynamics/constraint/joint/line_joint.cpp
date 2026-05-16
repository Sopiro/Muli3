#include "muli3/line_joint.h"
#include "muli3/frame.h"

namespace muli3
{

LineJoint::LineJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(line_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
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

    invIA = bodyA->GetWorldInverseInertiaTensor();
    invIB = bodyB->GetWorldInverseInertiaTensor();

    Mat2 k;
    k[0][0] = bodyA->invMass + bodyB->invMass + Dot(sa1, invIA * sa1) + Dot(sb1, invIB * sb1);
    k[1][1] = bodyA->invMass + bodyB->invMass + Dot(sa2, invIA * sa2) + Dot(sb2, invIB * sb2);
    k[0][1] = Dot(sa1, invIA * sa2) + Dot(sb1, invIB * sb2);
    k[1][0] = k[0][1];

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    bias.Set(Dot(d, t1), Dot(d, t2));
    bias *= beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void LineJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    Vec3 dv = bodyB->linearVelocity - bodyA->linearVelocity;
    Vec2 jv;
    jv.x = Dot(t1, dv) + Dot(sb1, bodyB->angularVelocity) - Dot(sa1, bodyA->angularVelocity);
    jv.y = Dot(t2, dv) + Dot(sb2, bodyB->angularVelocity) - Dot(sa2, bodyA->angularVelocity);

    Vec2 lambda = Mul(m, -(jv + bias + impulseSum * gamma));

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void LineJoint::ApplyImpulse(const Vec2& lambda)
{
    Vec3 p = t1 * lambda.x + t2 * lambda.y;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= invIA * (sa1 * lambda.x + sa2 * lambda.y);
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += invIB * (sb1 * lambda.x + sb2 * lambda.y);
}

} // namespace muli3
