#include "muli3/line_joint.h"
#include "muli3/frame.h"

namespace muli3
{

LineJoint::LineJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio)
    : Joint(line_joint, bodyA, bodyB, frequency, dampingRatio)
    , impulseSum{ 0.0f }
    , beta{ 0.0f }
    , gamma{ 0.0f }
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

    // The point on B may move only along worldAxis. The two scalar constraints
    // are Ci = dot(ti, pb - pa) = 0. Because ti rotates with body A,
    // Cidot = dot(ti, vb - va) + dot(sb_i, wb) - dot(sa_i, wa), where
    // sa_i = (ra + d) x ti and sb_i = rb x ti.
    sa1 = Cross(ra0 + d, t1);
    sb1 = Cross(rb0, t1);
    sa2 = Cross(ra0 + d, t2);
    sb2 = Cross(rb0, t2);

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    // Each Jacobian row is Ji = [-ti, -sa_i, ti, sb_i].
    // The 2x2 matrix below is K = J * M^-1 * J^T, including coupling between t1 and t2.
    Mat2 k;
    k[0][0] = sA->invMass + sB->invMass + Dot(sa1, s->invIA * sa1) + Dot(sb1, s->invIB * sb1);
    k[1][1] = sA->invMass + sB->invMass + Dot(sa2, s->invIA * sa2) + Dot(sb2, s->invIB * sb2);
    k[0][1] = Dot(sa1, s->invIA * sa2) + Dot(sb1, s->invIB * sb2);
    k[1][0] = k[0][1];

    ComputeBetaAndGamma(&beta, &gamma, frequency, dampingRatio, k.TraceInverse() / 2.0f, step.dt);

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    bias.Set(Dot(d, t1), Dot(d, t2));
    bias *= beta * step.inv_dt;
}

void LineJoint::WarmStart()
{
    ApplyImpulse(impulseSum);
}

void LineJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 dv = sB->linearVelocity - sA->linearVelocity;
    Vec2 jv;
    jv.x = Dot(t1, dv) + Dot(sb1, sB->angularVelocity) - Dot(sa1, sA->angularVelocity);
    jv.y = Dot(t2, dv) + Dot(sb2, sB->angularVelocity) - Dot(sa2, sA->angularVelocity);

    Vec2 lambda = Mul(m, -(jv + bias + impulseSum * gamma));

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void LineJoint::ApplyImpulse(const Vec2& lambda)
{
    JointState* s = GetJointState();
    BodyState* sA = bodyA->GetBodyState();
    BodyState* sB = bodyB->GetBodyState();

    Vec3 p = t1 * lambda.x + t2 * lambda.y;

    if (sA->invMass > 0.0f)
    {
        sA->linearVelocity -= p * sA->invMass;
        sA->angularVelocity -= s->invIA * (sa1 * lambda.x + sa2 * lambda.y);
    }
    if (sB->invMass > 0.0f)
    {
        sB->linearVelocity += p * sB->invMass;
        sB->angularVelocity += s->invIB * (sb1 * lambda.x + sb2 * lambda.y);
    }
}

} // namespace muli3
