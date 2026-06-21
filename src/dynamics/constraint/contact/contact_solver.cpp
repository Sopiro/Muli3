#include "muli3/contact_solver.h"
#include "muli3/body.h"
#include "muli3/contact.h"
#include "muli3/frame.h"
#include "muli3/joints.h" // IWYU pragma: keep
#include "muli3/solver_states.h"

namespace muli3
{

static void PrepareNormalContact(SolverNormalContact* n, ContactState* s, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec3 normal = s->manifold.contactPoints[index].normal;
    Vec3 ra = s->manifold.contactPoints[index].anchorA - sA->motion.c;
    Vec3 rb = s->manifold.contactPoints[index].anchorB - sB->motion.c;

    // Setup jacobian
    n->j.va = -normal;
    n->j.wa = -Cross(ra, normal);
    n->j.vb = normal;
    n->j.wb = Cross(rb, normal);

    n->bias = 0.0f;

    // Relative velocity at contact point
    Vec3 relativeVelocity =
        (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    // Normal velocity == velocity constraint: jv
    float normalVelocity = Dot(normal, relativeVelocity);

    if (-normalVelocity > s->restitutionThreshold)
    {
        n->bias = s->restitution * normalVelocity;
    }

    float k = sA->invMass + Dot(n->j.wa, s->invIA * n->j.wa) + sB->invMass + Dot(n->j.wb, s->invIB * n->j.wb);
    n->m = k > 0.0f ? 1.0f / k : 0.0f;
}

static void SolveNormalContact(SolverNormalContact* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ??-(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(n->j.va, sA->linearVelocity)
             + Dot(n->j.wa, sA->angularVelocity)
             + Dot(n->j.vb, sB->linearVelocity)
             + Dot(n->j.wb, sB->angularVelocity);
    // clang-format on

    float lambda = n->m * -(jv + n->bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = n->impulse;
    n->impulse = Max(0.0f, n->impulse + lambda);
    lambda = n->impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * λ
    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += n->j.va * (sA->invMass * lambda);
        sA->angularVelocity += s->invIA * n->j.wa * lambda;
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += n->j.vb * (sB->invMass * lambda);
        sB->angularVelocity += s->invIB * n->j.wb * lambda;
    }
}

static void PrepareTangentContact(ContactState* s, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 ra = s->manifold.contactPoints[index].p - sA->motion.c;
    Vec3 rb = s->manifold.contactPoints[index].p - sB->motion.c;
    Vec3 normal = s->manifold.contactPoints[index].normal;

    Vec3 reference = Abs(normal.x) < Abs(normal.y) ? x_axis : y_axis;
    if (Abs(normal.z) < AbsDot(reference, normal))
    {
        reference = z_axis;
    }

    Vec3 tangent1 = Normalize(GramSchmidt(reference, normal));
    Vec3 tangent2 = Cross(normal, tangent1);

    SolverTangentContact* t = s->tangentContact + index;

    // Project the relative contact velocity onto two tangent directions.
    // J = [-t, -(ra x t), t, rb x t]
    t->j1.va = -tangent1;
    t->j1.wa = -Cross(ra, tangent1);
    t->j1.vb = tangent1;
    t->j1.wb = Cross(rb, tangent1);

    t->j2.va = -tangent2;
    t->j2.wa = -Cross(ra, tangent2);
    t->j2.vb = tangent2;
    t->j2.wb = Cross(rb, tangent2);

    t->bias = -s->surfaceSpeed;

    // K = J M^-1 J^T
    float k11 = sA->invMass + Dot(t->j1.wa, s->invIA * t->j1.wa) + sB->invMass + Dot(t->j1.wb, s->invIB * t->j1.wb);
    float k12 = Dot(t->j1.wa, s->invIA * t->j2.wa) + Dot(t->j1.wb, s->invIB * t->j2.wb);
    float k22 = sA->invMass + Dot(t->j2.wa, s->invIA * t->j2.wa) + sB->invMass + Dot(t->j2.wb, s->invIB * t->j2.wb);
    Mat2 k = Mat2(Vec2(k11, k12), Vec2(k12, k22));

    t->m = k.GetInverse();
}

static Vec2 ComputeTangentImpulse(Vec2* tangentVelocity, const SolverTangentContact* t, const ContactState* s)
{
    const BodyState* sA = s->s1;
    const BodyState* sB = s->s2;

    // Solve both tangent axes as single 2D constraint.

    float jv1 = Dot(t->j1.va, sA->linearVelocity) + Dot(t->j1.wa, sA->angularVelocity) + Dot(t->j1.vb, sB->linearVelocity) +
                Dot(t->j1.wb, sB->angularVelocity);
    float jv2 = Dot(t->j2.va, sA->linearVelocity) + Dot(t->j2.wa, sA->angularVelocity) + Dot(t->j2.vb, sB->linearVelocity) +
                Dot(t->j2.wb, sB->angularVelocity);

    tangentVelocity->Set(jv1 + t->bias.x, jv2 + t->bias.y);

    Vec2 deltaLambda = -Mul(t->m, *tangentVelocity);

    // The unconstrained accumulated impulse that would stop tangent motion is
    // lambda_new = lambda_old + delta_lambda.
    return t->impulse + deltaLambda;
}

static void SolveTangentContact(SolverTangentContact* t, ContactState* s, const SolverNormalContact* n)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec2 tangentVelocity;
    Vec2 impulse = ComputeTangentImpulse(&tangentVelocity, t, s);

    // Coulomb friction limits the accumulated 2D tangent impulse lambda_t by |lambda_t| <= mu * lambda_n.
    // A larger normal impulse lets the contact provide more friction.
    // maxFriction = mu * lambda_n
    float maxFriction = s->friction * n->impulse;

    // impulse is the lambda_t needed to make the tangent velocity zero.
    // Test |lambda_t|^2 > (mu * lambda_n)^2 to avoid a square root.
    // If it is inside the Coulomb circle, static friction can stop the contact and uses it as-is.
    float impulse2 = Dot(impulse, impulse);
    if (impulse2 > Sqr(maxFriction))
    {
        impulse *= maxFriction / std::sqrt(impulse2);
    }

    // Only apply the change from the previously accumulated 2D impulse.
    Vec2 deltaLambda = impulse - t->impulse;
    t->impulse = impulse;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += (t->j1.va * deltaLambda.x + t->j2.va * deltaLambda.y) * sA->invMass;
        sA->angularVelocity += s->invIA * (t->j1.wa * deltaLambda.x + t->j2.wa * deltaLambda.y);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += (t->j1.vb * deltaLambda.x + t->j2.vb * deltaLambda.y) * sB->invMass;
        sB->angularVelocity += s->invIB * (t->j1.wb * deltaLambda.x + t->j2.wb * deltaLambda.y);
    }
}

static void WarmStartNormalContact(SolverNormalContact* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += n->j.va * (sA->invMass * n->impulse);
        sA->angularVelocity += s->invIA * (n->j.wa * n->impulse);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += n->j.vb * (sB->invMass * n->impulse);
        sB->angularVelocity += s->invIB * (n->j.wb * n->impulse);
    }
}

static void WarmStartTangentContact(SolverTangentContact* t, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += (t->j1.va * t->impulse.x + t->j2.va * t->impulse.y) * sA->invMass;
        sA->angularVelocity += s->invIA * (t->j1.wa * t->impulse.x + t->j2.wa * t->impulse.y);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += (t->j1.vb * t->impulse.x + t->j2.vb * t->impulse.y) * sB->invMass;
        sB->angularVelocity += s->invIB * (t->j1.wb * t->impulse.x + t->j2.wb * t->impulse.y);
    }
}

static void PreparePosition(SolverPosition* p, ContactState* s, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    p->localPointA = qA.RotateInv(s->manifold.contactPoints[index].anchorA - comA);
    p->localPointB = qB.RotateInv(s->manifold.contactPoints[index].anchorB - comB);
    p->localNormal = qA.RotateInv(s->manifold.contactPoints[index].normal);
}

struct PositionCorrection
{
    Vec3 linearImpulseA, linearImpulseB;
    Vec3 angularImpulseA, angularImpulseB;
};

static bool SolvePosition(PositionCorrection* r, const SolverPosition* p, const ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    Vec3 pointA = qA.Rotate(p->localPointA) + comA;
    Vec3 pointB = qB.Rotate(p->localPointB) + comB;
    Vec3 normal = qA.Rotate(p->localNormal);

    float separation = Dot(pointB - pointA, normal);

    Vec3 ra = pointA - comA;
    Vec3 rb = pointB - comB;

    Vec3 ran = Cross(ra, normal);
    Vec3 rbn = Cross(rb, normal);

    float k = sA->invMass + Dot(ran, s->invIA * ran) + sB->invMass + Dot(rbn, s->invIB * rbn);
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec3 impulse = normal * lambda;

    r->linearImpulseA -= impulse;
    r->angularImpulseA -= Cross(ra, impulse);
    r->linearImpulseB += impulse;
    r->angularImpulseB += Cross(rb, impulse);

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

void PrepareContact(ContactState* s)
{
    Contact* contact = s->contact;

    Body* bodyA = contact->GetBodyA();
    Body* bodyB = contact->GetBodyB();

    s->s1 = bodyA->GetBodyState();
    s->s2 = bodyB->GetBodyState();

    s->invIA = s->s1->body->GetWorldInverseInertiaTensor();
    s->invIB = s->s2->body->GetWorldInverseInertiaTensor();

    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        PrepareTangentContact(s, i);
        PrepareNormalContact(s->normalContact + i, s, i);
        PreparePosition(s->positionContact + i, s, i);
    }
}

void WarmStartContact(ContactState* s)
{
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        WarmStartTangentContact(s->tangentContact + i, s);
    }
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        WarmStartNormalContact(s->normalContact + i, s);
    }
}

void SolveContactVelocityConstraints(ContactState* s)
{
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        SolveTangentContact(s->tangentContact + i, s, s->normalContact + i);
    }
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        SolveNormalContact(s->normalContact + i, s);
    }
}

bool SolveContactPositionConstraints(ContactState* s)
{
    bool solved = true;

    PositionCorrection correction{};

    Body* bodyA = s->s1->body;
    Body* bodyB = s->s2->body;

    s->invIA = bodyA->GetWorldInverseInertiaTensor();
    s->invIB = bodyB->GetWorldInverseInertiaTensor();

    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        solved &= SolvePosition(&correction, s->positionContact + i, s);
    }

    BodyState* s1 = s->s1;
    BodyState* s2 = s->s2;

    if (!bodyA->IsStatic())
    {
        s1->motion.c += s1->invMass * correction.linearImpulseA;
        Vec3 angularCorrectionA = s->invIA * correction.angularImpulseA;
        Quat w1{ angularCorrectionA, 0.0f };
        s1->motion.q = s1->motion.q + (w1 * s1->motion.q) * 0.5f;
        s1->motion.q.Normalize();
    }

    if (!bodyB->IsStatic())
    {
        s2->motion.c += s2->invMass * correction.linearImpulseB;
        Vec3 angularCorrectionB = s->invIB * correction.angularImpulseB;
        Quat w2{ angularCorrectionB, 0.0f };
        s2->motion.q = s2->motion.q + (w2 * s2->motion.q) * 0.5f;
        s2->motion.q.Normalize();
    }

    return solved;
}

void PrepareJoint(JointState* s, const Timestep& step)
{
    s->joint->Prepare(step);
}

void WarmStartJoint(JointState* s)
{
    s->joint->WarmStart();
}

void SolveJointVelocityConstraints(JointState* s, const Timestep& step)
{
    s->joint->SolveVelocityConstraints(step);
}

bool SolveJointPositionConstraints(JointState* s, const Timestep& step)
{
    return s->joint->SolvePositionConstraints(step);
}

} // namespace muli3
