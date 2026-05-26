#include "muli3/contact_solver.h"
#include "muli3/frame.h"
#include "muli3/joint.h"
#include "muli3/rigidbody.h"
#include "muli3/solver_states.h"

namespace muli3
{

static void PrepareNormalContact(SolverContact* n, ContactState* s, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec3 normal = s->manifold.contactNormal;
    Vec3 point = s->manifold.contactPoints[index].p;
    Vec3 ra = point - sA->motion.c;
    Vec3 rb = point - sB->motion.c;

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

static void SolveNormalContact(SolverContact* n, ContactState* s)
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

    sA->linearVelocity += n->j.va * (sA->invMass * lambda);
    sA->angularVelocity += s->invIA * n->j.wa * lambda;
    sB->linearVelocity += n->j.vb * (sB->invMass * lambda);
    sB->angularVelocity += s->invIB * n->j.wb * lambda;
}

static void PrepareTangentContact(SolverContact* t, ContactState* s, const Vec3& tangent, uint8 tangentIndex, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 point = s->manifold.contactPoints[index].p;
    Vec3 ra = point - sA->motion.c;
    Vec3 rb = point - sB->motion.c;

    t->j.va = -tangent;
    t->j.wa = -Cross(ra, tangent);
    t->j.vb = tangent;
    t->j.wb = Cross(rb, tangent);

    t->bias = -s->surfaceSpeed[tangentIndex];

    float k = sA->invMass + Dot(t->j.wa, s->invIA * t->j.wa) + sB->invMass + Dot(t->j.wb, s->invIB * t->j.wb);
    t->m = k > 0.0f ? 1.0f / k : 0.0f;
}

static void SolveTangentContact(SolverContact* t, ContactState* s, const SolverContact* n)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 * -(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(t->j.va, sA->linearVelocity)
             + Dot(t->j.wa, sA->angularVelocity)
             + Dot(t->j.vb, sB->linearVelocity)
             + Dot(t->j.wb, sB->angularVelocity);
    // clang-format on

    float lambda = t->m * -(jv + t->bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = t->impulse;
    float maxFriction = s->friction * n->impulse;
    t->impulse = Clamp(t->impulse + lambda, -maxFriction, maxFriction);
    lambda = t->impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * λ

    sA->linearVelocity += t->j.va * (sA->invMass * lambda);
    sA->angularVelocity += s->invIA * t->j.wa * lambda;
    sB->linearVelocity += t->j.vb * (sB->invMass * lambda);
    sB->angularVelocity += s->invIB * t->j.wb * lambda;
}

static void WarmStartSolverContact(SolverContact* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Warm start
    sA->linearVelocity += n->j.va * (sA->invMass * n->impulse);
    sA->angularVelocity += s->invIA * n->j.wa * n->impulse;
    sB->linearVelocity += n->j.vb * (sB->invMass * n->impulse);
    sB->angularVelocity += s->invIB * n->j.wb * n->impulse;
}

static void PreparePosition(SolverPosition* p, ContactState* s, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    p->localPlanePoint = qA.RotateInv(s->manifold.referencePoint.p - comA);
    p->localClipPoint = qB.RotateInv(s->manifold.contactPoints[index].p - comB);
    p->localNormal = qA.RotateInv(s->manifold.contactNormal);
}

static bool SolvePosition(SolverPosition* p, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    Vec3 planePoint = qA.Rotate(p->localPlanePoint) + comA;
    Vec3 clipPoint = qB.Rotate(p->localClipPoint) + comB;
    Vec3 normal = qA.Rotate(p->localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec3 ra = clipPoint - comA;
    Vec3 rb = clipPoint - comB;

    Vec3 ran = Cross(ra, normal);
    Vec3 rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k
    float k = sA->invMass
            + Dot(ran, s->invIA * ran)
            + sB->invMass
            + Dot(rbn, s->invIB * rbn);
    // clang-format on

    // Constraint (bias)
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec3 impulse = normal * lambda;

    s->cLinearImpulseA -= impulse;
    s->cAngularImpulseA -= Cross(ra, impulse);
    s->cLinearImpulseB += impulse;
    s->cAngularImpulseB += Cross(rb, impulse);

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

void PrepareContact(ContactState* s)
{
    s->invIA = s->s1->body->GetWorldInverseInertiaTensor();
    s->invIB = s->s2->body->GetWorldInverseInertiaTensor();

    Vec3 tangent1 = GramSchmidt(x_axis, s->manifold.contactNormal);
    if (tangent1.Normalize() == 0)
    {
        tangent1 = Normalize(GramSchmidt(z_axis, s->manifold.contactNormal));
    }
    Vec3 tangent2 = Cross(s->manifold.contactNormal, tangent1);

    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        PrepareNormalContact(s->normalContact + i, s, i);
        PrepareTangentContact(s->tangentContact1 + i, s, tangent1, 0, i);
        PrepareTangentContact(s->tangentContact2 + i, s, tangent2, 1, i);
        PreparePosition(s->positionContact + i, s, i);
    }
}

void WarmStartContact(ContactState* s)
{
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        WarmStartSolverContact(s->normalContact + i, s);
        WarmStartSolverContact(s->tangentContact1 + i, s);
        WarmStartSolverContact(s->tangentContact2 + i, s);
    }
}

void SolveContactVelocityConstraints(ContactState* s)
{
    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        SolveTangentContact(s->tangentContact1 + i, s, s->normalContact + i);
        SolveTangentContact(s->tangentContact2 + i, s, s->normalContact + i);
    }

    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        SolveNormalContact(s->normalContact + i, s);
    }
}

bool SolveContactPositionConstraints(ContactState* s)
{
    bool solved = true;

    s->cLinearImpulseA.SetZero();
    s->cLinearImpulseB.SetZero();
    s->cAngularImpulseA.SetZero();
    s->cAngularImpulseB.SetZero();

    s->invIA = s->s1->body->GetWorldInverseInertiaTensor();
    s->invIB = s->s2->body->GetWorldInverseInertiaTensor();

    for (int32 i = 0; i < s->manifold.contactCount; ++i)
    {
        solved &= SolvePosition(s->positionContact + i, s);
    }

    BodyState* bodySimA = s->s1;
    BodyState* bodySimB = s->s2;

    bodySimA->motion.c += bodySimA->invMass * s->cLinearImpulseA;
    Vec3 angularCorrectionA = s->invIA * s->cAngularImpulseA;
    Quat w1{ angularCorrectionA, 0.0f };
    bodySimA->motion.q = bodySimA->motion.q + (w1 * bodySimA->motion.q) * 0.5f;
    bodySimA->motion.q.Normalize();

    bodySimB->motion.c += bodySimB->invMass * s->cLinearImpulseB;
    Vec3 angularCorrectionB = s->invIB * s->cAngularImpulseB;
    Quat w2{ angularCorrectionB, 0.0f };
    bodySimB->motion.q = bodySimB->motion.q + (w2 * bodySimB->motion.q) * 0.5f;
    bodySimB->motion.q.Normalize();

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
