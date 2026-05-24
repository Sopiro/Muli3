#include "muli3/contact_solver.h"
#include "muli3/settings.h"
#include "muli3/solver_states.h"

namespace muli3
{

void ContactSolverNormal::Prepare(ContactState* cs, int32 index, const Timestep& step)
{
    BodyState* sA = cs->s1;
    BodyState* sB = cs->s2;

    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec3 normal = cs->manifold.contactNormal;
    Vec3 point = cs->manifold.contactPoints[index].p;
    Vec3 ra = point - sA->motion.c;
    Vec3 rb = point - sB->motion.c;

    // Setup jacobian
    j.va = -normal;
    j.wa = -Cross(ra, normal);
    j.vb = normal;
    j.wb = Cross(rb, normal);

    bias = 0.0f;

    // Relative velocity at contact point
    Vec3 relativeVelocity =
        (sB->linearVelocity + Cross(sB->angularVelocity, rb)) - (sA->linearVelocity + Cross(sA->angularVelocity, ra));

    // Normal velocity == velocity constraint: jv
    float normalVelocity = Dot(normal, relativeVelocity);

    if (-normalVelocity > cs->restitutionThreshold)
    {
        bias = cs->restitution * normalVelocity;
    }

    // clang-format off
    float k = sA->invMass
            + Dot(j.wa, cs->invIA * j.wa)
            + sB->invMass
            + Dot(j.wb, cs->invIB * j.wb);
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        sA->linearVelocity += j.va * (sA->invMass * impulse);
        sA->angularVelocity += cs->invIA * j.wa * impulse;
        sB->linearVelocity += j.vb * (sB->invMass * impulse);
        sB->angularVelocity += cs->invIB * j.wb * impulse;
    }
}

void ContactSolverNormal::Solve(ContactState* cs)
{
    BodyState* sA = cs->s1;
    BodyState* sB = cs->s2;

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ??-(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(j.va, sA->linearVelocity)
             + Dot(j.wa, sA->angularVelocity)
             + Dot(j.vb, sB->linearVelocity)
             + Dot(j.wb, sB->angularVelocity);
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    impulse = Max(0.0f, impulse + lambda);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ??Pc
    // Pc = J^t ??λ

    sA->linearVelocity += j.va * (sA->invMass * lambda);
    sA->angularVelocity += cs->invIA * j.wa * lambda;
    sB->linearVelocity += j.vb * (sB->invMass * lambda);
    sB->angularVelocity += cs->invIB * j.wb * lambda;
}

void ContactSolverTangent::Prepare(ContactState* s, const Vec3& tangent, uint8 tangentIndex, int32 index, const Timestep& step)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute Jacobian J and effective mass W
    // J = [-t, -ra × t, t, rb × t]
    // W = (J · M^-1 · J^t)^-1

    Vec3 point = s->manifold.contactPoints[index].p;
    Vec3 ra = point - sA->motion.c;
    Vec3 rb = point - sB->motion.c;

    // Setup jacobian
    j.va = -tangent;
    j.wa = -Cross(ra, tangent);
    j.vb = tangent;
    j.wb = Cross(rb, tangent);

    bias = -s->surfaceSpeed[tangentIndex];

    // clang-format off
    float k = sA->invMass
            + Dot(j.wa, s->invIA * j.wa)
            + sB->invMass
            + Dot(j.wb, s->invIB * j.wb);
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        sA->linearVelocity += j.va * (sA->invMass * impulse);
        sA->angularVelocity += s->invIA * j.wa * impulse;
        sB->linearVelocity += j.vb * (sB->invMass * impulse);
        sB->angularVelocity += s->invIB * j.wb * impulse;
    }
}

void ContactSolverTangent::Solve(ContactState* s, const ContactSolverNormal* normalSolver)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ??-(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(j.va, sA->linearVelocity)
             + Dot(j.wa, sA->angularVelocity)
             + Dot(j.vb, sB->linearVelocity)
             + Dot(j.wb, sB->angularVelocity);
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    float maxFriction = s->friction * normalSolver->impulse;
    impulse = Clamp(impulse + lambda, -maxFriction, maxFriction);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ??Pc
    // Pc = J^t ??λ

    sA->linearVelocity += j.va * (sA->invMass * lambda);
    sA->angularVelocity += s->invIA * j.wa * lambda;
    sB->linearVelocity += j.vb * (sB->invMass * lambda);
    sB->angularVelocity += s->invIB * j.wb * lambda;
}

} // namespace muli3
