#include "muli3/contact_solver.h"
#include "muli3/contact.h"

namespace muli3
{

void ContactSolverNormal::Prepare(Contact* c, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass W
    // J = [-n, -ra × n, n, rb × n]
    // W = (J · M^-1 · J^t)^-1

    Vec3 normal = c->manifold.contactNormal;
    Vec3 point = c->manifold.contactPoints[index].p;
    Vec3 ra = point - c->b1->GetWorldCenterOfMass();
    Vec3 rb = point - c->b2->GetWorldCenterOfMass();

    Mat3 iiA = c->b1->GetInverseInertiaTensorWorld();
    Mat3 iiB = c->b2->GetInverseInertiaTensorWorld();

    // Setup jacobian
    j.va = -normal;
    j.wa = -Cross(ra, normal);
    j.vb = normal;
    j.wb = Cross(rb, normal);

    bias = 0.0f;

    // Relative velocity at contact point
    Vec3 relativeVelocity =
        (c->b2->linearVelocity + Cross(c->b2->angularVelocity, rb)) -
        (c->b1->linearVelocity + Cross(c->b1->angularVelocity, ra));

    // Normal velocity == velocity constraint: jv
    float normalVelocity = Dot(normal, relativeVelocity);

    if (-normalVelocity > c->restitutionThreshold)
    {
        bias = c->restitution * normalVelocity;
    }

    // clang-format off
    float k = c->b1->invMass
            + Dot(j.wa, iiA * j.wa)
            + c->b2->invMass
            + Dot(j.wb, iiB * j.wb);
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += iiA * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += iiB * j.wb * impulse;
    }
}

void ContactSolverNormal::Solve(Contact* c)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Mat3 iiA = c->b1->GetInverseInertiaTensorWorld();
    Mat3 iiB = c->b2->GetInverseInertiaTensorWorld();

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(j.va, c->b1->linearVelocity)
             + Dot(j.wa, c->b1->angularVelocity)
             + Dot(j.vb, c->b2->linearVelocity)
             + Dot(j.wb, c->b2->angularVelocity);
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    impulse = Max(0.0f, impulse + lambda);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += iiA * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += iiB * j.wb * lambda;
}

void ContactSolverTangent::Prepare(Contact* c, const Vec3& tangent, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass W
    // J = [-t, -ra × t, t, rb × t]
    // W = (J · M^-1 · J^t)^-1

    Vec3 point = c->manifold.contactPoints[index].p;
    Vec3 ra = point - c->b1->GetWorldCenterOfMass();
    Vec3 rb = point - c->b2->GetWorldCenterOfMass();

    Mat3 iiA = c->b1->GetInverseInertiaTensorWorld();
    Mat3 iiB = c->b2->GetInverseInertiaTensorWorld();

    // Setup jacobian
    j.va = -tangent;
    j.wa = -Cross(ra, tangent);
    j.vb = tangent;
    j.wb = Cross(rb, tangent);

    bias = -c->surfaceSpeed;

    // clang-format off
    float k = c->b1->invMass
            + Dot(j.wa, iiA * j.wa)
            + c->b2->invMass
            + Dot(j.wb, iiB * j.wb);
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += iiA * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += iiB * j.wb * impulse;
    }
}

void ContactSolverTangent::Solve(Contact* c, const ContactSolverNormal* normalSolver)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Mat3 iiA = c->b1->GetInverseInertiaTensorWorld();
    Mat3 iiB = c->b2->GetInverseInertiaTensorWorld();

    // clang-format off
    // Velocity constraint: C' = jv
    float jv = Dot(j.va, c->b1->linearVelocity)
             + Dot(j.wa, c->b1->angularVelocity)
             + Dot(j.vb, c->b2->linearVelocity)
             + Dot(j.wb, c->b2->angularVelocity);
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    float maxFriction = c->friction * normalSolver->impulse;
    impulse = Clamp(impulse + lambda, -maxFriction, maxFriction);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += iiA * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += iiB * j.wb * lambda;
}

} // namespace muli3
