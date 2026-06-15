#include "muli3/contact_solver.h"
#include "muli3/body.h"
#include "muli3/contact.h"
#include "muli3/frame.h"
#include "muli3/joints.h" // IWYU pragma: keep
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

static void PrepareTangentContact(SolverContact* t, ContactState* s, const Vec3& tangent, uint8 tangentIndex, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 point = s->manifold.contactPoints[index].p;
    Vec3 ra = point - sA->motion.c;
    Vec3 rb = point - sB->motion.c;

    // Project the relative contact velocity onto one tangent direction.
    // J = [-t, -(ra x t), t, rb x t]
    t->j.va = -tangent;
    t->j.wa = -Cross(ra, tangent);
    t->j.vb = tangent;
    t->j.wb = Cross(rb, tangent);

    t->bias = -s->surfaceSpeed[tangentIndex];

    float k = sA->invMass + Dot(t->j.wa, s->invIA * t->j.wa) + sB->invMass + Dot(t->j.wb, s->invIB * t->j.wb);
    t->m = k > 0.0f ? 1.0f / k : 0.0f;
}

static Vec2 ComputeTangentImpulse(Vec2* tangentVelocity, const SolverContact* t1, const SolverContact* t2, const ContactState* s)
{
    const BodyState* sA = s->s1;
    const BodyState* sB = s->s2;

    // Solve both tangent axes as single 2D constraint.

    float jv1 = Dot(t1->j.va, sA->linearVelocity) + Dot(t1->j.wa, sA->angularVelocity) + Dot(t1->j.vb, sB->linearVelocity) +
                Dot(t1->j.wb, sB->angularVelocity);
    float jv2 = Dot(t2->j.va, sA->linearVelocity) + Dot(t2->j.wa, sA->angularVelocity) + Dot(t2->j.vb, sB->linearVelocity) +
                Dot(t2->j.wb, sB->angularVelocity);

    tangentVelocity->Set(jv1 + t1->bias, jv2 + t2->bias);

    // K = J M^-1 J^T
    float k11 = sA->invMass + Dot(t1->j.wa, s->invIA * t1->j.wa) + sB->invMass + Dot(t1->j.wb, s->invIB * t1->j.wb);
    float k12 = Dot(t1->j.wa, s->invIA * t2->j.wa) + Dot(t1->j.wb, s->invIB * t2->j.wb);
    float k22 = sA->invMass + Dot(t2->j.wa, s->invIA * t2->j.wa) + sB->invMass + Dot(t2->j.wb, s->invIB * t2->j.wb);

    Mat2 mass = Mat2(Vec2(k11, k12), Vec2(k12, k22)).GetInverse();
    Vec2 deltaLambda = -Mul(mass, *tangentVelocity);

    // The unconstrained accumulated impulse that would stop tangent motion is
    // lambda_new = lambda_old + delta_lambda.
    return Vec2(t1->impulse, t2->impulse) + deltaLambda;
}

static void SolveTangentContact(SolverContact* t1, SolverContact* t2, ContactState* s, const SolverContact* n)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec2 tangentVelocity;
    Vec2 impulse = ComputeTangentImpulse(&tangentVelocity, t1, t2, s);

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
        // The required impulse is outside the Coulomb circle, so the contact slides.
        // Kinetic friction uses the maximum magnitude opposite to slip:
        // lambda_t = -mu * lambda_n * v_t / |v_t|.
        float velocity2 = Dot(tangentVelocity, tangentVelocity);
        if (velocity2 > Sqr(epsilon))
        {
            impulse = -tangentVelocity * (maxFriction / std::sqrt(velocity2));
        }
        else
        {
            // v_t / |v_t| is undefined near zero.
            // Preserve the candidate direction and project only its length onto the Coulomb circle.
            impulse *= maxFriction / std::sqrt(impulse2);
        }
    }

    // Only apply the change from the previously accumulated 2D impulse.
    Vec2 deltaLambda = impulse - Vec2(t1->impulse, t2->impulse);
    t1->impulse = impulse.x;
    t2->impulse = impulse.y;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += (t1->j.va * deltaLambda.x + t2->j.va * deltaLambda.y) * sA->invMass;
        sA->angularVelocity += s->invIA * (t1->j.wa * deltaLambda.x + t2->j.wa * deltaLambda.y);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += (t1->j.vb * deltaLambda.x + t2->j.vb * deltaLambda.y) * sB->invMass;
        sB->angularVelocity += s->invIB * (t1->j.wb * deltaLambda.x + t2->j.wb * deltaLambda.y);
    }
}

static void WarmStartSolverContact(SolverContact* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Warm start
    if (!sA->body->IsStatic())
    {
        sA->linearVelocity += n->j.va * (sA->invMass * n->impulse);
        sA->angularVelocity += s->invIA * n->j.wa * n->impulse;
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += n->j.vb * (sB->invMass * n->impulse);
        sB->angularVelocity += s->invIB * n->j.wb * n->impulse;
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

    p->localPlanePoint = qA.RotateInv(s->manifold.referencePoint - comA);
    p->localClipPoint = qB.RotateInv(s->manifold.contactPoints[index].p - comB);
    p->localNormal = qA.RotateInv(s->manifold.contactNormal);
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

    Vec3 planePoint = qA.Rotate(p->localPlanePoint) + comA;
    Vec3 clipPoint = qB.Rotate(p->localClipPoint) + comB;
    Vec3 normal = qA.Rotate(p->localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec3 ra = clipPoint - comA;
    Vec3 rb = clipPoint - comB;

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

    if (s->manifold.featureFlipped)
    {
        s->s1 = bodyB->GetBodyState();
        s->s2 = bodyA->GetBodyState();
    }
    else
    {
        s->s1 = bodyA->GetBodyState();
        s->s2 = bodyB->GetBodyState();
    }

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
        SolveTangentContact(s->tangentContact1 + i, s->tangentContact2 + i, s, s->normalContact + i);
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
