#include "muli3/contact_solver.h"
#include "muli3/body.h"
#include "muli3/contact.h"
#include "muli3/frame.h"
#include "muli3/joints.h" // IWYU pragma: keep
#include "muli3/solver_states.h"

namespace muli3
{

static void PrepareNormal(NormalConstraint* n, ContactState* s, const ContactManifold* m, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute Jacobian J and effective mass W
    // J = [-n, -ra x n, n, rb x n]
    // W = (J * M^-1 * J^t)^-1

    Vec3 normal = m->normal;
    Vec3 ra = m->contactPoints[index].anchorA - sA->motion.c;
    Vec3 rb = m->contactPoints[index].anchorB - sB->motion.c;

    // Setup jacobian
    n->n = normal;
    n->wa = Cross(ra, normal);
    n->wb = Cross(rb, normal);
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

    float k = sA->invMass + Dot(n->wa, s->invIA * n->wa) + sB->invMass + Dot(n->wb, s->invIB * n->wb);
    n->m = k > 0.0f ? 1.0f / k : 0.0f;
}

static void SolveNormal(ContactPoint* p, NormalConstraint* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    // Compute corrective impulse: Pc
    // Pc = J^t * lambda (lambda: lagrangian multiplier)
    // lambda = (J * M^-1 * J^t)^-1 * -(Jv + b)

    // Velocity constraint: C' = jv
    float jv = (Dot(n->n, sB->linearVelocity) + Dot(n->wb, sB->angularVelocity)) -
               (Dot(n->n, sA->linearVelocity) + Dot(n->wa, sA->angularVelocity));

    float lambda = n->m * -(jv + n->bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = p->impulse;
    p->impulse = Max(0.0f, p->impulse + lambda);
    lambda = p->impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda
    if (!sA->body->IsStatic())
    {
        sA->linearVelocity -= n->n * (sA->invMass * lambda);
        sA->angularVelocity -= s->invIA * n->wa * lambda;
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += n->n * (sB->invMass * lambda);
        sB->angularVelocity += s->invIB * n->wb * lambda;
    }
}

static void PrepareFriction(FrictionConstraint* f, ContactState* s, const ContactManifold* m)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 normal = m->normal;

    Vec3 tangent1, tangent2;
    CoordinateSystem(normal, &tangent1, &tangent2);

    Vec3 ra(0);
    Vec3 rb(0);
    for (int32 i = 0; i < m->contactCount; ++i)
    {
        ra += m->contactPoints[i].anchorA - sA->motion.c;
        rb += m->contactPoints[i].anchorB - sB->motion.c;
    }

    float invCount = 1.0f / m->contactCount;
    ra *= invCount;
    rb *= invCount;

    // Linear friction is applied once at the center of the contact patch.
    // J = [-t, -(ra x t), t, rb x t]

    f->t1 = tangent1;
    f->t2 = tangent2;
    f->ra = ra;
    f->rb = rb;

    f->wa1 = Cross(ra, tangent1);
    f->wb1 = Cross(rb, tangent1);
    f->wa2 = Cross(ra, tangent2);
    f->wb2 = Cross(rb, tangent2);

    f->bias = -s->surfaceSpeed;

    // K = J * M^-1 * J^T
    float k11 = sA->invMass + Dot(f->wa1, s->invIA * f->wa1) + sB->invMass + Dot(f->wb1, s->invIB * f->wb1);
    float k12 = Dot(f->wa1, s->invIA * f->wa2) + Dot(f->wb1, s->invIB * f->wb2);
    float k22 = sA->invMass + Dot(f->wa2, s->invIA * f->wa2) + sB->invMass + Dot(f->wb2, s->invIB * f->wb2);
    Mat2 k = Mat2(Vec2(k11, k12), Vec2(k12, k22));

    f->linearMass = k.GetInverse();

    float twistK = Dot(normal, (s->invIA + s->invIB) * normal);
    f->twistMass = twistK > 0.0f ? 1.0f / twistK : 0.0f;
}

static void SolveFriction(FrictionConstraint* f, ContactState* s, ContactManifold* m)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    float totalNormalImpulse = 0.0f;
    float totalTwistLimit = 0.0f;
    for (int32 i = 0; i < m->contactCount; ++i)
    {
        Vec3 ra = m->contactPoints[i].anchorA - sA->motion.c;
        float impulse = m->contactPoints[i].impulse;

        totalNormalImpulse += impulse;
        totalTwistLimit += Dist(ra, f->ra) * impulse;
    }

    // Twist friction limits angular motion around the contact normal.
    float twistSpeed = Dot(m->normal, sB->angularVelocity - sA->angularVelocity);
    float maxTwistFriction = s->friction * totalTwistLimit;
    float twistLambda = -f->twistMass * twistSpeed;
    float oldAngularImpulse = m->angularImpulse;
    m->angularImpulse = Clamp(m->angularImpulse + twistLambda, -maxTwistFriction, maxTwistFriction);
    twistLambda = m->angularImpulse - oldAngularImpulse;

    if (!sA->body->IsStatic())
    {
        sA->angularVelocity -= s->invIA * m->normal * twistLambda;
    }
    if (!sB->body->IsStatic())
    {
        sB->angularVelocity += s->invIB * m->normal * twistLambda;
    }

    // Solve both tangent axes as single 2D constraint at the manifold center.
    float jv1 = Dot(f->t1, sB->linearVelocity) + Dot(f->wb1, sB->angularVelocity) -
                (Dot(f->t1, sA->linearVelocity) + Dot(f->wa1, sA->angularVelocity));
    float jv2 = Dot(f->t2, sB->linearVelocity) + Dot(f->wb2, sB->angularVelocity) -
                (Dot(f->t2, sA->linearVelocity) + Dot(f->wa2, sA->angularVelocity));

    Vec2 tangentVelocity{ jv1 + f->bias.x, jv2 + f->bias.y };
    Vec2 deltaLambda = -Mul(f->linearMass, tangentVelocity);
    Vec2 impulse = m->impulse + deltaLambda;

    // Coulomb friction limits the accumulated 2D tangent impulse lambda_t by |lambda_t| <= mu * lambda_n.
    // A larger normal impulse lets the contact provide more friction.
    // maxFriction = mu * lambda_n
    float maxFriction = s->friction * totalNormalImpulse;

    // impulse is the lambda_t needed to make the tangent velocity zero.
    // Test |lambda_t|^2 > (mu * lambda_n)^2 to avoid a square root.
    // If it is inside the Coulomb circle, static friction can stop the contact and uses it as-is.
    float impulse2 = Dot(impulse, impulse);
    if (impulse2 > Sqr(maxFriction))
    {
        impulse *= maxFriction / std::sqrt(impulse2);
    }

    // Only apply the change from the previously accumulated 2D impulse.
    deltaLambda = impulse - m->impulse;
    m->impulse = impulse;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity -= (f->t1 * deltaLambda.x + f->t2 * deltaLambda.y) * sA->invMass;
        sA->angularVelocity -= s->invIA * (f->wa1 * deltaLambda.x + f->wa2 * deltaLambda.y);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += (f->t1 * deltaLambda.x + f->t2 * deltaLambda.y) * sB->invMass;
        sB->angularVelocity += s->invIB * (f->wb1 * deltaLambda.x + f->wb2 * deltaLambda.y);
    }
}

static void WarmStartNormal(ContactPoint* p, NormalConstraint* n, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity -= n->n * (sA->invMass * p->impulse);
        sA->angularVelocity -= s->invIA * (n->wa * p->impulse);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += n->n * (sB->invMass * p->impulse);
        sB->angularVelocity += s->invIB * (n->wb * p->impulse);
    }
}

static void WarmStartFriction(const ContactManifold* m, const FrictionConstraint* f, ContactState* s)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    if (!sA->body->IsStatic())
    {
        sA->linearVelocity -= (f->t1 * m->impulse.x + f->t2 * m->impulse.y) * sA->invMass;
        sA->angularVelocity -= s->invIA * (f->wa1 * m->impulse.x + f->wa2 * m->impulse.y + m->normal * m->angularImpulse);
    }
    if (!sB->body->IsStatic())
    {
        sB->linearVelocity += (f->t1 * m->impulse.x + f->t2 * m->impulse.y) * sB->invMass;
        sB->angularVelocity += s->invIB * (f->wb1 * m->impulse.x + f->wb2 * m->impulse.y + m->normal * m->angularImpulse);
    }
}

static void PreparePosition(PositionConstraint* p, ContactState* s, const ContactManifold* m, int32 index)
{
    BodyState* sA = s->s1;
    BodyState* sB = s->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    p->localPointA = qA.RotateInv(m->contactPoints[index].anchorA - comA);
    p->localPointB = qB.RotateInv(m->contactPoints[index].anchorB - comB);
    p->localNormal = qA.RotateInv(m->normal);
}

struct PositionCorrection
{
    Vec3 linearImpulseA, linearImpulseB;
    Vec3 angularImpulseA, angularImpulseB;
};

static bool SolvePosition(PositionCorrection* r, const PositionConstraint* p, const ContactState* s)
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

    s->contactConstraints.resize(s->manifolds.size());

    for (int32 m = 0; m < s->manifolds.size(); ++m)
    {
        ContactManifold* manifold = s->manifolds.data() + m;
        ContactConstraint* constraint = s->contactConstraints.data() + m;

        PrepareFriction(&constraint->frictionContact, s, manifold);

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            PrepareNormal(constraint->normalContact + i, s, manifold, i);
            PreparePosition(constraint->positionContact + i, s, manifold, i);
        }
    }
}

void WarmStartContact(ContactState* s)
{
    for (int32 m = 0; m < s->manifolds.size(); ++m)
    {
        ContactManifold* manifold = s->manifolds.data() + m;
        ContactConstraint* constraint = s->contactConstraints.data() + m;

        WarmStartFriction(manifold, &constraint->frictionContact, s);

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            WarmStartNormal(manifold->contactPoints + i, constraint->normalContact + i, s);
        }
    }
}

void SolveContactVelocityConstraints(ContactState* s)
{
    for (int32 m = 0; m < s->manifolds.size(); ++m)
    {
        ContactManifold* manifold = s->manifolds.data() + m;
        ContactConstraint* constraint = s->contactConstraints.data() + m;

        SolveFriction(&constraint->frictionContact, s, manifold);

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            SolveNormal(manifold->contactPoints + i, constraint->normalContact + i, s);
        }
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

    for (int32 m = 0; m < s->manifolds.size(); ++m)
    {
        ContactManifold* manifold = s->manifolds.data() + m;
        ContactConstraint* constraint = s->contactConstraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            solved &= SolvePosition(&correction, constraint->positionContact + i, s);
        }
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
