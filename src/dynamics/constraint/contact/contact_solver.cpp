#include "muli3/contact_solver.h"
#include "muli3/body.h"
#include "muli3/contact.h"
#include "muli3/frame.h"
#include "muli3/joints.h" // IWYU pragma: keep
#include "muli3/solver_states.h"

namespace muli3
{

static void PrepareNormal(NormalConstraint* constraint, ContactState* contact, const ContactManifold* manifold, int32 index)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    // Compute Jacobian J and effective mass W
    // J = [-n, -ra x n, n, rb x n]
    // W = (J * M^-1 * J^t)^-1

    Vec3 normal = manifold->normal;
    Vec3 ra = manifold->contactPoints[index].anchorA - bodyA->motion.c;
    Vec3 rb = manifold->contactPoints[index].anchorB - bodyB->motion.c;

    // Setup Jacobian
    constraint->n = normal;
    constraint->wa = Cross(ra, normal);
    constraint->wb = Cross(rb, normal);
    constraint->bias = 0.0f;

    // Relative velocity at contact point
    Vec3 relativeVelocity =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) - (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));

    // Normal velocity == velocity constraint: Jv
    float normalVelocity = Dot(normal, relativeVelocity);
    if (-normalVelocity > contact->restitutionThreshold)
    {
        constraint->bias = contact->restitution * normalVelocity;
    }

    float k = bodyA->invMass + Dot(constraint->wa, contact->invIA * constraint->wa) + bodyB->invMass +
              Dot(constraint->wb, contact->invIB * constraint->wb);
    constraint->m = k > 0.0f ? 1.0f / k : 0.0f;
}

static void SolveNormal(ContactPoint* point, NormalConstraint* constraint, ContactState* contact)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    // Compute corrective impulse: Pc
    // Pc = J^t * lambda (lambda: lagrangian multiplier)
    // lambda = (J * M^-1 * J^t)^-1 * -(Jv + b)

    // Velocity Constraint: C' = Jv
    float Jv = (Dot(constraint->n, bodyB->linearVelocity) + Dot(constraint->wb, bodyB->angularVelocity)) -
               (Dot(constraint->n, bodyA->linearVelocity) + Dot(constraint->wa, bodyA->angularVelocity));

    float lambda = constraint->m * -(Jv + constraint->bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = point->impulse;
    point->impulse = Max(0.0f, point->impulse + lambda);
    lambda = point->impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 * Pc
    // Pc = J^t * lambda
    if (bodyA->invMass > 0.0f)
    {
        bodyA->linearVelocity -= constraint->n * (bodyA->invMass * lambda);
        bodyA->angularVelocity -= contact->invIA * constraint->wa * lambda;
    }
    if (bodyB->invMass > 0.0f)
    {
        bodyB->linearVelocity += constraint->n * (bodyB->invMass * lambda);
        bodyB->angularVelocity += contact->invIB * constraint->wb * lambda;
    }
}

static void PrepareFriction(FrictionConstraint* constraint, ContactState* contact, const ContactManifold* manifold)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    Vec3 tangent1, tangent2;
    CoordinateSystem(manifold->normal, &tangent1, &tangent2);

    Vec3 ra = Vec3::zero;
    Vec3 rb = Vec3::zero;
    for (int32 i = 0; i < manifold->contactCount; ++i)
    {
        ra += manifold->contactPoints[i].anchorA - bodyA->motion.c;
        rb += manifold->contactPoints[i].anchorB - bodyB->motion.c;
    }

    float invCount = 1.0f / manifold->contactCount;
    ra *= invCount;
    rb *= invCount;

    // Linear friction is applied once at the center of the contact patch.
    // J = [-t, -(ra x t), t, rb x t]

    constraint->t1 = tangent1;
    constraint->t2 = tangent2;
    constraint->ra = ra;
    constraint->rb = rb;

    for (int32 i = 0; i < manifold->contactCount; ++i)
    {
        constraint->da[i] = Dist(manifold->contactPoints[i].anchorA - bodyA->motion.c, ra);
    }

    constraint->wa1 = Cross(ra, tangent1);
    constraint->wb1 = Cross(rb, tangent1);
    constraint->wa2 = Cross(ra, tangent2);
    constraint->wb2 = Cross(rb, tangent2);

    constraint->bias = -contact->surfaceSpeed;

    // K = J * M^-1 * J^T
    float k11 = bodyA->invMass + Dot(constraint->wa1, contact->invIA * constraint->wa1) + bodyB->invMass +
                Dot(constraint->wb1, contact->invIB * constraint->wb1);
    float k12 = Dot(constraint->wa1, contact->invIA * constraint->wa2) + Dot(constraint->wb1, contact->invIB * constraint->wb2);
    float k22 = bodyA->invMass + Dot(constraint->wa2, contact->invIA * constraint->wa2) + bodyB->invMass +
                Dot(constraint->wb2, contact->invIB * constraint->wb2);
    Mat2 k = Mat2(Vec2(k11, k12), Vec2(k12, k22));

    constraint->linearMass = k.GetInverse();

    float twistK = Dot(manifold->normal, (contact->invIA + contact->invIB) * manifold->normal);
    constraint->angularMass = twistK > 0.0f ? 1.0f / twistK : 0.0f;
}

static void SolveFriction(FrictionConstraint* constraint, ContactState* contact, ContactManifold* manifold)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    float totalNormalImpulse = 0.0f;
    float totalTwistLimit = 0.0f;
    for (int32 i = 0; i < manifold->contactCount; ++i)
    {
        float impulse = manifold->contactPoints[i].impulse;

        totalNormalImpulse += impulse;
        totalTwistLimit += constraint->da[i] * impulse;
    }

    // Twist friction limits angular motion around the contact normal.
    float twistSpeed = Dot(manifold->normal, bodyB->angularVelocity - bodyA->angularVelocity);
    float maxTwistFriction = contact->friction * totalTwistLimit;
    float twistLambda = -constraint->angularMass * twistSpeed;
    float oldAngularImpulse = manifold->angularImpulse;
    manifold->angularImpulse = Clamp(manifold->angularImpulse + twistLambda, -maxTwistFriction, maxTwistFriction);
    twistLambda = manifold->angularImpulse - oldAngularImpulse;

    if (bodyA->invMass > 0.0f)
    {
        bodyA->angularVelocity -= contact->invIA * manifold->normal * twistLambda;
    }
    if (bodyB->invMass > 0.0f)
    {
        bodyB->angularVelocity += contact->invIB * manifold->normal * twistLambda;
    }

    // Solve both tangent axes as single 2D constraint at the manifold center.
    float Jv1 = Dot(constraint->t1, bodyB->linearVelocity) + Dot(constraint->wb1, bodyB->angularVelocity) -
                (Dot(constraint->t1, bodyA->linearVelocity) + Dot(constraint->wa1, bodyA->angularVelocity));
    float Jv2 = Dot(constraint->t2, bodyB->linearVelocity) + Dot(constraint->wb2, bodyB->angularVelocity) -
                (Dot(constraint->t2, bodyA->linearVelocity) + Dot(constraint->wa2, bodyA->angularVelocity));

    Vec2 tangentVelocity{ Jv1 + constraint->bias.x, Jv2 + constraint->bias.y };
    Vec2 deltaLambda = -Mul(constraint->linearMass, tangentVelocity);
    Vec2 oldImpulse{ Dot(manifold->linearImpulse, constraint->t1), Dot(manifold->linearImpulse, constraint->t2) };
    Vec2 impulse = oldImpulse + deltaLambda;

    // Coulomb friction limits the accumulated 2D tangent impulse lambda_t by |lambda_t| <= mu * lambda_n.
    // A larger normal impulse lets the contact provide more friction.
    // maxFriction = mu * lambda_n
    float maxFriction = contact->friction * totalNormalImpulse;

    // impulse is the lambda_t needed to make the tangent velocity zero.
    // Test |lambda_t|^2 > (mu * lambda_n)^2 to avoid a square root.
    // If it is inside the Coulomb circle, static friction can stop the contact and uses it as-is.
    float impulse2 = Dot(impulse, impulse);
    if (impulse2 > Sqr(maxFriction))
    {
        impulse *= maxFriction / std::sqrt(impulse2);
    }

    // Only apply the change from the previously accumulated 2D impulse.
    deltaLambda = impulse - oldImpulse;
    manifold->linearImpulse = constraint->t1 * impulse.x + constraint->t2 * impulse.y;

    if (bodyA->invMass > 0.0f)
    {
        bodyA->linearVelocity -= (constraint->t1 * deltaLambda.x + constraint->t2 * deltaLambda.y) * bodyA->invMass;
        bodyA->angularVelocity -= contact->invIA * (constraint->wa1 * deltaLambda.x + constraint->wa2 * deltaLambda.y);
    }
    if (bodyB->invMass > 0.0f)
    {
        bodyB->linearVelocity += (constraint->t1 * deltaLambda.x + constraint->t2 * deltaLambda.y) * bodyB->invMass;
        bodyB->angularVelocity += contact->invIB * (constraint->wb1 * deltaLambda.x + constraint->wb2 * deltaLambda.y);
    }
}

static void WarmStartNormal(ContactPoint* point, NormalConstraint* constraint, ContactState* contact)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    if (bodyA->invMass > 0.0f)
    {
        bodyA->linearVelocity -= constraint->n * (bodyA->invMass * point->impulse);
        bodyA->angularVelocity -= contact->invIA * (constraint->wa * point->impulse);
    }
    if (bodyB->invMass > 0.0f)
    {
        bodyB->linearVelocity += constraint->n * (bodyB->invMass * point->impulse);
        bodyB->angularVelocity += contact->invIB * (constraint->wb * point->impulse);
    }
}

static void WarmStartFriction(const ContactManifold* manifold, const FrictionConstraint* constraint, ContactState* contact)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    Vec2 impulse{ Dot(manifold->linearImpulse, constraint->t1), Dot(manifold->linearImpulse, constraint->t2) };

    if (bodyA->invMass > 0.0f)
    {
        bodyA->linearVelocity -= manifold->linearImpulse * bodyA->invMass;
        bodyA->angularVelocity -= contact->invIA * (constraint->wa1 * impulse.x + constraint->wa2 * impulse.y +
                                                    manifold->normal * manifold->angularImpulse);
    }
    if (bodyB->invMass > 0.0f)
    {
        bodyB->linearVelocity += manifold->linearImpulse * bodyB->invMass;
        bodyB->angularVelocity += contact->invIB * (constraint->wb1 * impulse.x + constraint->wb2 * impulse.y +
                                                    manifold->normal * manifold->angularImpulse);
    }
}

static void PreparePosition(PositionConstraint* constraint, ContactState* contact, const ContactManifold* manifold, int32 index)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    constraint->localPointA = bodyA->motion.q.RotateInv(manifold->contactPoints[index].anchorA - bodyA->motion.c);
    constraint->localPointB = bodyB->motion.q.RotateInv(manifold->contactPoints[index].anchorB - bodyB->motion.c);
}

static bool SolvePosition(const PositionConstraint* constraint, const Vec3& localNormal, ContactState* contact)
{
    BodyState* bodyA = contact->bodyA;
    BodyState* bodyB = contact->bodyB;

    // Contact arms and normal follow the current poses during each nonlinear iteration.
    Vec3 ra = bodyA->motion.q.Rotate(constraint->localPointA);
    Vec3 rb = bodyB->motion.q.Rotate(constraint->localPointB);
    Vec3 normal = bodyA->motion.q.Rotate(localNormal);
    float separation = Dot((bodyB->motion.c - bodyA->motion.c) + rb - ra, normal);
    if (separation >= -linear_slop)
    {
        return true;
    }

    Vec3 ran = Cross(ra, normal);
    Vec3 rbn = Cross(rb, normal);

    // K = mA^-1 + mB^-1
    // + (ra x n)ᵀ Iw_A^-1 (ra x n)
    // + (rb x n)ᵀ Iw_B^-1 (rb x n)
    //
    // The world inverse inertia is Iw^-1 = R * Il^-1 * R^T.
    //
    // x^T * Iw^-1 * x
    // = x^T * R * Il^-1 * R^T * x
    // = (R^T * x)^T * Il^-1 * (R^T * x),
    //
    // localRan is R^T * (ra x n), and angularA is Il^-1 * localRan.
    Vec3 angularA = Vec3::zero;
    Vec3 angularB = Vec3::zero;
    float k = bodyA->invMass + bodyB->invMass;

    if (bodyA->invMass > 0.0f)
    {
        Vec3 localRan = bodyA->motion.q.RotateInv(ran);
        angularA = bodyA->invInertia * localRan;
        k += Dot(localRan, angularA);
    }
    if (bodyB->invMass > 0.0f)
    {
        Vec3 localRbn = bodyB->motion.q.RotateInv(rbn);
        angularB = bodyB->invInertia * localRbn;
        k += Dot(localRbn, angularB);
    }

    // Only correct penetration deeper than linear_slop.
    // separation < -linear_slop -> c < 0 -> positive lambda.
    float c = Min(0.0f, position_correction * (separation + linear_slop));

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec3 linearCorrection = normal * lambda;

    // Apply immediately
    if (bodyA->invMass > 0.0f)
    {
        bodyA->motion.c -= bodyA->invMass * linearCorrection;

        // Rotate Il^-1 * R^T * angularImpulse back to world space.
        Vec3 angularCorrection = bodyA->motion.q.Rotate(-angularA * lambda);
        Quat w{ angularCorrection, 0.0f };
        bodyA->motion.q = Normalize(bodyA->motion.q + (w * bodyA->motion.q) * 0.5f);
    }

    if (bodyB->invMass > 0.0f)
    {
        bodyB->motion.c += bodyB->invMass * linearCorrection;

        // Rotate Il^-1 * R^T * angularImpulse back to world space.
        Vec3 angularCorrection = bodyB->motion.q.Rotate(angularB * lambda);
        Quat w{ angularCorrection, 0.0f };
        bodyB->motion.q = Normalize(bodyB->motion.q + (w * bodyB->motion.q) * 0.5f);
    }

    return -separation <= position_solver_threshold;
}

void PrepareContact(ContactState* contact)
{
    Body* bodyA = contact->contact->GetBodyA();
    Body* bodyB = contact->contact->GetBodyB();

    contact->bodyA = bodyA->GetBodyState();
    contact->bodyB = bodyB->GetBodyState();

    contact->invIA = bodyA->GetWorldInverseInertiaTensor();
    contact->invIB = bodyB->GetWorldInverseInertiaTensor();

    contact->contactConstraints.resize(contact->manifolds.size());

    for (int32 m = 0; m < contact->manifolds.size(); ++m)
    {
        ContactManifold* manifold = contact->manifolds.data() + m;
        ContactConstraint* constraint = contact->contactConstraints.data() + m;

        PrepareFriction(&constraint->frictionContact, contact, manifold);
        constraint->localNormal = contact->bodyA->motion.q.RotateInv(manifold->normal);

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            PrepareNormal(constraint->normalContact + i, contact, manifold, i);
            PreparePosition(constraint->positionContact + i, contact, manifold, i);
        }
    }
}

void WarmStartContact(ContactState* contact)
{
    for (int32 m = 0; m < contact->manifolds.size(); ++m)
    {
        ContactManifold* manifold = contact->manifolds.data() + m;
        ContactConstraint* constraint = contact->contactConstraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            WarmStartNormal(manifold->contactPoints + i, constraint->normalContact + i, contact);
        }

        WarmStartFriction(manifold, &constraint->frictionContact, contact);
    }
}

void SolveContactVelocityConstraints(ContactState* contact)
{
    for (int32 m = 0; m < contact->manifolds.size(); ++m)
    {
        ContactManifold* manifold = contact->manifolds.data() + m;
        ContactConstraint* constraint = contact->contactConstraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            SolveNormal(manifold->contactPoints + i, constraint->normalContact + i, contact);
        }

        SolveFriction(&constraint->frictionContact, contact, manifold);
    }
}

bool SolveContactPositionConstraints(ContactState* contact)
{
    bool solved = true;

    for (int32 m = 0; m < contact->manifolds.size(); ++m)
    {
        ContactManifold* manifold = contact->manifolds.data() + m;
        ContactConstraint* constraint = contact->contactConstraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            solved &= SolvePosition(constraint->positionContact + i, constraint->localNormal, contact);
        }
    }

    return solved;
}

void PrepareJoint(JointState* j, const Timestep& step)
{
    j->joint->Prepare(step);
}

void WarmStartJoint(JointState* j)
{
    j->joint->WarmStart();
}

void SolveJointVelocityConstraints(JointState* j, const Timestep& step)
{
    j->joint->SolveVelocityConstraints(step);
}

} // namespace muli3
