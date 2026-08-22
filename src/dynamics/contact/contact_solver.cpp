#include "muli3/body.h"
#include "muli3/constraint.h"
#include "muli3/contact.h"
#include "muli3/frame.h"
#include "muli3/joints.h"
#include "muli3/solver_states.h"

namespace muli3
{

void PrepareContact(ContactState* state, ScalarContactConstraint* solver)
{
    // Prepare normal, friction, and position terms for one scalar contact.
    Body* bodyA = state->contact->GetBodyA();
    Body* bodyB = state->contact->GetBodyB();

    solver->bodyA = bodyA->GetBodyState();
    solver->bodyB = bodyB->GetBodyState();

    solver->invIA = bodyA->GetWorldInverseInertiaTensor();
    solver->invIB = bodyB->GetWorldInverseInertiaTensor();
    solver->constraints.resize(state->manifolds.size());

    for (int32 m = 0; m < state->manifolds.size(); ++m)
    {
        ContactManifold* manifold = state->manifolds.data() + m;
        ContactConstraint* constraint = solver->constraints.data() + m;

        Vec3 tangent1, tangent2;
        CoordinateSystem(manifold->normal, &tangent1, &tangent2);

        Vec3 ra = Vec3::zero;
        Vec3 rb = Vec3::zero;
        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            ra += manifold->contactPoints[i].anchorA - solver->bodyA->motion.c;
            rb += manifold->contactPoints[i].anchorB - solver->bodyB->motion.c;
        }

        float invCount = 1.0f / manifold->contactCount;
        ra *= invCount;
        rb *= invCount;

        // Apply linear friction impulse once at the center of the contact patch.
        // J = [-t, -(ra x t), t, rb x t].
        constraint->tangent1 = tangent1;
        constraint->tangent2 = tangent2;
        constraint->frictionWA1 = Cross(ra, tangent1);
        constraint->frictionWB1 = Cross(rb, tangent1);
        constraint->frictionWA2 = Cross(ra, tangent2);
        constraint->frictionWB2 = Cross(rb, tangent2);
        constraint->tangentBias = -state->surfaceSpeed;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            constraint->leverArm[i] = Dist(manifold->contactPoints[i].anchorA - solver->bodyA->motion.c, ra);
        }

        // The 2D friction effective mass is K^-1, where K = J * M^-1 * J^T.
        float k11 = solver->bodyA->invMass + Dot(constraint->frictionWA1, solver->invIA * constraint->frictionWA1) +
                    solver->bodyB->invMass + Dot(constraint->frictionWB1, solver->invIB * constraint->frictionWB1);
        float k12 = Dot(constraint->frictionWA1, solver->invIA * constraint->frictionWA2) +
                    Dot(constraint->frictionWB1, solver->invIB * constraint->frictionWB2);
        float k22 = solver->bodyA->invMass + Dot(constraint->frictionWA2, solver->invIA * constraint->frictionWA2) +
                    solver->bodyB->invMass + Dot(constraint->frictionWB2, solver->invIB * constraint->frictionWB2);
        constraint->linearMass = Mat2(Vec2(k11, k12), Vec2(k12, k22)).GetInverse();

        float twistK = Dot(manifold->normal, (solver->invIA + solver->invIB) * manifold->normal);
        constraint->angularMass = twistK > 0.0f ? 1.0f / twistK : 0.0f;
        constraint->localNormal = solver->bodyA->motion.q.RotateInv(manifold->normal);

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            Vec3 normal = manifold->normal;
            Vec3 contactA = manifold->contactPoints[i].anchorA - solver->bodyA->motion.c;
            Vec3 contactB = manifold->contactPoints[i].anchorB - solver->bodyB->motion.c;

            // Cache the angular Jacobian terms and the scalar effective mass.
            // J = [-n, -(ra x n), n, rb x n]
            // W = (J * M^-1 * J^T)^-1
            constraint->normalWA[i] = Cross(contactA, normal);
            constraint->normalWB[i] = Cross(contactB, normal);
            constraint->normalBias[i] = 0.0f;

            Vec3 relativeVelocity = (solver->bodyB->linearVelocity + Cross(solver->bodyB->angularVelocity, contactB)) -
                                    (solver->bodyA->linearVelocity + Cross(solver->bodyA->angularVelocity, contactA));
            float normalVelocity = Dot(normal, relativeVelocity);
            if (-normalVelocity > state->restitutionThreshold)
            {
                constraint->normalBias[i] = state->restitution * normalVelocity;
            }

            float normalK = solver->bodyA->invMass + Dot(constraint->normalWA[i], solver->invIA * constraint->normalWA[i]) +
                            solver->bodyB->invMass + Dot(constraint->normalWB[i], solver->invIB * constraint->normalWB[i]);
            constraint->normalMass[i] = normalK > 0.0f ? 1.0f / normalK : 0.0f;

            constraint->localPointA[i] =
                solver->bodyA->motion.q.RotateInv(manifold->contactPoints[i].anchorA - solver->bodyA->motion.c);
            constraint->localPointB[i] =
                solver->bodyB->motion.q.RotateInv(manifold->contactPoints[i].anchorB - solver->bodyB->motion.c);
        }
    }
}

void WarmStartContact(ContactState* state, ScalarContactConstraint* solver)
{
    // Apply the impulses accumulated in the previous step before iteration.
    BodyState* bodyA = solver->bodyA;
    BodyState* bodyB = solver->bodyB;

    for (int32 m = 0; m < state->manifolds.size(); ++m)
    {
        ContactManifold* manifold = state->manifolds.data() + m;
        ContactConstraint* constraint = solver->constraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            Vec3 normal = manifold->normal;
            Vec3 wa = constraint->normalWA[i];
            Vec3 wb = constraint->normalWB[i];
            float impulse = manifold->contactPoints[i].impulse;

            if (bodyA->invMass > 0.0f)
            {
                bodyA->linearVelocity -= normal * (bodyA->invMass * impulse);
                bodyA->angularVelocity -= solver->invIA * (wa * impulse);
            }
            if (bodyB->invMass > 0.0f)
            {
                bodyB->linearVelocity += normal * (bodyB->invMass * impulse);
                bodyB->angularVelocity += solver->invIB * (wb * impulse);
            }
        }

        Vec2 impulse{ Dot(manifold->linearImpulse, constraint->tangent1), Dot(manifold->linearImpulse, constraint->tangent2) };

        if (bodyA->invMass > 0.0f)
        {
            bodyA->linearVelocity -= manifold->linearImpulse * bodyA->invMass;
            bodyA->angularVelocity -= solver->invIA * (constraint->frictionWA1 * impulse.x + constraint->frictionWA2 * impulse.y +
                                                       manifold->normal * manifold->angularImpulse);
        }
        if (bodyB->invMass > 0.0f)
        {
            bodyB->linearVelocity += manifold->linearImpulse * bodyB->invMass;
            bodyB->angularVelocity += solver->invIB * (constraint->frictionWB1 * impulse.x + constraint->frictionWB2 * impulse.y +
                                                       manifold->normal * manifold->angularImpulse);
        }
    }
}

void SolveContactVelocityScalar(ContactState* state, ScalarContactConstraint* solver)
{
    // Solve normal impulses, twist friction, and the 2D tangent friction constraint.
    BodyState* bodyA = solver->bodyA;
    BodyState* bodyB = solver->bodyB;

    for (int32 m = 0; m < state->manifolds.size(); ++m)
    {
        ContactManifold* manifold = state->manifolds.data() + m;
        ContactConstraint* constraint = solver->constraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            // Compute corrective impulse: Pc
            // Pc = J^t * lambda (lambda: lagrangian multiplier)
            // lambda = (J * M^-1 * J^t)^-1 * -(Jv + b)
            Vec3 normal = manifold->normal;
            Vec3 wa = constraint->normalWA[i];
            Vec3 wb = constraint->normalWB[i];
            float Jv = (Dot(normal, bodyB->linearVelocity) + Dot(wb, bodyB->angularVelocity)) -
                       (Dot(normal, bodyA->linearVelocity) + Dot(wa, bodyA->angularVelocity));
            float lambda = constraint->normalMass[i] * -(Jv + constraint->normalBias[i]);

            // Clamp the accumulated impulse to be nonnegative.
            ContactPoint* point = manifold->contactPoints + i;
            float oldImpulse = point->impulse;
            point->impulse = Max(0.0f, point->impulse + lambda);
            lambda = point->impulse - oldImpulse;

            // Apply the "change" in the accumulated impulse.
            // V2 = V2' + M^-1 * Pc
            // Pc = J^t * lambda
            if (bodyA->invMass > 0.0f)
            {
                bodyA->linearVelocity -= normal * (bodyA->invMass * lambda);
                bodyA->angularVelocity -= solver->invIA * wa * lambda;
            }
            if (bodyB->invMass > 0.0f)
            {
                bodyB->linearVelocity += normal * (bodyB->invMass * lambda);
                bodyB->angularVelocity += solver->invIB * wb * lambda;
            }
        }

        float totalNormalImpulse = 0.0f;
        float totalTwistLimit = 0.0f;
        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            float impulse = manifold->contactPoints[i].impulse;
            totalNormalImpulse += impulse;
            totalTwistLimit += constraint->leverArm[i] * impulse;
        }

        // Twist friction limits angular motion around the contact normal.
        float twistSpeed = Dot(manifold->normal, bodyB->angularVelocity - bodyA->angularVelocity);
        float maxTwistFriction = state->friction * totalTwistLimit;
        float twistLambda = -constraint->angularMass * twistSpeed;
        float oldAngularImpulse = manifold->angularImpulse;
        manifold->angularImpulse = Clamp(manifold->angularImpulse + twistLambda, -maxTwistFriction, maxTwistFriction);
        twistLambda = manifold->angularImpulse - oldAngularImpulse;

        if (bodyA->invMass > 0.0f)
        {
            bodyA->angularVelocity -= solver->invIA * manifold->normal * twistLambda;
        }
        if (bodyB->invMass > 0.0f)
        {
            bodyB->angularVelocity += solver->invIB * manifold->normal * twistLambda;
        }

        // Solve both tangent axes as a single 2D constraint at the manifold center.
        float Jv1 = Dot(constraint->tangent1, bodyB->linearVelocity) + Dot(constraint->frictionWB1, bodyB->angularVelocity) -
                    (Dot(constraint->tangent1, bodyA->linearVelocity) + Dot(constraint->frictionWA1, bodyA->angularVelocity));
        float Jv2 = Dot(constraint->tangent2, bodyB->linearVelocity) + Dot(constraint->frictionWB2, bodyB->angularVelocity) -
                    (Dot(constraint->tangent2, bodyA->linearVelocity) + Dot(constraint->frictionWA2, bodyA->angularVelocity));

        Vec2 tangentVelocity{ Jv1 + constraint->tangentBias.x, Jv2 + constraint->tangentBias.y };
        Vec2 deltaLambda = -Mul(constraint->linearMass, tangentVelocity);
        Vec2 oldImpulse{ Dot(manifold->linearImpulse, constraint->tangent1), Dot(manifold->linearImpulse, constraint->tangent2) };
        Vec2 impulse = oldImpulse + deltaLambda;

        // Coulomb friction limits the accumulated tangent impulse to mu * lambda_n.
        float maxFriction = state->friction * totalNormalImpulse;
        float impulse2 = Dot(impulse, impulse);
        if (impulse2 > Sqr(maxFriction))
        {
            impulse *= maxFriction / std::sqrt(impulse2);
        }

        // Only apply the change from the previously accumulated impulse.
        deltaLambda = impulse - oldImpulse;
        manifold->linearImpulse = constraint->tangent1 * impulse.x + constraint->tangent2 * impulse.y;

        if (bodyA->invMass > 0.0f)
        {
            bodyA->linearVelocity -=
                (constraint->tangent1 * deltaLambda.x + constraint->tangent2 * deltaLambda.y) * bodyA->invMass;
            bodyA->angularVelocity -=
                solver->invIA * (constraint->frictionWA1 * deltaLambda.x + constraint->frictionWA2 * deltaLambda.y);
        }
        if (bodyB->invMass > 0.0f)
        {
            bodyB->linearVelocity +=
                (constraint->tangent1 * deltaLambda.x + constraint->tangent2 * deltaLambda.y) * bodyB->invMass;
            bodyB->angularVelocity +=
                solver->invIB * (constraint->frictionWB1 * deltaLambda.x + constraint->frictionWB2 * deltaLambda.y);
        }
    }
}

void SolveContactPositionScalar(ContactState* state, ScalarContactConstraint* solver)
{
    // Correct penetration using the current poses during nonlinear iterations.
    BodyState* bodyA = solver->bodyA;
    BodyState* bodyB = solver->bodyB;

    for (int32 m = 0; m < state->manifolds.size(); ++m)
    {
        ContactManifold* manifold = state->manifolds.data() + m;
        ContactConstraint* constraint = solver->constraints.data() + m;

        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            // Recompute the contact arms and normal from the current poses.
            Vec3 ra = bodyA->motion.q.Rotate(constraint->localPointA[i]);
            Vec3 rb = bodyB->motion.q.Rotate(constraint->localPointB[i]);
            Vec3 normal = bodyA->motion.q.Rotate(constraint->localNormal);

            float separation = Dot((bodyB->motion.c - bodyA->motion.c) + rb - ra, normal);
            if (separation >= -linear_slop)
            {
                continue;
            }

            Vec3 ran = Cross(ra, normal);
            Vec3 rbn = Cross(rb, normal);

            // K = mA^-1 + mB^-1
            // + (ra x n)^T Iw_A^-1 (ra x n)
            // + (rb x n)^T Iw_B^-1 (rb x n)
            //
            // The world inverse inertia is Iw^-1 = R * Il^-1 * R^T.
            // Therefore x^T * Iw^-1 * x = (R^T * x)^T * Il^-1 * (R^T * x).
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
            float c = Min(0.0f, position_correction * (separation + linear_slop));
            float lambda = k > 0.0f ? -c / k : 0.0f;
            Vec3 linearCorrection = normal * lambda;

            // Apply the position correction immediately to both bodies.
            if (bodyA->invMass > 0.0f)
            {
                bodyA->motion.c -= bodyA->invMass * linearCorrection;

                Vec3 angularCorrection = bodyA->motion.q.Rotate(-angularA * lambda);
                Quat w{ angularCorrection, 0.0f };
                bodyA->motion.q = Normalize(bodyA->motion.q + (w * bodyA->motion.q) * 0.5f);
            }

            if (bodyB->invMass > 0.0f)
            {
                bodyB->motion.c += bodyB->invMass * linearCorrection;

                Vec3 angularCorrection = bodyB->motion.q.Rotate(angularB * lambda);
                Quat w{ angularCorrection, 0.0f };
                bodyB->motion.q = Normalize(bodyB->motion.q + (w * bodyB->motion.q) * 0.5f);
            }
        }
    }
}

void PrepareJoint(JointState* j, const Timestep& step)
{
    j->joint->Prepare(step);
}

void WarmStartJoint(JointState* j)
{
    j->joint->WarmStart();
}

void SolveJointVelocity(JointState* j, const Timestep& step)
{
    j->joint->SolveVelocityConstraints(step);
}

} // namespace muli3
