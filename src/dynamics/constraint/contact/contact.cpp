#include "muli3/contact.h"
#include "muli3/callbacks.h"
#include "muli3/frame.h"
#include "muli3/settings.h"

namespace muli3
{

void Contact::Update()
{
    flag |= flag_enabled;

    ContactManifold oldManifold = manifold;
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        normalSolvers[i].impulseSave = normalSolvers[i].impulse;
        tangent1Solvers[i].impulseSave = tangent1Solvers[i].impulse;
        tangent2Solvers[i].impulseSave = tangent2Solvers[i].impulse;
        normalSolvers[i].impulse = 0.0f;
        tangent1Solvers[i].impulse = 0.0f;
        tangent2Solvers[i].impulse = 0.0f;
    }

    bool wasTouching = (flag & flag_touching) == flag_touching;
    bool touching = Collide(colliderA->shape, bodyA->transform, colliderB->shape, bodyB->transform, &manifold);

    if (touching)
    {
        flag |= flag_touching;
    }
    else
    {
        flag &= ~flag_touching;
    }

    if (touching == false)
    {
        if (wasTouching)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactEnd(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactEnd(colliderB, colliderA, this);
        }

        return;
    }

    if (manifold.featureFlipped)
    {
        b1 = bodyB;
        b2 = bodyA;
    }
    else
    {
        b1 = bodyA;
        b2 = bodyB;
    }

    for (int32 n = 0; n < manifold.contactCount; ++n)
    {
        for (int32 o = 0; o < oldManifold.contactCount; ++o)
        {
            if (manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                normalSolvers[n].impulse = normalSolvers[o].impulseSave;
                tangent1Solvers[n].impulse = tangent1Solvers[o].impulseSave;
                tangent2Solvers[n].impulse = tangent2Solvers[o].impulseSave;
                break;
            }
        }
    }

    if (wasTouching == false)
    {
        if (colliderA->ContactListener) colliderA->ContactListener->OnContactBegin(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnContactBegin(colliderB, colliderA, this);
    }
    else
    {
        if (colliderA->ContactListener) colliderA->ContactListener->OnContactTouching(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnContactTouching(colliderB, colliderA, this);
    }

    if (colliderA->ContactListener) colliderA->ContactListener->OnPreSolve(colliderA, colliderB, this);
    if (colliderB->ContactListener) colliderB->ContactListener->OnPreSolve(colliderB, colliderA, this);

    if (colliderA->IsEnabled() == false || colliderB->IsEnabled() == false)
    {
        flag &= ~flag_enabled;
    }
}

void Contact::Prepare(const Timestep& step)
{
    friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    restitutionThreshold = MixRestitutionTreshold(colliderA->GetRestitutionTreshold(), colliderB->GetRestitutionTreshold());
    surfaceSpeed = colliderA->GetSurfaceSpeed() + colliderB->GetSurfaceSpeed();

    invIA = b1->GetWorldInverseInertiaTensor();
    invIB = b2->GetWorldInverseInertiaTensor();

    Vec3 tangent1, tangent2;
    CoordinateSystem(manifold.contactNormal, &tangent1, &tangent2);

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].Prepare(this, i, step);
        tangent1Solvers[i].Prepare(this, tangent1, i, step);
        tangent2Solvers[i].Prepare(this, tangent2, i, step);
        positionSolvers[i].Prepare(this, i);
    }
}

void Contact::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        tangent1Solvers[i].Solve(this, normalSolvers + i);
        tangent2Solvers[i].Solve(this, normalSolvers + i);
    }

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].Solve(this);
    }
}

bool Contact::SolvePositionConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA.SetZero();
    cAngularImpulseB.SetZero();

    invIA = b1->GetWorldInverseInertiaTensor();
    invIB = b2->GetWorldInverseInertiaTensor();

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].Solve(this);
    }

    b1->motion.c += b1->invMass * cLinearImpulseA;
    Vec3 angularCorrectionA = b1->GetWorldInverseInertiaTensor() * cAngularImpulseA;
    Quat w1{ angularCorrectionA, 0.0f };
    b1->motion.q = b1->motion.q + (w1 * b1->motion.q) * 0.5f;
    b1->motion.q.Normalize();

    b2->motion.c += b2->invMass * cLinearImpulseB;
    Vec3 angularCorrectionB = b2->GetWorldInverseInertiaTensor() * cAngularImpulseB;
    Quat w2{ angularCorrectionB, 0.0f };
    b2->motion.q = b2->motion.q + (w2 * b2->motion.q) * 0.5f;
    b2->motion.q.Normalize();

    return solved;
}

} // namespace muli3
