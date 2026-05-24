#include "muli3/contact.h"
#include "muli3/callbacks.h"
#include "muli3/frame.h"
#include "muli3/settings.h"
#include "muli3/world.h"

namespace muli3
{

extern CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];

Contact::Contact(Collider* colliderA, Collider* colliderB)
    : collideFunction{ nullptr }
    , colliderA{ colliderA }
    , colliderB{ colliderB }
    , setIndex{ -1 }
    , localIndex{ -1 }
    , flag{ 0 }
{
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    collideFunction = collide_function_map[colliderA->GetType()][colliderB->GetType()];
    MuliAssert(collideFunction != nullptr);
}

ContactState* Contact::GetContactState()
{
    return &colliderA->body->world->solverSets[setIndex].contactStates[localIndex];
}

const ContactState* Contact::GetContactState() const
{
    return &colliderA->body->world->solverSets[setIndex].contactStates[localIndex];
}

void ContactState::Update()
{
    Contact* c = contact;

    // The parallel-safe pure mathematical part of updating a contact's manifold and solver warm-starting.
    // Writes are strictly isolated to this contact instance, and read accesses to rigidbody transforms are read-only.
    c->flag |= Contact::flag_enabled;

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

    bool wasTouching = (c->flag & Contact::flag_touching) == Contact::flag_touching;
    if (wasTouching)
    {
        c->flag |= Contact::flag_was_touching;
    }
    else
    {
        c->flag &= ~Contact::flag_was_touching;
    }

    RigidBody* bodyA = c->colliderA->GetBody();
    RigidBody* bodyB = c->colliderB->GetBody();

    bool touching = c->collideFunction(
        c->colliderA->GetShape(), bodyA->GetBodyState()->transform, c->colliderB->GetShape(), bodyB->GetBodyState()->transform,
        &manifold
    );

    if (touching)
    {
        c->flag |= Contact::flag_touching;
    }
    else
    {
        c->flag &= ~Contact::flag_touching;
    }

    if (touching == false)
    {
        return;
    }

    if (manifold.featureFlipped)
    {
        s1 = bodyB->GetBodyState();
        s2 = bodyA->GetBodyState();
    }
    else
    {
        s1 = bodyA->GetBodyState();
        s2 = bodyB->GetBodyState();
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
}

void Contact::TriggerCallbacks()
{
    // Safely execute all user contact listener callbacks sequentially on the main thread during serial state integration.
    if (colliderA->ContactListener == nullptr && colliderB->ContactListener == nullptr)
    {
        return;
    }

    bool wasTouching = (flag & flag_was_touching) == flag_was_touching;
    bool touching = (flag & flag_touching) == flag_touching;

    if (touching == false)
    {
        if (wasTouching)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactEnd(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactEnd(colliderB, colliderA, this);
        }

        return;
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

void ContactState::Prepare(const Timestep& step)
{
    invIA = s1->body->GetWorldInverseInertiaTensor();
    invIB = s2->body->GetWorldInverseInertiaTensor();

    Vec3 tangent1 = GramSchmidt(x_axis, manifold.contactNormal);
    if (tangent1.Normalize() == 0)
    {
        tangent1 = Normalize(GramSchmidt(z_axis, manifold.contactNormal));
    }
    Vec3 tangent2 = Cross(manifold.contactNormal, tangent1);

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].Prepare(this, i, step);
        tangent1Solvers[i].Prepare(this, tangent1, 0, i, step);
        tangent2Solvers[i].Prepare(this, tangent2, 1, i, step);
        positionSolvers[i].Prepare(this, i);
    }
}

void ContactState::SolveVelocityConstraints(const Timestep& step)
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

bool ContactState::SolvePositionConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA.SetZero();
    cAngularImpulseB.SetZero();

    invIA = s1->body->GetWorldInverseInertiaTensor();
    invIB = s2->body->GetWorldInverseInertiaTensor();

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].Solve(this);
    }

    BodyState* bodySimA = s1;
    BodyState* bodySimB = s2;

    bodySimA->motion.c += bodySimA->invMass * cLinearImpulseA;
    Vec3 angularCorrectionA = invIA * cAngularImpulseA;
    Quat w1{ angularCorrectionA, 0.0f };
    bodySimA->motion.q = bodySimA->motion.q + (w1 * bodySimA->motion.q) * 0.5f;
    bodySimA->motion.q.Normalize();

    bodySimB->motion.c += bodySimB->invMass * cLinearImpulseB;
    Vec3 angularCorrectionB = invIB * cAngularImpulseB;
    Quat w2{ angularCorrectionB, 0.0f };
    bodySimB->motion.q = bodySimB->motion.q + (w2 * bodySimB->motion.q) * 0.5f;
    bodySimB->motion.q.Normalize();

    return solved;
}

} // namespace muli3
