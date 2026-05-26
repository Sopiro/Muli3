#include "muli3/contact.h"
#include "muli3/callbacks.h"
#include "muli3/world.h"

namespace muli3
{

extern CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];

Contact::Contact(Collider* colliderA, Collider* colliderB)
    : collideFunction{ nullptr }
    , colliderA{ colliderA }
    , colliderB{ colliderB }
    , setIndex{ null_index }
    , localIndex{ null_index }
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

void Contact::Update()
{
    ContactState* s = GetContactState();

    // The parallel-safe pure mathematical part of updating a contact's manifold and solver warm-starting.
    // Writes are strictly isolated to this contact instance, and read accesses to rigidbody transforms are read-only.
    flag |= Contact::flag_enabled;

    ContactManifold oldManifold = s->manifold;
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        s->normalContact[i].impulseSave = s->normalContact[i].impulse;
        s->tangentContact1[i].impulseSave = s->tangentContact1[i].impulse;
        s->tangentContact2[i].impulseSave = s->tangentContact2[i].impulse;
        s->normalContact[i].impulse = 0.0f;
        s->tangentContact1[i].impulse = 0.0f;
        s->tangentContact2[i].impulse = 0.0f;
    }

    bool wasTouching = (flag & Contact::flag_touching) == Contact::flag_touching;
    if (wasTouching)
    {
        flag |= Contact::flag_was_touching;
    }
    else
    {
        flag &= ~Contact::flag_was_touching;
    }

    RigidBody* bodyA = colliderA->GetBody();
    RigidBody* bodyB = colliderB->GetBody();

    bool touching = collideFunction(
        colliderA->GetShape(), bodyA->GetBodyState()->transform, colliderB->GetShape(), bodyB->GetBodyState()->transform,
        &s->manifold
    );

    if (touching)
    {
        flag |= Contact::flag_touching;
    }
    else
    {
        flag &= ~Contact::flag_touching;
    }

    if (touching == false)
    {
        return;
    }

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

    for (int32 n = 0; n < s->manifold.contactCount; ++n)
    {
        for (int32 o = 0; o < oldManifold.contactCount; ++o)
        {
            if (s->manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                s->normalContact[n].impulse = s->normalContact[o].impulseSave;
                s->tangentContact1[n].impulse = s->tangentContact1[o].impulseSave;
                s->tangentContact2[n].impulse = s->tangentContact2[o].impulseSave;
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

} // namespace muli3
