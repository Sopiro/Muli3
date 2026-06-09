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
    , prev{ nullptr }
    , next{ nullptr }
    , id{ null_index }
    , setIndex{ null_index }
    , colorIndex{ null_index }
    , localIndex{ null_index }
    , flag{ 0 }
{
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    collideFunction = collide_function_map[colliderA->GetType()][colliderB->GetType()];
    MuliAssert(collideFunction != nullptr);
}

ContactState* Contact::GetContactState()
{
    if (colorIndex != null_index)
    {
        return &colliderA->body->world->constraintGraph.batches[colorIndex].contactStates[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &colliderA->body->world->solverSets[setIndex].contactStates[localIndex];
    }
}

const ContactState* Contact::GetContactState() const
{
    if (colorIndex != null_index)
    {
        return &colliderA->body->world->constraintGraph.batches[colorIndex].contactStates[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &colliderA->body->world->solverSets[setIndex].contactStates[localIndex];
    }
}

void Contact::Update()
{
    ContactState* s = GetContactState();

    s->friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    s->restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    s->restitutionThreshold = MixRestitutionTreshold(colliderA->GetRestitutionTreshold(), colliderB->GetRestitutionTreshold());
    s->surfaceSpeed = colliderB->GetSurfaceSpeed() + colliderA->GetSurfaceSpeed();

    // The parallel-safe pure mathematical part of updating a contact's manifold and solver warm-starting.
    // Writes are strictly isolated to this contact instance, and read accesses to rigidbody transforms are read-only.
    flag |= Contact::flag_enabled;

    ContactManifold oldManifold = s->manifold;

    float impulseSaveNormal[max_contact_point_count];
    float impulseSaveTangent1[max_contact_point_count];
    float impulseSaveTangent2[max_contact_point_count];
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        impulseSaveNormal[i] = s->normalContact[i].impulse;
        impulseSaveTangent1[i] = s->tangentContact1[i].impulse;
        impulseSaveTangent2[i] = s->tangentContact2[i].impulse;
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

    Body* bodyA = colliderA->GetBody();
    Body* bodyB = colliderB->GetBody();

    bool touching =
        collideFunction(colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &s->manifold);

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

    for (int32 n = 0; n < s->manifold.contactCount; ++n)
    {
        for (int32 o = 0; o < oldManifold.contactCount; ++o)
        {
            if (s->manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                s->normalContact[n].impulse = impulseSaveNormal[o];
                s->tangentContact1[n].impulse = impulseSaveTangent1[o];
                s->tangentContact2[n].impulse = impulseSaveTangent2[o];
                break;
            }
        }
    }
}

void Contact::TriggerCallbacks()
{
    // Execute all user contact listener callbacks sequentially on the main thread during serial state integration.
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
