#include "muli3/contact.h"
#include "muli3/callbacks.h"
#include "muli3/frame.h"
#include "muli3/world.h"

namespace muli3
{

extern CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];
extern CollideFunction2* collide_function_map2[Shape::shape_count - Shape::height_field];

Contact::Contact(Collider* colliderA, Collider* colliderB)
    : colliderA{ colliderA }
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
    s->restitutionThreshold = MixRestitutionThreshold(colliderA->GetRestitutionThreshold(), colliderB->GetRestitutionThreshold());
    s->surfaceSpeed = colliderB->GetSurfaceSpeed() + colliderA->GetSurfaceSpeed();

    // The parallel-safe pure mathematical part of updating a contact's manifold and solver warm-starting.
    // Writes are strictly isolated to this contact instance, and read accesses to rigidbody transforms are read-only.
    flag |= Contact::flag_enabled;

    GrowableStack<ContactManifold, 4> oldManifolds;

    int32 oldManifoldCount = s->manifolds.size();
    if (oldManifoldCount > 0)
    {
        oldManifolds.resize(oldManifoldCount);
        memcpy(oldManifolds.data(), s->manifolds.data(), oldManifoldCount * sizeof(ContactManifold));
    }

    s->manifolds.clear();

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

    bool touching = false;

    if (colliderA->GetType() < Shape::height_field)
    {
        ContactManifold& manifold = s->manifolds.emplace_back();

        touching = collide_function_map[colliderA->GetType()][colliderB->GetType()](
            colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &manifold
        );
    }
    else
    {
        touching = collide_function_map2[colliderA->GetType() - Shape::height_field](
            colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &s->manifolds
        );
    }

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

    const float normalMatchThreshold = 0.9986f; // ~ cos 3

    for (int32 i = 0; i < s->manifolds.size(); ++i)
    {
        ContactManifold& manifold = s->manifolds[i];

        int32 oldIndex = null_index;
        float bestSimilarity = normalMatchThreshold;
        for (int32 j = 0; j < oldManifolds.size(); ++j)
        {
            if (manifold.id != oldManifolds[j].id)
            {
                continue;
            }

            float similarity = Dot(manifold.normal, oldManifolds[j].normal);
            if (similarity > bestSimilarity)
            {
                oldIndex = j;
                bestSimilarity = similarity;
            }
        }

        if (oldIndex == null_index)
        {
            continue;
        }

        ContactManifold& oldManifold = oldManifolds[oldIndex];
        if (manifold.contactCount == oldManifold.contactCount)
        {
            for (int32 j = 0; j < manifold.contactCount; ++j)
            {
                for (int32 k = 0; k < oldManifold.contactCount; ++k)
                {
                    if (manifold.contactPoints[j].id == oldManifold.contactPoints[k].id)
                    {
                        manifold.contactPoints[j].impulse = oldManifold.contactPoints[k].impulse;
                        oldManifold.contactPoints[k].id = -1;
                        break;
                    }
                }
            }
        }
        else
        {
            const float contactMatchDistance2 = Sqr(2.0f * linear_slop);

            Transform oldTransformA;
            Transform oldTransformB;
            bodyA->GetBodyState()->motion.GetTransform(0.0f, &oldTransformA);
            bodyB->GetBodyState()->motion.GetTransform(0.0f, &oldTransformB);

            // Match the closest points whose anchors remain near each other on both bodies.
            for (int32 j = 0; j < manifold.contactCount; ++j)
            {
                const ContactPoint& point = manifold.contactPoints[j];
                Vec3 localA = MulT(bodyA->transform, point.anchorA);
                Vec3 localB = MulT(bodyB->transform, point.anchorB);

                int32 bestOld = null_index;
                float bestDistance2 = max_float;

                for (int32 k = 0; k < oldManifold.contactCount; ++k)
                {
                    if (oldManifold.contactPoints[k].id < 0)
                    {
                        continue;
                    }

                    const ContactPoint& oldPoint = oldManifold.contactPoints[k];
                    float distanceA2 = Dist2(localA, MulT(oldTransformA, oldPoint.anchorA));
                    float distanceB2 = Dist2(localB, MulT(oldTransformB, oldPoint.anchorB));
                    float distance2 = distanceA2 + distanceB2;

                    if (distanceA2 <= contactMatchDistance2 && distanceB2 <= contactMatchDistance2 && distance2 < bestDistance2)
                    {
                        bestOld = k;
                        bestDistance2 = distance2;
                    }
                }

                if (bestOld != null_index)
                {
                    manifold.contactPoints[j].impulse = oldManifold.contactPoints[bestOld].impulse;
                    oldManifold.contactPoints[bestOld].id = -1;
                }
            }
        }

        Vec3 oldLinearImpulse = oldManifold.linearImpulse;
        Vec3 oldAngularImpulse = oldManifold.normal * oldManifold.angularImpulse;

        Vec3 tangent1, tangent2;
        CoordinateSystem(manifold.normal, &tangent1, &tangent2);

        manifold.linearImpulse = tangent1 * Dot(oldLinearImpulse, tangent1) + tangent2 * Dot(oldLinearImpulse, tangent2);
        manifold.angularImpulse = Dot(oldAngularImpulse, manifold.normal);

        oldManifold.id = -1;
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
