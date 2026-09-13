#include "muli3/contact.h"
#include "muli3/callbacks.h"
#include "muli3/frame.h"
#include "muli3/world.h"

namespace muli3
{

extern CollideFunctionSimple* simple_collide_function_map[Shape::shape_count][Shape::shape_count];
extern CollideFunctionComplex* complex_collide_function_map[Shape::shape_count - Shape::height_field];

Contact::Contact(Collider* colliderA, Collider* colliderB)
    : colliderA{ colliderA }
    , colliderB{ colliderB }
    , poolIndex{ null_index }
    , graphIndex{ null_index }
    , bodyIndexA{ null_index }
    , bodyIndexB{ null_index }
    , setIndex{ null_index }
    , colorIndex{ null_index }
    , localIndex{ null_index }
    , flag{ 0 }
{
    MuliAssert(colliderA->GetType() >= colliderB->GetType());
}

bool Contact::IsSimpleContact() const
{
    return (flag & flag_simple) != 0;
}

ContactState* Contact::GetContactState()
{
    if (colorIndex != null_index)
    {
        MuliAssert(IsSimpleContact() == false);
        return &colliderA->body->world->constraintGraph.batches[colorIndex].scalarContacts.states[localIndex];
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
        MuliAssert(IsSimpleContact() == false);
        return &colliderA->body->world->constraintGraph.batches[colorIndex].scalarContacts.states[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &colliderA->body->world->solverSets[setIndex].contactStates[localIndex];
    }
}

static Manifold ReadBlockManifold(const BlockContactState& state, int32 block, int32 lane)
{
    Manifold manifold;
    manifold.id = state.manifoldId[block].lane[lane];
    manifold.contactCount = int32(state.pointCount[block].lane[lane]);
    manifold.normal = {
        state.normal[block].x.lane[lane],
        state.normal[block].y.lane[lane],
        state.normal[block].z.lane[lane],
    };
    manifold.linearImpulse = {
        state.linearImpulse[block].x.lane[lane],
        state.linearImpulse[block].y.lane[lane],
        state.linearImpulse[block].z.lane[lane],
    };
    manifold.angularImpulse = state.angularImpulse[block].lane[lane];

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        ContactPoint& point = manifold.contactPoints[i];
        point.id = state.pointId[i][block].lane[lane];
        point.anchorA = {
            state.anchorA[i][block].x.lane[lane],
            state.anchorA[i][block].y.lane[lane],
            state.anchorA[i][block].z.lane[lane],
        };
        point.anchorB = {
            state.anchorB[i][block].x.lane[lane],
            state.anchorB[i][block].y.lane[lane],
            state.anchorB[i][block].z.lane[lane],
        };
        point.impulse = state.normalImpulse[i][block].lane[lane];
    }

    return manifold;
}

static void WriteBlockManifold(BlockContactState* state, int32 block, int32 lane, const Manifold& manifold)
{
    state->manifoldId[block].lane[lane] = manifold.id;
    state->pointCount[block].lane[lane] = Float(manifold.contactCount);
    state->normal[block].x.lane[lane] = manifold.normal.x;
    state->normal[block].y.lane[lane] = manifold.normal.y;
    state->normal[block].z.lane[lane] = manifold.normal.z;
    state->linearImpulse[block].x.lane[lane] = manifold.linearImpulse.x;
    state->linearImpulse[block].y.lane[lane] = manifold.linearImpulse.y;
    state->linearImpulse[block].z.lane[lane] = manifold.linearImpulse.z;
    state->angularImpulse[block].lane[lane] = manifold.angularImpulse;

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        if (i < manifold.contactCount)
        {
            const ContactPoint& point = manifold.contactPoints[i];
            state->pointId[i][block].lane[lane] = point.id;
            state->anchorA[i][block].x.lane[lane] = point.anchorA.x;
            state->anchorA[i][block].y.lane[lane] = point.anchorA.y;
            state->anchorA[i][block].z.lane[lane] = point.anchorA.z;
            state->anchorB[i][block].x.lane[lane] = point.anchorB.x;
            state->anchorB[i][block].y.lane[lane] = point.anchorB.y;
            state->anchorB[i][block].z.lane[lane] = point.anchorB.z;
            state->normalImpulse[i][block].lane[lane] = point.impulse;
        }
        else
        {
            state->pointId[i][block].lane[lane] = 0;
            state->anchorA[i][block].x.lane[lane] = 0.0f;
            state->anchorA[i][block].y.lane[lane] = 0.0f;
            state->anchorA[i][block].z.lane[lane] = 0.0f;
            state->anchorB[i][block].x.lane[lane] = 0.0f;
            state->anchorB[i][block].y.lane[lane] = 0.0f;
            state->anchorB[i][block].z.lane[lane] = 0.0f;
            state->normalImpulse[i][block].lane[lane] = 0.0f;
        }
    }
}

int32 Contact::GetManifoldCount() const
{
    return IsSimpleContact() ? 1 : GetContactState()->manifolds.size();
}

Manifold Contact::GetContactManifold(int32 index) const
{
    if (IsSimpleContact())
    {
        MuliAssert(index == 0);
        const BlockContactState& state = colliderA->body->world->constraintGraph.batches[colorIndex].blockContacts.state;
        return ReadBlockManifold(state, localIndex / simd_width, localIndex % simd_width);
    }
    else
    {
        const ContactState* state = GetContactState();
        MuliAssert(0 <= index && index < state->manifolds.size());
        return state->manifolds[index];
    }
}

float Contact::GetFriction() const
{
    if (IsSimpleContact())
    {
        const BlockContactState& state = colliderA->body->world->constraintGraph.batches[colorIndex].blockContacts.state;
        return state.friction[localIndex / simd_width].lane[localIndex % simd_width];
    }
    else
    {
        return GetContactState()->friction;
    }
}

float Contact::GetRestitution() const
{
    if (IsSimpleContact())
    {
        const BlockContactState& state = colliderA->body->world->constraintGraph.batches[colorIndex].blockContacts.state;
        return state.restitution[localIndex / simd_width].lane[localIndex % simd_width];
    }
    else
    {
        return GetContactState()->restitution;
    }
}

float Contact::GetRestitutionThreshold() const
{
    if (IsSimpleContact())
    {
        const BlockContactState& state = colliderA->body->world->constraintGraph.batches[colorIndex].blockContacts.state;
        return state.restitutionThreshold[localIndex / simd_width].lane[localIndex % simd_width];
    }
    else
    {
        return GetContactState()->restitutionThreshold;
    }
}

Vec2 Contact::GetSurfaceSpeed() const
{
    if (IsSimpleContact())
    {
        const BlockContactState& state = colliderA->body->world->constraintGraph.batches[colorIndex].blockContacts.state;
        int32 block = localIndex / simd_width;
        int32 lane = localIndex % simd_width;
        return { state.surfaceSpeed[block].x.lane[lane], state.surfaceSpeed[block].y.lane[lane] };
    }
    else
    {
        return GetContactState()->surfaceSpeed;
    }
}

void Contact::ProjectManifold(Manifold* manifold, Manifold* oldManifolds, int32 oldManifoldCount)
{
    Body* bodyA = colliderA->GetBody();
    Body* bodyB = colliderB->GetBody();

    const float normalMatchThreshold = 0.9986f; // ~ cos 3

    int32 oldIndex = null_index;
    float bestSimilarity = normalMatchThreshold;
    for (int32 i = 0; i < oldManifoldCount; ++i)
    {
        if (manifold->id != oldManifolds[i].id)
        {
            continue;
        }

        float similarity = Dot(manifold->normal, oldManifolds[i].normal);
        if (similarity > bestSimilarity)
        {
            oldIndex = i;
            bestSimilarity = similarity;
        }
    }

    if (oldIndex == null_index)
    {
        return;
    }

    Manifold& oldManifold = oldManifolds[oldIndex];
    if (manifold->contactCount == oldManifold.contactCount)
    {
        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            for (int32 j = 0; j < oldManifold.contactCount; ++j)
            {
                if (manifold->contactPoints[i].id == oldManifold.contactPoints[j].id)
                {
                    manifold->contactPoints[i].impulse = oldManifold.contactPoints[j].impulse;
                    oldManifold.contactPoints[j].id = -1;
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
        for (int32 i = 0; i < manifold->contactCount; ++i)
        {
            const ContactPoint& point = manifold->contactPoints[i];
            Vec3 localA = MulT(bodyA->transform, point.anchorA);
            Vec3 localB = MulT(bodyB->transform, point.anchorB);

            int32 bestOld = null_index;
            float bestDistance2 = max_float;

            for (int32 j = 0; j < oldManifold.contactCount; ++j)
            {
                if (oldManifold.contactPoints[j].id < 0)
                {
                    continue;
                }

                const ContactPoint& oldPoint = oldManifold.contactPoints[j];
                float distanceA2 = Dist2(localA, MulT(oldTransformA, oldPoint.anchorA));
                float distanceB2 = Dist2(localB, MulT(oldTransformB, oldPoint.anchorB));
                float distance2 = distanceA2 + distanceB2;

                if (distanceA2 <= contactMatchDistance2 && distanceB2 <= contactMatchDistance2 && distance2 < bestDistance2)
                {
                    bestOld = j;
                    bestDistance2 = distance2;
                }
            }

            if (bestOld != null_index)
            {
                manifold->contactPoints[i].impulse = oldManifold.contactPoints[bestOld].impulse;
                oldManifold.contactPoints[bestOld].id = -1;
            }
        }
    }

    Vec3 oldLinearImpulse = oldManifold.linearImpulse;
    Vec3 oldAngularImpulse = oldManifold.normal * oldManifold.angularImpulse;

    Vec3 tangent1, tangent2;
    CoordinateSystem(manifold->normal, &tangent1, &tangent2);

    manifold->linearImpulse = tangent1 * Dot(oldLinearImpulse, tangent1) + tangent2 * Dot(oldLinearImpulse, tangent2);
    manifold->angularImpulse = Dot(oldAngularImpulse, manifold->normal);
    oldManifold.id = -1;
}

void Contact::Update()
{
    flag |= Contact::flag_enabled;
    bool wasTouching = (flag & Contact::flag_touching) == Contact::flag_touching;
    flag = wasTouching ? flag | Contact::flag_was_touching : flag & ~Contact::flag_was_touching;

    float friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    float restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    float restitutionThreshold =
        MixRestitutionThreshold(colliderA->GetRestitutionThreshold(), colliderB->GetRestitutionThreshold());
    Vec2 surfaceSpeed = colliderB->GetSurfaceSpeed() + colliderA->GetSurfaceSpeed();

    Body* bodyA = colliderA->GetBody();
    Body* bodyB = colliderB->GetBody();

    if (IsSimpleContact())
    {
        BlockContactState& state = bodyA->world->constraintGraph.batches[colorIndex].blockContacts.state;

        int32 block = localIndex / simd_width;
        int32 lane = localIndex % simd_width;

        state.friction[block].lane[lane] = friction;
        state.restitution[block].lane[lane] = restitution;
        state.restitutionThreshold[block].lane[lane] = restitutionThreshold;
        state.surfaceSpeed[block].x.lane[lane] = surfaceSpeed.x;
        state.surfaceSpeed[block].y.lane[lane] = surfaceSpeed.y;

        Manifold oldManifold = ReadBlockManifold(state, block, lane);
        Manifold manifold{};

        bool touching = simple_collide_function_map[colliderA->GetType()][colliderB->GetType()](
            colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &manifold
        );

        flag = touching ? flag | Contact::flag_touching : flag & ~Contact::flag_touching;
        if (touching)
        {
            ProjectManifold(&manifold, &oldManifold, 1);
        }

        WriteBlockManifold(&state, block, lane, manifold);
    }
    else
    {
        ContactState& state = *GetContactState();
        state.friction = friction;
        state.restitution = restitution;
        state.restitutionThreshold = restitutionThreshold;
        state.surfaceSpeed = surfaceSpeed;

        GrowableStack<Manifold, 4> oldManifolds;
        int32 oldManifoldCount = state.manifolds.size();
        if (oldManifoldCount > 0)
        {
            oldManifolds.resize(oldManifoldCount);
            memcpy(oldManifolds.data(), state.manifolds.data(), oldManifoldCount * sizeof(Manifold));
        }
        state.manifolds.clear();

        bool touching;
        if (colliderA->shape->IsSimpleShape())
        {
            Manifold& manifold = state.manifolds.emplace_back();
            touching = simple_collide_function_map[colliderA->GetType()][colliderB->GetType()](
                colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &manifold
            );
        }
        else
        {
            touching = complex_collide_function_map[colliderA->GetType() - Shape::height_field](
                colliderA->GetShape(), bodyA->transform, colliderB->GetShape(), bodyB->transform, &state.manifolds
            );
        }

        flag = touching ? flag | Contact::flag_touching : flag & ~Contact::flag_touching;
        if (touching == false)
        {
            return;
        }

        for (int32 i = 0; i < state.manifolds.size(); ++i)
        {
            ProjectManifold(&state.manifolds[i], oldManifolds.data(), oldManifoldCount);
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
