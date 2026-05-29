#include "muli3/world.h"
#include "muli3/callbacks.h"
#include "muli3/capsule_shape.h"
#include "muli3/collider.h"
#include "muli3/island.h"
#include "muli3/parallel_for.h"
#include "muli3/raycast.h"
#include "muli3/shapes.h"

namespace muli3
{

World::World(const WorldSettings& settings)
    : settings{ settings }
    , constraintGraph{ this }
{
}

World::~World()
{
    Reset();
}

void World::Reset()
{
    while (bodyList)
    {
        Destroy(bodyList);
    }

    MuliAssert(bodyList == nullptr);
    MuliAssert(bodyListTail == nullptr);
    MuliAssert(jointList == nullptr);
    MuliAssert(bodyCount == 0);
    MuliAssert(jointCount == 0);
    MuliAssert(constraintGraph.contactList == nullptr);
    MuliAssert(constraintGraph.contactCount == 0);

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();

    for (int32 i = 0; i < solver_set_count; ++i)
    {
        solverSets[i].bodyStates.clear();
        solverSets[i].contactStates.clear();
        solverSets[i].jointStates.clear();
    }
}

RigidBody* World::CreateEmptyBody(const Transform& transform, RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* b = new (mem) RigidBody(transform, type);
    AddBody(b);
    return b;
}

RigidBody* World::CreateSphere(float radius, const Transform& transform, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateSphereCollider(radius, identity, density);
    return b;
}

RigidBody* World::CreateCapsule(float height, float radius, const Transform& transform, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateCapsuleCollider(height, radius, identity, density);
    return b;
}

RigidBody* World::CreateCapsule(
    const Vec3& point1,
    const Vec3& point2,
    float radius,
    const Transform& tf,
    RigidBody::Type type,
    bool resetPosition,
    float density
)
{
    RigidBody* b = CreateEmptyBody(tf, type);

    Vec3 center = (point1 + point2) * 0.5f;
    CapsuleShape capsule = CapsuleShape{ point1 - center, point2 - center, radius };
    b->CreateCollider(&capsule, identity, density);

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreateBox(
    float width, float height, float depth, const Transform& transform, RigidBody::Type type, float radius, float density
)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateBoxCollider(width, height, depth, identity, radius, density);
    return b;
}

RigidBody* World::CreateBox(const Vec3& size, const Transform& transform, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size.x, size.y, size.z, transform, type, radius, density);
}

RigidBody* World::CreateBox(float size, const Transform& transform, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, size, transform, type, radius, density);
}

RigidBody* World::CreateConvex(
    std::span<const Vec3> vertices, const Transform& transform, RigidBody::Type type, float radius, float density
)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateConvexCollider(vertices, identity, radius, density);
    return b;
}

float World::Step(float dt)
{
    profile = {};
    ProfileScope profile_step{ &profile.step };
    MuliProfileZoneNC(world_step, "Step", color::step, true);

    if (dt <= 0.0f)
    {
        MuliProfileZoneEnd(world_step);
        return 0.0f;
    }

    settings.step.dt = dt;
    settings.step.inv_dt = 1 / dt;

    linearAllocator.GrowMemory();

    {
        ProfileScope profile_broad_phase{ &profile.broad_phase };
        MuliProfileZoneNC(broad_phase, "Broad Phase", color::broad_phase, true);
        constraintGraph.UpdateContactGraph();
        MuliProfileZoneEnd(broad_phase);
    }

    {
        ProfileScope profile_narrow_phase{ &profile.narrow_phase };
        MuliProfileZoneNC(narrow_phase, "Narrow Phase", color::narrow_phase, true);
        constraintGraph.EvaluateContacts();
        MuliProfileZoneEnd(narrow_phase);
    }

    {
        ProfileScope profile_solve{ &profile.solve };
        MuliProfileZoneNC(world_solve, "Solve", color::solve, true);
        Solve();
        MuliProfileZoneEnd(world_solve);
    }

    {
        ProfileScope profile_destroy_buffer{ &profile.deferred_destroy };
        MuliProfileZoneNC(destroy_buffer, "Deferred Destroy", color::deferred_destroy, true);

        for (RigidBody* body : destroyBodyBuffer)
        {
            Destroy(body);
        }
        for (Joint* j : destroyJointBuffer)
        {
            Destroy(j);
        }

        destroyBodyBuffer.clear();
        destroyJointBuffer.clear();
        MuliProfileZoneEnd(destroy_buffer);
    }

    MuliProfileZoneEnd(world_step);
    return 1.0f;
}

void World::Destroy(RigidBody* body)
{
    if (body == nullptr)
    {
        return;
    }

    MuliAssert(body->world == this);

    // Destroy attached joints
    JointEdge* je = body->jointList;
    while (je)
    {
        JointEdge* je0 = je;
        je = je->next;
        je0->other->Awake();

        Destroy(je0->joint);
    }

    while (body->colliderList)
    {
        body->DestroyCollider(body->colliderList);
    }

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;
    --bodyCount;

    RemoveBodyState(body);
    FreeBody(body);
}

void World::Destroy(std::span<RigidBody*> bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (!destroyed.contains(b))
        {
            destroyed.insert(b);
            Destroy(b);
        }
    }
}

void World::BufferDestroy(RigidBody* body)
{
    MuliAssert(body != nullptr);
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<RigidBody*> bodies)
{
    for (RigidBody* body : bodies)
    {
        BufferDestroy(body);
    }
}

void World::Destroy(Joint* joint)
{
    RigidBody* bodyA = joint->bodyA;
    RigidBody* bodyB = joint->bodyB;

    // Remove from the world
    if (joint->prev) joint->prev->next = joint->next;
    if (joint->next) joint->next->prev = joint->prev;
    if (joint == jointList) jointList = joint->next;

    // Remove from bodyA
    if (joint->nodeA.prev) joint->nodeA.prev->next = joint->nodeA.next;
    if (joint->nodeA.next) joint->nodeA.next->prev = joint->nodeA.prev;
    if (&joint->nodeA == bodyA->jointList) bodyA->jointList = joint->nodeA.next;

    // Remove from bodyB
    if (joint->bodyA != joint->bodyB)
    {
        if (joint->nodeB.prev) joint->nodeB.prev->next = joint->nodeB.next;
        if (joint->nodeB.next) joint->nodeB.next->prev = joint->nodeB.prev;
        if (&joint->nodeB == bodyB->jointList) bodyB->jointList = joint->nodeB.next;
    }

    RemoveJointState(joint);
    FreeJoint(joint);
    --jointCount;
}

void World::Destroy(std::span<Joint*> joints)
{
    std::unordered_set<Joint*> destroyed;

    for (size_t i = 0; i < joints.size(); ++i)
    {
        Joint* j = joints[i];

        if (!destroyed.contains(j))
        {
            destroyed.insert(j);
            Destroy(j);
        }
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyJointBuffer.push_back(joint);
}

void World::BufferDestroy(std::span<Joint*> joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
    {
        BufferDestroy(joints[i]);
    }
}

void World::Query(const Vec3& point, WorldQueryCallback* callback) const
{
    struct TempCallback
    {
        Vec3 point;
        WorldQueryCallback* callback;

        bool QueryCallback(NodeIndex node, Collider* collider)
        {
            MuliNotUsed(node);

            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback;

    tempCallback.point = point;
    tempCallback.callback = callback;

    constraintGraph.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, WorldQueryCallback* callback) const
{
    BoxShape region{ aabb.GetExtents(), 0.0f };
    Transform transform{ aabb.GetCenter() };

    struct TempCallback
    {
        BoxShape region;
        Transform transform;
        WorldQueryCallback* callback;

        TempCallback(const BoxShape& region, const Transform& transform)
            : region{ region }
            , transform{ transform }
        {
        }

        bool QueryCallback(NodeIndex node, Collider* collider)
        {
            MuliNotUsed(node);

            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &region, transform))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback(region, transform);

    tempCallback.callback = callback;

    constraintGraph.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::RayCastAny(const Vec3& from, const Vec3& to, float radius, RayCastAnyCallback* callback) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius, radius, radius);

    struct TempCallback
    {
        RayCastAnyCallback* callback;

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput rayInput;
            rayInput.from = subInput.from;
            rayInput.to = subInput.to;
            rayInput.maxFraction = subInput.maxFraction;
            rayInput.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(rayInput, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec3 point = (1.0f - fraction) * rayInput.from + fraction * rayInput.to;

                return callback->OnHitAny(collider, point, output.normal, fraction);
            }

            return rayInput.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;

    constraintGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(const Vec3& from, const Vec3& to, float radius, RayCastClosestCallback* callback) const
{
    struct TempCallback : RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestFraction;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float fraction) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void World::ShapeCastAny(const Shape* shape, const Transform& tf, const Vec3& translation, ShapeCastAnyCallback* callback) const
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    struct TempCallback
    {
        ShapeCastAnyCallback* callback;
        const Shape* shape;
        Transform tf;
        Vec3 translation;

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec3::zero,
                &output
            );
            if (hit)
            {
                return callback->OnHitAny(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;
    tempCallback.shape = shape;
    tempCallback.tf = tf;
    tempCallback.translation = translation;

    AABBCastInput input;
    input.from = tf.p;
    input.to = tf.p + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = aabb.GetExtents() * 0.5f;

    constraintGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(
    const Shape* shape, const Transform& tf, const Vec3& translation, ShapeCastClosestCallback* callback
) const
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float t) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT
        );
        return true;
    }

    return false;
}

void World::Query(const Vec3& point, std::function<bool(Collider* collider)> callback) const
{
    struct TempCallback
    {
        Vec3 point;
        decltype(callback)& callbackFcn;

        TempCallback(Vec3 point, decltype(callback)& callback)
            : point{ point }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeIndex node, Collider* collider)
        {
            MuliNotUsed(node);

            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(point, callback);

    constraintGraph.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const
{
    BoxShape region{ aabb.GetExtents(), 0.0f };
    Transform transform{ aabb.GetCenter() };

    struct TempCallback
    {
        BoxShape region;
        Transform transform;
        decltype(callback)& callbackFcn;

        TempCallback(const BoxShape& region, const Transform& transform, decltype(callback)& callback)
            : region{ region }
            , transform{ transform }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeIndex node, Collider* collider)
        {
            MuliNotUsed(node);

            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &region, transform))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(region, transform, callback);

    constraintGraph.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::RayCastAny(
    const Vec3& from,
    const Vec3& to,
    float radius,
    std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius, radius, radius);

    struct TempCallback
    {
        decltype(callback)& callbackFcn;

        TempCallback(decltype(callback)& callback)
            : callbackFcn{ callback }
        {
        }

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput rayInput;
            rayInput.from = subInput.from;
            rayInput.to = subInput.to;
            rayInput.maxFraction = subInput.maxFraction;
            rayInput.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(rayInput, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec3 point = (1.0f - fraction) * rayInput.from + fraction * rayInput.to;

                return callbackFcn(collider, point, output.normal, fraction);
            }

            return rayInput.maxFraction;
        }
    } tempCallback(callback);

    constraintGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(
    const Vec3& from,
    const Vec3& to,
    float radius,
    std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    struct TempCallback : RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestFraction;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float fraction) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void World::ShapeCastAny(
    const Shape* shape,
    const Transform& tf,
    const Vec3& translation,
    std::function<float(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
) const
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    AABBCastInput input;
    input.from = tf.p;
    input.to = tf.p + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = aabb.GetExtents() * 0.5f;

    struct TempCallback
    {
        decltype(callback)& callbackFcn;
        const Shape* shape;
        Transform tf;
        Vec3 translation;

        TempCallback(decltype(callback)& callback, const Shape* shape, Transform tf, Vec3 translation)
            : callbackFcn{ callback }
            , shape{ shape }
            , tf{ tf }
            , translation{ translation }
        {
        }

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec3::zero,
                &output
            );
            if (hit)
            {
                return callbackFcn(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback(callback, shape, tf, translation);

    constraintGraph.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(
    const Shape* shape,
    const Transform& tf,
    const Vec3& translation,
    std::function<void(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
) const
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float t) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback(tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT);
        return true;
    }

    return false;
}

void World::Solve()
{
    if (bodyCount == 0)
    {
        return;
    }

    int32 stackPointer = 0;
    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));

    int32 islandCount = 0;
    Island* islands = (Island*)linearAllocator.Allocate(bodyCount * sizeof(Island));

    int32 contactIndex0 = 0, bodyIndex0 = 0, jointIndex0 = 0;
    int32 contactIndex = 0, bodyIndex = 0, jointIndex = 0;
    BodyState** islandBodies = (BodyState**)linearAllocator.Allocate(bodyCount * sizeof(BodyState*));
    ContactState** islandContacts =
        (ContactState**)linearAllocator.Allocate(constraintGraph.contactCount * sizeof(ContactState*));
    JointState** islandJoints = (JointState**)linearAllocator.Allocate(jointCount * sizeof(JointState*));

    MuliProfileZoneNC(build_islands, "Build Islands", color::build_islands, true);
    ProfileScope profile_build_islands{ &profile.build_islands };

    SolverSet& awakeSet = solverSets[awake_set];
    int32 awakeBodyCount = int32(awakeSet.bodyStates.size());

    for (int32 i = 0; i < awakeBodyCount; ++i)
    {
        RigidBody* b = awakeSet.bodyStates[i].body;
        if (b->flag & RigidBody::flag_island)
        {
            continue;
        }

        if (b->IsSleeping())
        {
            continue;
        }

        if (b->IsStatic())
        {
            continue;
        }

        if (b->IsEnabled() == false)
        {
            continue;
        }

        MuliProfileZoneNC(build_island, "Build Island", color::random(31928), true);
        stack[stackPointer++] = b;
        b->flag |= RigidBody::flag_island;

        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            islandBodies[bodyIndex++] = t->GetBodyState();
            t->islandIndex = islandCount;

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                Contact* c = ce->contact;

                if (c->flag & Contact::flag_island)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_touching) == 0)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_enabled) == 0)
                {
                    continue;
                }

                islandContacts[contactIndex++] = c->GetContactState();
                c->flag |= Contact::flag_island;

                RigidBody* other = ce->other;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->IsStatic())
                {
                    continue;
                }

                if (other->IsSleeping())
                {
                    WakeBody(other);
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                Joint* j = je->joint;

                if (j->flagIsland == true)
                {
                    continue;
                }

                RigidBody* other = je->other;

                if (other->IsEnabled() == false)
                {
                    continue;
                }

                islandJoints[jointIndex++] = j->GetJointState();
                j->flagIsland = true;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->IsStatic())
                {
                    continue;
                }

                if (other->IsSleeping())
                {
                    WakeBody(other);
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }
        }

        int32 islandContactCount = contactIndex - contactIndex0;
        int32 islandBodyCount = bodyIndex - bodyIndex0;
        int32 islandJointCount = jointIndex - jointIndex0;

        Island* island = &islands[islandCount++];

        island->Prepare(
            islandContacts + contactIndex0, islandBodies + bodyIndex0, islandJoints + jointIndex0, islandContactCount,
            islandBodyCount, islandJointCount
        );

        contactIndex0 = contactIndex;
        bodyIndex0 = bodyIndex;
        jointIndex0 = jointIndex;

        MuliProfileZoneEnd(build_island);
    }
    profile_build_islands.Stop();
    MuliProfileZoneEnd(build_islands);

    {
        ProfileScope profile_solve_islands{ &profile.solve_islands };
        MuliProfileZoneNC(solve_islands, "Solve Islands", color::solve_islands, true);
        ParallelFor(0, islandCount, [&](int32 i) {
            Island* island = islands + i;
            island->Solve(this);
        });
        MuliProfileZoneEnd(solve_islands);
    }

    {
        ProfileScope profile_sync_transforms{ &profile.sync_transforms };
        MuliProfileZoneNC(sync_transforms, "Sync Transforms", color::sync_transforms, true);

        for (int32 i = 0; i < bodyIndex; ++i)
        {
            BodyState* s = islandBodies[i];
            RigidBody* body = s->body;
            MuliAssert(body->IsStatic() == false);

            Transform transform0;
            s->motion.GetTransform(0.0f, &transform0);
            body->SynchronizeTransform();

            if (settings.world_bounds.TestPoint(body->transform.p) == false)
            {
                BufferDestroy(body);
            }
            else
            {
                for (Collider* collider = body->colliderList; collider; collider = collider->next)
                {
                    constraintGraph.UpdateCollider(collider, transform0, body->transform);
                }
            }
        }

        MuliProfileZoneEnd(sync_transforms);
    }

    MuliProfileZoneNC(finalize, "Finalize", color::finalize, true);
    ProfileScope profile_finalize{ &profile.finalize };

    // Transfer awake contacts backward because TransferContact() swap-removes from contactStates.
    // Touching island contacts are marked with flag_island. Non-touching contacts are not in islands,
    // but may also need to leave awakeSet when both bodies fell asleep.
    for (int32 i = int32(awakeSet.contactStates.size()) - 1; i >= 0; --i)
    {
        Contact* contact = awakeSet.contactStates[i].contact;

        RigidBody* bodyA = contact->GetBodyA();
        RigidBody* bodyB = contact->GetBodyB();

        SolverSetIndex targetSet;
        if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
        {
            targetSet = disabled_set;
        }
        else
        {
            bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
            bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
            targetSet = awakeA || awakeB ? awake_set : sleeping_set;
        }

        if (contact->flag & Contact::flag_island)
        {
            contact->flag &= ~Contact::flag_island;
            if (targetSet != awake_set)
            {
                TransferContact(contact, targetSet);
            }
        }
        else if (targetSet != awake_set)
        {
            TransferContact(contact, targetSet);
        }
    }

    // Transfer awake joints backward because TransferJoint() swap-removes from jointStates.
    for (int32 i = int32(awakeSet.jointStates.size()) - 1; i >= 0; --i)
    {
        Joint* joint = awakeSet.jointStates[i].joint;
        if (joint->flagIsland == false)
        {
            continue;
        }

        RigidBody* bodyA = joint->GetBodyA();
        RigidBody* bodyB = joint->GetBodyB();

        SolverSetIndex targetSet;
        if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
        {
            targetSet = disabled_set;
        }
        else if (bodyA->IsStatic() && bodyB->IsStatic())
        {
            targetSet = static_set;
        }
        else
        {
            bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
            bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
            targetSet = awakeA || awakeB ? awake_set : sleeping_set;
        }

        joint->flagIsland = false;
        if (targetSet != awake_set)
        {
            TransferJoint(joint, targetSet);
        }
    }

    // Transfer bodies last. BodyState pointers into awakeSet can be invalidated by swap-remove,
    // so iterate the set itself backward and use flag_island to find bodies solved this step.
    for (int32 i = int32(awakeSet.bodyStates.size()) - 1; i >= 0; --i)
    {
        RigidBody* body = awakeSet.bodyStates[i].body;
        if (body->flag & RigidBody::flag_island)
        {
            body->flag &= ~RigidBody::flag_island;
            if (body->IsEnabled() == false)
            {
                TransferBody(body, disabled_set);
            }
            else if (body->IsSleeping())
            {
                TransferBody(body, sleeping_set);
            }
        }
    }

    sleepingBodyCount = int32(solverSets[sleeping_set].bodyStates.size());
    // ValidateSolverSets();

    linearAllocator.Free(islandJoints, jointCount * sizeof(JointState*));
    linearAllocator.Free(islandContacts, constraintGraph.contactCount * sizeof(ContactState*));
    linearAllocator.Free(islandBodies, bodyCount * sizeof(BodyState*));
    linearAllocator.Free(islands, bodyCount * sizeof(Island));
    linearAllocator.Free(stack, bodyCount * sizeof(RigidBody*));
    MuliProfileZoneEnd(finalize);
}

// Joint factory functions

GrabJoint* World::CreateGrabJoint(RigidBody* body, const Vec3& anchor, const Vec3& target, float frequency, float dampingRatio)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, frequency, dampingRatio);

    AddJoint(gj);
    return gj;
}

FixedRotationJoint* World::CreateFixedRotationJoint(RigidBody* body, float frequency, float dampingRatio)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(FixedRotationJoint));
    FixedRotationJoint* frj = new (mem) FixedRotationJoint(body, frequency, dampingRatio);

    AddJoint(frj);
    return frj;
}

ConeSwingJoint* World::CreateConeSwingJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(ConeSwingJoint));
    ConeSwingJoint* csj = new (mem) ConeSwingJoint(bodyA, bodyB, axis, maxAngle, frequency, dampingRatio);

    AddJoint(csj);
    return csj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, const Vec3& axis, float frequency, float dampingRatio
)
{
    return CreateLimitedRevoluteJoint(bodyA, bodyB, anchor, axis, -pi, pi, frequency, dampingRatio);
}

RevoluteJoint* World::CreateLimitedRevoluteJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float frequency,
    float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj = new (mem) RevoluteJoint(bodyA, bodyB, anchor, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(rj);
    return rj;
}

RevoluteAngleJoint* World::CreateRevoluteAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float frequency, float dampingRatio
)
{
    return CreateLimitedRevoluteAngleJoint(bodyA, bodyB, axis, -pi, pi, frequency, dampingRatio);
}

RevoluteAngleJoint* World::CreateLimitedRevoluteAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteAngleJoint));
    RevoluteAngleJoint* raj = new (mem) RevoluteAngleJoint(bodyA, bodyB, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(raj);
    return raj;
}

TwistAngleJoint* World::CreateTwistAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(TwistAngleJoint));
    TwistAngleJoint* taj = new (mem) TwistAngleJoint(bodyA, bodyB, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(taj);
    return taj;
}

BallSocketJoint* World::CreateBallSocketJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(BallSocketJoint));
    BallSocketJoint* bsj = new (mem) BallSocketJoint(bodyA, bodyB, anchor, frequency, dampingRatio);

    AddJoint(bsj);
    return bsj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float length,
    float frequency,
    float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj = new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, length, frequency, dampingRatio);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, frequency, dampingRatio);
}

DistanceJoint* World::CreateLimitedDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float minLength,
    float maxLength,
    float frequency,
    float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj = new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, minLength, maxLength, frequency, dampingRatio);

    AddJoint(dj);
    return dj;
}

WeldJoint* World::CreateWeldJoint(RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, anchor, frequency, dampingRatio);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(LineJoint));
    LineJoint* lj = new (mem) LineJoint(bodyA, bodyB, anchor, dir, frequency, dampingRatio);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), frequency, dampingRatio
    );
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, frequency, dampingRatio);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), frequency, dampingRatio
    );
}

PulleyJoint* World::CreatePulleyJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    const Vec3& groundAnchorA,
    const Vec3& groundAnchorB,
    float ratio,
    float frequency,
    float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj =
        new (mem) PulleyJoint(bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, ratio, frequency, dampingRatio);

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float maxForce, float maxTorque, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(MotorJoint));
    MotorJoint* mj = new (mem) MotorJoint(bodyA, bodyB, anchor, maxForce, maxTorque, frequency, dampingRatio);

    AddJoint(mj);
    return mj;
}

void World::AddBody(RigidBody* body)
{
    body->world = this;
    body->prev = bodyListTail;
    body->next = nullptr;
    body->colliderList = nullptr;
    body->colliderCount = 0;
    body->contactList = nullptr;
    body->jointList = nullptr;
    body->flag &= ~RigidBody::flag_island;
    body->flag |= RigidBody::flag_enabled;

    // Connect to tail
    if (bodyListTail)
    {
        bodyListTail->next = body;
    }
    else
    {
        bodyList = body;
    }
    bodyListTail = body;

    SolverSetIndex setIndex = body->type == RigidBody::static_body ? static_set : awake_set;
    AddBodyState(body, setIndex);
    ++bodyCount;
}

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

void World::AddJoint(Joint* joint)
{
    // Insert into the world
    joint->prev = nullptr;
    joint->next = jointList;
    if (jointList != nullptr)
    {
        jointList->prev = joint;
    }
    jointList = joint;

    // Connect to island graph

    // Connect joint edge to body A
    joint->nodeA.joint = joint;
    joint->nodeA.other = joint->bodyB;

    joint->nodeA.prev = nullptr;
    joint->nodeA.next = joint->bodyA->jointList;
    if (joint->bodyA->jointList != nullptr)
    {
        joint->bodyA->jointList->prev = &joint->nodeA;
    }
    joint->bodyA->jointList = &joint->nodeA;

    // Connect joint edge to body B
    if (joint->bodyA != joint->bodyB)
    {
        joint->nodeB.joint = joint;
        joint->nodeB.other = joint->bodyA;

        joint->nodeB.prev = nullptr;
        joint->nodeB.next = joint->bodyB->jointList;
        if (joint->bodyB->jointList != nullptr)
        {
            joint->bodyB->jointList->prev = &joint->nodeB;
        }
        joint->bodyB->jointList = &joint->nodeB;
    }

    SolverSetIndex setIndex;
    if (joint->bodyA->IsEnabled() == false || joint->bodyB->IsEnabled() == false)
    {
        setIndex = disabled_set;
    }
    else if (joint->bodyA->IsStatic() && joint->bodyB->IsStatic())
    {
        setIndex = static_set;
    }
    else
    {
        bool awakeA = joint->bodyA->IsStatic() == false && joint->bodyA->IsSleeping() == false;
        bool awakeB = joint->bodyB->IsStatic() == false && joint->bodyB->IsSleeping() == false;
        setIndex = awakeA || awakeB ? awake_set : sleeping_set;
    }

    AddJointState(joint, setIndex);
    ++jointCount;
}

void World::FreeJoint(Joint* joint)
{
    switch (joint->GetType())
    {
    case Joint::Type::grab_joint:
        ((GrabJoint*)joint)->~GrabJoint();
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::fixed_rotation_joint:
        ((FixedRotationJoint*)joint)->~FixedRotationJoint();
        blockAllocator.Free(joint, sizeof(FixedRotationJoint));
        break;
    case Joint::Type::cone_swing_joint:
        ((ConeSwingJoint*)joint)->~ConeSwingJoint();
        blockAllocator.Free(joint, sizeof(ConeSwingJoint));
        break;
    case Joint::Type::revolute_joint:
        ((RevoluteJoint*)joint)->~RevoluteJoint();
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::revolute_angle_joint:
        ((RevoluteAngleJoint*)joint)->~RevoluteAngleJoint();
        blockAllocator.Free(joint, sizeof(RevoluteAngleJoint));
        break;
    case Joint::Type::twist_angle_joint:
        ((TwistAngleJoint*)joint)->~TwistAngleJoint();
        blockAllocator.Free(joint, sizeof(TwistAngleJoint));
        break;
    case Joint::Type::ball_socket_joint:
        ((BallSocketJoint*)joint)->~BallSocketJoint();
        blockAllocator.Free(joint, sizeof(BallSocketJoint));
        break;
    case Joint::Type::distance_joint:
        ((DistanceJoint*)joint)->~DistanceJoint();
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::weld_joint:
        ((WeldJoint*)joint)->~WeldJoint();
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::line_joint:
        ((LineJoint*)joint)->~LineJoint();
        blockAllocator.Free(joint, sizeof(LineJoint));
        break;
    case Joint::Type::prismatic_joint:
        ((PrismaticJoint*)joint)->~PrismaticJoint();
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::pulley_joint:
        ((PulleyJoint*)joint)->~PulleyJoint();
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    case Joint::Type::motor_joint:
        ((MotorJoint*)joint)->~MotorJoint();
        blockAllocator.Free(joint, sizeof(MotorJoint));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

Shape* World::CloneShape(const Shape* shape, const Transform& transform)
{
    if (shape == nullptr)
    {
        return nullptr;
    }

    switch (shape->GetType())
    {
    case Shape::sphere:
    {
        void* mem = blockAllocator.Allocate(sizeof(SphereShape));
        return new (mem) SphereShape(*(const SphereShape*)shape, transform);
    }
    case Shape::capsule:
    {
        void* mem = blockAllocator.Allocate(sizeof(CapsuleShape));
        return new (mem) CapsuleShape(*(const CapsuleShape*)shape, transform);
    }
    case Shape::box:
    {
        void* mem = blockAllocator.Allocate(sizeof(BoxShape));
        return new (mem) BoxShape(*(const BoxShape*)shape, transform);
    }
    case Shape::convex:
    {
        void* mem = blockAllocator.Allocate(sizeof(ConvexShape));
        return new (mem) ConvexShape(*(const ConvexShape*)shape, transform);
    }
    default:
        MuliAssert(false);
        break;
    }

    return nullptr;
}

void World::FreeShape(Shape* shape)
{
    switch (shape->GetType())
    {
    case Shape::sphere:
        ((SphereShape*)shape)->~SphereShape();
        blockAllocator.Free(shape, sizeof(SphereShape));
        break;
    case Shape::capsule:
        ((CapsuleShape*)shape)->~CapsuleShape();
        blockAllocator.Free(shape, sizeof(CapsuleShape));
        break;
    case Shape::box:
        ((BoxShape*)shape)->~BoxShape();
        blockAllocator.Free(shape, sizeof(BoxShape));
        break;
    case Shape::convex:
        ((ConvexShape*)shape)->~ConvexShape();
        blockAllocator.Free(shape, sizeof(ConvexShape));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

BodyState* World::AddBodyState(RigidBody* body, SolverSetIndex setIndex)
{
    SolverSet& set = solverSets[setIndex];
    body->setIndex = setIndex;
    body->localIndex = int32(set.bodyStates.size());

    BodyState state{};
    state.body = body;
    state.motion = Motion{ body->transform };
    state.linearVelocity = Vec3::zero;
    state.angularVelocity = Vec3::zero;
    state.invMass = 0.0f;
    state.invInertia = Mat3::zero;
    state.linearDamping = default_linear_damping;
    state.angularDamping = default_angular_damping;
    state.force = Vec3::zero;
    state.torque = Vec3::zero;
    state.resting = 0.0f;

    set.bodyStates.push_back(state);
    return &set.bodyStates.back();
}

void World::RemoveBodyState(RigidBody* body)
{
    SolverSet& set = solverSets[body->setIndex];
    int32 index = body->localIndex;
    int32 last = int32(set.bodyStates.size() - 1);

    if (index != last)
    {
        set.bodyStates[index] = set.bodyStates[last];
        set.bodyStates[index].body->localIndex = index;
    }

    set.bodyStates.pop_back();
    body->setIndex = null_index;
    body->localIndex = null_index;
}

void World::TransferBody(RigidBody* body, SolverSetIndex targetSet)
{
    if (body->setIndex == targetSet)
    {
        return;
    }

    SolverSet& source = solverSets[body->setIndex];
    SolverSet& target = solverSets[targetSet];

    int32 sourceIndex = body->localIndex;
    int32 targetIndex = int32(target.bodyStates.size());
    target.bodyStates.push_back(source.bodyStates[sourceIndex]);
    target.bodyStates.back().body = body;

    int32 last = int32(source.bodyStates.size() - 1);
    if (sourceIndex != last)
    {
        source.bodyStates[sourceIndex] = source.bodyStates[last];
        source.bodyStates[sourceIndex].body->localIndex = sourceIndex;
    }
    source.bodyStates.pop_back();

    body->setIndex = targetSet;
    body->localIndex = targetIndex;
}

ContactState* World::AddContactState(Contact* contact, SolverSetIndex setIndex)
{
    SolverSet& set = solverSets[setIndex];
    contact->setIndex = setIndex;
    contact->localIndex = int32(set.contactStates.size());

    ContactState state{};
    state.contact = contact;
    state.manifold.contactCount = 0;
    state.friction = MixFriction(contact->colliderA->GetFriction(), contact->colliderB->GetFriction());
    state.restitution = MixRestitution(contact->colliderA->GetRestitution(), contact->colliderB->GetRestitution());
    state.restitutionThreshold =
        MixRestitutionTreshold(contact->colliderA->GetRestitutionTreshold(), contact->colliderB->GetRestitutionTreshold());
    state.surfaceSpeed = contact->colliderB->GetSurfaceSpeed() + contact->colliderA->GetSurfaceSpeed();

    set.contactStates.push_back(state);
    return &set.contactStates.back();
}

void World::RemoveContactState(Contact* contact)
{
    SolverSet& set = solverSets[contact->setIndex];
    int32 index = contact->localIndex;
    int32 last = int32(set.contactStates.size() - 1);

    if (index != last)
    {
        set.contactStates[index] = set.contactStates[last];
        set.contactStates[index].contact->localIndex = index;
    }

    set.contactStates.pop_back();
    contact->setIndex = null_index;
    contact->localIndex = null_index;
}

void World::TransferContact(Contact* contact, SolverSetIndex targetSet)
{
    if (contact->setIndex == targetSet)
    {
        return;
    }

    SolverSet& source = solverSets[contact->setIndex];
    SolverSet& target = solverSets[targetSet];

    int32 sourceIndex = contact->localIndex;
    int32 targetIndex = int32(target.contactStates.size());
    target.contactStates.push_back(source.contactStates[sourceIndex]);
    target.contactStates.back().contact = contact;

    int32 last = int32(source.contactStates.size() - 1);
    if (sourceIndex != last)
    {
        source.contactStates[sourceIndex] = source.contactStates[last];
        source.contactStates[sourceIndex].contact->localIndex = sourceIndex;
    }
    source.contactStates.pop_back();

    contact->setIndex = targetSet;
    contact->localIndex = targetIndex;
}

JointState* World::AddJointState(Joint* joint, SolverSetIndex setIndex)
{
    SolverSet& set = solverSets[setIndex];
    joint->setIndex = setIndex;
    joint->localIndex = int32(set.jointStates.size());

    JointState state{};
    state.joint = joint;
    state.invIA = Mat3::zero;
    state.invIB = Mat3::zero;
    state.beta = 0.0f;
    state.gamma = 0.0f;

    set.jointStates.push_back(state);
    return &set.jointStates.back();
}

void World::RemoveJointState(Joint* joint)
{
    SolverSet& set = solverSets[joint->setIndex];
    int32 index = joint->localIndex;
    int32 last = int32(set.jointStates.size() - 1);

    if (index != last)
    {
        set.jointStates[index] = set.jointStates[last];
        set.jointStates[index].joint->localIndex = index;
    }

    set.jointStates.pop_back();
    joint->setIndex = null_index;
    joint->localIndex = null_index;
}

void World::TransferJoint(Joint* joint, SolverSetIndex targetSet)
{
    if (joint->setIndex == targetSet)
    {
        return;
    }

    SolverSet& source = solverSets[joint->setIndex];
    SolverSet& target = solverSets[targetSet];

    int32 sourceIndex = joint->localIndex;
    int32 targetIndex = int32(target.jointStates.size());
    target.jointStates.push_back(source.jointStates[sourceIndex]);
    target.jointStates.back().joint = joint;

    int32 last = int32(source.jointStates.size() - 1);
    if (sourceIndex != last)
    {
        source.jointStates[sourceIndex] = source.jointStates[last];
        source.jointStates[sourceIndex].joint->localIndex = sourceIndex;
    }
    source.jointStates.pop_back();

    joint->setIndex = targetSet;
    joint->localIndex = targetIndex;
}

void World::WakeBody(RigidBody* body)
{
    if (body == nullptr || body->IsStatic() || body->IsEnabled() == false)
    {
        return;
    }

    if (body->IsSleeping() == false && body->setIndex == awake_set)
    {
        return;
    }

    GrowableArray<RigidBody*, 64> stack;
    stack.push_back(body);

    while (stack.size() > 0)
    {
        RigidBody* b = stack.back();
        stack.pop_back();

        if (b->IsStatic() || b->IsEnabled() == false || (b->IsSleeping() == false && b->setIndex == awake_set))
        {
            continue;
        }

        b->flag &= ~RigidBody::flag_sleeping;
        b->GetBodyState()->resting = 0.0f;
        TransferBody(b, awake_set);

        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            RigidBody* other = ce->other;

            if (other->IsEnabled() == false)
            {
                continue;
            }

            TransferContact(contact, awake_set);

            if (other->IsStatic() == false && other->IsSleeping())
            {
                stack.push_back(other);
            }
        }

        for (JointEdge* je = b->jointList; je; je = je->next)
        {
            Joint* joint = je->joint;
            RigidBody* other = je->other;

            if (other->IsEnabled() == false)
            {
                continue;
            }

            TransferJoint(joint, awake_set);

            if (other->IsStatic() == false && other->IsSleeping())
            {
                stack.push_back(other);
            }
        }
    }
}

void World::SleepBody(RigidBody* body)
{
    if (body == nullptr || body->IsStatic() || body->IsEnabled() == false)
    {
        return;
    }

    if (body->IsSleeping() && body->setIndex == sleeping_set)
    {
        return;
    }

    GrowableArray<RigidBody*, 64> stack;
    GrowableArray<RigidBody*, 64> bodies;

    stack.push_back(body);
    body->flag |= RigidBody::flag_island;

    while (stack.size() > 0)
    {
        RigidBody* b = stack.back();
        stack.pop_back();
        bodies.push_back(b);

        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            RigidBody* other = ce->other;

            if ((contact->flag & Contact::flag_touching) == 0 || (contact->flag & Contact::flag_enabled) == 0)
            {
                continue;
            }

            if (other->IsStatic() || other->IsEnabled() == false || (other->flag & RigidBody::flag_island))
            {
                continue;
            }

            other->flag |= RigidBody::flag_island;
            stack.push_back(other);
        }

        for (JointEdge* je = b->jointList; je; je = je->next)
        {
            RigidBody* other = je->other;

            if (other->IsStatic() || other->IsEnabled() == false || (other->flag & RigidBody::flag_island))
            {
                continue;
            }

            other->flag |= RigidBody::flag_island;
            stack.push_back(other);
        }
    }

    for (RigidBody* b : bodies)
    {
        BodyState* state = b->GetBodyState();
        state->resting = max_float;
        state->force = Vec3::zero;
        state->torque = Vec3::zero;
        state->linearVelocity = Vec3::zero;
        state->angularVelocity = Vec3::zero;

        b->flag |= RigidBody::flag_sleeping;
    }

    for (RigidBody* b : bodies)
    {
        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            RigidBody* bodyA = contact->GetBodyA();
            RigidBody* bodyB = contact->GetBodyB();

            SolverSetIndex targetSet;
            if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
            {
                targetSet = disabled_set;
            }
            else
            {
                bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
                bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
                targetSet = awakeA || awakeB ? awake_set : sleeping_set;
            }

            TransferContact(contact, targetSet);
        }

        for (JointEdge* je = b->jointList; je; je = je->next)
        {
            Joint* joint = je->joint;
            RigidBody* bodyA = joint->GetBodyA();
            RigidBody* bodyB = joint->GetBodyB();

            SolverSetIndex targetSet;
            if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
            {
                targetSet = disabled_set;
            }
            else if (bodyA->IsStatic() && bodyB->IsStatic())
            {
                targetSet = static_set;
            }
            else
            {
                bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
                bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
                targetSet = awakeA || awakeB ? awake_set : sleeping_set;
            }

            TransferJoint(joint, targetSet);
        }
    }

    for (RigidBody* b : bodies)
    {
        b->flag &= ~RigidBody::flag_island;
        TransferBody(b, sleeping_set);
    }
}

void World::ValidateSolverSets() const
{
    for (int32 setIndex = 0; setIndex < solver_set_count; ++setIndex)
    {
        const SolverSet& set = solverSets[setIndex];

        for (int32 i = 0; i < int32(set.bodyStates.size()); ++i)
        {
            MuliAssert(set.bodyStates[i].body->setIndex == setIndex);
            MuliAssert(set.bodyStates[i].body->localIndex == i);
        }

        for (int32 i = 0; i < int32(set.contactStates.size()); ++i)
        {
            MuliAssert(set.contactStates[i].contact->setIndex == setIndex);
            MuliAssert(set.contactStates[i].contact->localIndex == i);
        }

        for (int32 i = 0; i < int32(set.jointStates.size()); ++i)
        {
            MuliAssert(set.jointStates[i].joint->setIndex == setIndex);
            MuliAssert(set.jointStates[i].joint->localIndex == i);
        }
    }
}

} // namespace muli3
