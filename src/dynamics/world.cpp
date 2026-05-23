#include "muli3/world.h"
#include "muli3/box_shape.h"
#include "muli3/capsule_shape.h"
#include "muli3/collider.h"
#include "muli3/convex_shape.h"
#include "muli3/island.h"
#include "muli3/raycast.h"
#include "muli3/sphere_shape.h"

namespace muli3
{

World::World(const WorldSettings& settings)
    : settings{ settings }
    , contactGraph{ this }
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
    MuliAssert(contactGraph.contactList == nullptr);
    MuliAssert(contactGraph.contactCount == 0);

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();
}

RigidBody* World::CreateEmptyBody(const Transform& transform, RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* b = new (mem) RigidBody(transform, type);

    b->world = this;
    b->prev = bodyListTail;
    b->next = nullptr;
    b->contactList = nullptr;
    b->jointList = nullptr;
    b->colliderList = nullptr;
    b->colliderCount = 0;
    b->flag &= ~RigidBody::flag_island;
    b->flag |= RigidBody::flag_enabled;

    if (bodyListTail)
    {
        bodyListTail->next = b;
    }
    else
    {
        bodyList = b;
    }
    bodyListTail = b;
    ++bodyCount;

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
    MuliProfileZoneNC(world_step, "World::Step", color::step, true);

    settings.step.dt = dt;
    settings.step.inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    if (settings.step.inv_dt == 0.0f)
    {
        MuliProfileZoneEnd(world_step);
        return 0.0f;
    }

    linearAllocator.GrowMemory();

    {
        ProfileScope profile_broad_phase{ &profile.broad_phase };
        MuliProfileZoneNC(broad_phase, "Broad Phase", color::broad_phase, true);
        contactGraph.UpdateContactGraph();
        MuliProfileZoneEnd(broad_phase);
    }

    {
        ProfileScope profile_narrow_phase{ &profile.narrow_phase };
        MuliProfileZoneNC(narrow_phase, "Narrow Phase", color::narrow_phase, true);
        contactGraph.EvaluateContacts();
        MuliProfileZoneEnd(narrow_phase);
    }

    {
        ProfileScope profile_solve{ &profile.solve };
        MuliProfileZoneNC(solve, "Solve", color::solve, true);
        Solve();
        MuliProfileZoneEnd(solve);
    }

    ProfileScope profile_destroy_buffer{ &profile.deferred_destroy };
    MuliProfileZoneNC(destroy_buffer, "Deferred Destroy", color::deferred_destroy, true);
    for (RigidBody* body : destroyBodyBuffer)
    {
        if (body && body->world == this)
        {
            Destroy(body);
        }
    }
    for (Joint* j : destroyJointBuffer)
    {
        Destroy(j);
    }

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();
    MuliProfileZoneEnd(destroy_buffer);

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

    FreeBody(body);
}

void World::Destroy(std::span<RigidBody*> bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (destroyed.find(b) == destroyed.end())
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

    FreeJoint(joint);
    --jointCount;
}

void World::Destroy(std::span<Joint*> joints)
{
    std::unordered_set<Joint*> destroyed;

    for (size_t i = 0; i < joints.size(); ++i)
    {
        Joint* j = joints[i];

        if (destroyed.find(j) == destroyed.end())
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

    contactGraph.broadPhase.tree.Query(point, &tempCallback);
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

    contactGraph.broadPhase.tree.Query(aabb, &tempCallback);
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

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
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

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
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

    contactGraph.broadPhase.tree.Query(point, &tempCallback);
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

    contactGraph.broadPhase.tree.Query(aabb, &tempCallback);
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

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
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

    contactGraph.broadPhase.tree.AABBCast(input, &tempCallback);
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
    MuliProfileZoneNC(solve_world, "World::Solve", color::solve, true);

    int32 bodyCount = GetBodyCount();
    if (bodyCount == 0)
    {
        MuliProfileZoneEnd(solve_world);
        return;
    }

    Island island;

    int32 restingBodies = 0;
    int32 islandCount = 0;
    sleepingBodyCount = 0;

    int32 stackPointer = 0;
    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));

    int32 contactIndex0 = 0, bodyIndex0 = 0, jointIndex0 = 0;
    int32 contactIndex = 0, bodyIndex = 0, jointIndex = 0;
    Contact** islandContacts = (Contact**)linearAllocator.Allocate(contactGraph.contactCount * sizeof(Contact*));
    RigidBody** islandBodies = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    Joint** islandJoints = (Joint**)linearAllocator.Allocate(jointCount * sizeof(Joint*));

    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->flag & RigidBody::flag_island)
        {
            continue;
        }

        if (b->IsSleeping())
        {
            ++sleepingBodyCount;
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

        {
            ProfileScope profile_build_islands{ &profile.build_islands };
            MuliProfileZoneNC(build_islands, "Build Islands", color::build_islands, true);

            stack[stackPointer++] = b;
            b->flag |= RigidBody::flag_island;

            ++islandCount;
            while (stackPointer > 0)
            {
                RigidBody* t = stack[--stackPointer];

                islandBodies[bodyIndex++] = t;
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

                    islandContacts[contactIndex++] = c;
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

                    islandJoints[jointIndex++] = j;
                    j->flagIsland = true;

                    if (other->flag & RigidBody::flag_island)
                    {
                        continue;
                    }

                    if (other->IsStatic())
                    {
                        continue;
                    }

                    MuliAssert(stackPointer < bodyCount);
                    stack[stackPointer++] = other;
                    other->flag |= RigidBody::flag_island;
                }

                if (t->resting > settings.sleeping_time)
                {
                    ++restingBodies;
                }
            }

            MuliProfileZoneEnd(build_islands);
        }

        int32 islandContactCount = contactIndex - contactIndex0;
        int32 islandBodyCount = bodyIndex - bodyIndex0;
        int32 islandJointCount = jointIndex - jointIndex0;
        bool sleeping = settings.sleeping && (restingBodies == islandBodyCount);

        island.Prepare(
            sleeping, islandContacts + contactIndex0, islandBodies + bodyIndex0, islandJoints + jointIndex0, islandContactCount,
            islandBodyCount, islandJointCount
        );

        contactIndex0 = contactIndex;
        bodyIndex0 = bodyIndex;
        jointIndex0 = jointIndex;

        {
            ProfileScope profile_solve_islands{ &profile.solve_islands };
            island.Solve(this);
        }

        restingBodies = 0;
    }

    linearAllocator.Free(islandJoints, jointCount * sizeof(Joint*));
    linearAllocator.Free(islandBodies, bodyCount * sizeof(RigidBody*));
    linearAllocator.Free(islandContacts, contactGraph.contactCount * sizeof(Contact*));
    linearAllocator.Free(stack, bodyCount * sizeof(RigidBody*));

    {
        ProfileScope profile_sync_transforms{ &profile.sync_transforms };
        MuliProfileZoneNC(sync_transforms, "Sync Transforms", color::sync_transforms, true);

        for (RigidBody* body = bodyList; body; body = body->next)
        {
            if ((body->flag & RigidBody::flag_island) == 0)
            {
                continue;
            }

            MuliAssert(body->IsStatic() == false);

            body->flag &= ~RigidBody::flag_island;
            Transform transform0;
            body->motion.GetTransform(0.0f, &transform0);
            body->SynchronizeTransform();

            if (settings.world_bounds.TestPoint(body->transform.p) == false)
            {
                BufferDestroy(body);
            }
            else
            {
                for (Collider* collider = body->colliderList; collider; collider = collider->next)
                {
                    contactGraph.UpdateCollider(collider, transform0, body->transform);
                }
            }
        }

        MuliProfileZoneEnd(sync_transforms);
    }

    {
        ProfileScope profile_clear_flags{ &profile.clear_island_flags };
        MuliProfileZoneNC(clear_flags, "Clear Island Flags", color::clear_island_flags, true);

        for (Contact* contact = contactGraph.contactList; contact; contact = contact->next)
        {
            contact->flag &= ~Contact::flag_island;
        }

        for (Joint* joint = jointList; joint; joint = joint->next)
        {
            joint->flagIsland = false;
        }

        MuliProfileZoneEnd(clear_flags);
    }

    MuliProfileZoneEnd(solve_world);
}

// Joint factory functions

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, const Vec3& anchor, const Vec3& target, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(gj);
    return gj;
}

FixedRotationJoint* World::CreateFixedRotationJoint(
    RigidBody* body, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(FixedRotationJoint));
    FixedRotationJoint* frj = new (mem) FixedRotationJoint(body, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(frj);
    return frj;
}

ConeSwingJoint* World::CreateConeSwingJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& axis,
    float maxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(ConeSwingJoint));
    ConeSwingJoint* csj = new (mem) ConeSwingJoint(bodyA, bodyB, axis, maxAngle, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(csj);
    return csj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    return CreateLimitedRevoluteJoint(bodyA, bodyB, anchor, axis, -pi, pi, jointFrequency, jointDampingRatio, jointMass);
}

RevoluteJoint* World::CreateLimitedRevoluteJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj =
        new (mem) RevoluteJoint(bodyA, bodyB, anchor, axis, minAngle, maxAngle, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(rj);
    return rj;
}

RevoluteAngleJoint* World::CreateRevoluteAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateLimitedRevoluteAngleJoint(bodyA, bodyB, axis, -pi, pi, jointFrequency, jointDampingRatio, jointMass);
}

RevoluteAngleJoint* World::CreateLimitedRevoluteAngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteAngleJoint));
    RevoluteAngleJoint* raj =
        new (mem) RevoluteAngleJoint(bodyA, bodyB, axis, minAngle, maxAngle, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(raj);
    return raj;
}

TwistAngleJoint* World::CreateTwistAngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(TwistAngleJoint));
    TwistAngleJoint* taj =
        new (mem) TwistAngleJoint(bodyA, bodyB, axis, minAngle, maxAngle, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(taj);
    return taj;
}

BallSocketJoint* World::CreateBallSocketJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(BallSocketJoint));
    BallSocketJoint* bsj = new (mem) BallSocketJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(bsj);
    return bsj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float length,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj =
        new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, length, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateDistanceJoint(
        bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, jointFrequency, jointDampingRatio, jointMass
    );
}

DistanceJoint* World::CreateLimitedDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchorA,
    const Vec3& anchorB,
    float minLength,
    float maxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj = new (mem)
        DistanceJoint(bodyA, bodyB, anchorA, anchorB, minLength, maxLength, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

WeldJoint* World::CreateWeldJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(LineJoint));
    LineJoint* lj = new (mem) LineJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    const Vec3& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
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
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj = new (mem) PulleyJoint(
        bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, ratio, jointFrequency, jointDampingRatio, jointMass
    );

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& anchor,
    float maxForce,
    float maxTorque,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(MotorJoint));
    MotorJoint* mj =
        new (mem) MotorJoint(bodyA, bodyB, anchor, maxForce, maxTorque, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(mj);
    return mj;
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

    ++jointCount;
}

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

void World::FreeJoint(Joint* joint)
{
    Joint::Type type = joint->type;
    joint->~Joint();

    switch (type)
    {
    case Joint::Type::grab_joint:
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::fixed_rotation_joint:
        blockAllocator.Free(joint, sizeof(FixedRotationJoint));
        break;
    case Joint::Type::cone_swing_joint:
        blockAllocator.Free(joint, sizeof(ConeSwingJoint));
        break;
    case Joint::Type::revolute_joint:
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::revolute_angle_joint:
        blockAllocator.Free(joint, sizeof(RevoluteAngleJoint));
        break;
    case Joint::Type::twist_angle_joint:
        blockAllocator.Free(joint, sizeof(TwistAngleJoint));
        break;
    case Joint::Type::ball_socket_joint:
        blockAllocator.Free(joint, sizeof(BallSocketJoint));
        break;
    case Joint::Type::distance_joint:
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::weld_joint:
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::line_joint:
        blockAllocator.Free(joint, sizeof(LineJoint));
        break;
    case Joint::Type::prismatic_joint:
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::pulley_joint:
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    case Joint::Type::motor_joint:
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

} // namespace muli3
