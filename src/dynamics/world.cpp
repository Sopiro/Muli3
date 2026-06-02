#include "muli3/world.h"
#include "muli3/callbacks.h"
#include "muli3/capsule_shape.h"
#include "muli3/collider.h"
#include "muli3/island.h"
#include "muli3/parallel_for.h"
#include "muli3/raycast.h"
#include "muli3/shapes.h"

#include "muli3/contact_solver.h"

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
    MuliAssert(jointListTail == nullptr);
    MuliAssert(bodyCount == 0);
    MuliAssert(jointCount == 0);
    MuliAssert(constraintGraph.contactList == nullptr);
    MuliAssert(constraintGraph.contactCount == 0);

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();

    for (int32 i = 0; i < solver_set_count; ++i)
    {
        MuliAssert(solverSets[i].bodyStates.empty());
        MuliAssert(solverSets[i].contactStates.empty());
        MuliAssert(solverSets[i].jointStates.empty());
    }

    for (int32 i = 0; i < constraint_color_count; ++i)
    {
        MuliAssert(constraintGraph.batches[i].contactStates.empty());
        MuliAssert(constraintGraph.batches[i].jointStates.empty());
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
    if (joint == jointListTail) jointListTail = joint->prev;

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

    if (joint->colorIndex != null_index)
    {
        constraintGraph.RemoveJointFromGraph(joint);
    }
    else
    {
        RemoveJointState(joint);
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

// https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf
// Erin Catto's numerical method for stable gyroscopic torque integration
static Vec3 SolveGyroscopic(const Quat& q, const Mat3& inertia, const Vec3& w, float h)
{
    // Convert to body frame
    Vec3 localW = q.RotateInv(w);
    Vec3 localL = inertia * localW;

    // Residual vector
    Vec3 f = h * Cross(localW, localL);
    Mat3 gyro = Skew(localW) * inertia - Skew(localL);

    // Jacobian
    Mat3 j = inertia + Mat3{ gyro.ex * h, gyro.ey * h, gyro.ez * h };

    // Single Newton-Raphson update
    localW -= j.GetInverse() * f;

    // Back to world frame
    return q.Rotate(localW);
}

void World::Solve()
{
    SolverSet& awakeSet = solverSets[awake_set];
    int32 awakeBodyCount = int32(awakeSet.bodyStates.size());
    if (awakeBodyCount == 0)
    {
        return;
    }

    struct StepIsland
    {
        int32 bodyStart;
        int32 contactStart;
        int32 jointStart;
        int32 bodyCount;
        int32 contactCount;
        int32 jointCount;
    };

    int32 stackPointer = 0;
    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));

    islandCount = 0;
    StepIsland* islands = (StepIsland*)linearAllocator.Allocate(bodyCount * sizeof(StepIsland));

    int32 contactIndex0 = 0, bodyIndex0 = 0, jointIndex0 = 0;
    int32 contactIndex = 0, bodyIndex = 0, jointIndex = 0;
    BodyState** islandBodies = (BodyState**)linearAllocator.Allocate(bodyCount * sizeof(BodyState*));
    Contact** islandContacts = (Contact**)linearAllocator.Allocate(constraintGraph.contactCount * sizeof(Contact*));
    Joint** islandJoints = (Joint**)linearAllocator.Allocate(jointCount * sizeof(Joint*));

    MuliProfileZoneNC(build_islands, "Build Islands", color::build_islands, true);
    ProfileScope profile_build_islands{ &profile.build_islands };

    for (size_t i = 0; i < awakeSet.bodyStates.size(); ++i)
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

                RigidBody* other = ce->other;

                islandContacts[contactIndex++] = c;
                c->flag |= Contact::flag_island;

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
        }

        int32 islandContactCount = contactIndex - contactIndex0;
        int32 islandBodyCount = bodyIndex - bodyIndex0;
        int32 islandJointCount = jointIndex - jointIndex0;

        StepIsland* island = &islands[islandCount++];
        island->bodyStart = bodyIndex0;
        island->contactStart = contactIndex0;
        island->jointStart = jointIndex0;
        island->bodyCount = islandBodyCount;
        island->contactCount = islandContactCount;
        island->jointCount = islandJointCount;

        contactIndex0 = contactIndex;
        bodyIndex0 = bodyIndex;
        jointIndex0 = jointIndex;

        MuliProfileZoneEnd(build_island);
    }

    profile_build_islands.Stop();
    MuliProfileZoneEnd(build_islands);

    const Timestep& step = settings.step;

    // Integrate velocities for all awake bodies
    MuliProfileZoneNC(integrate_velocities, "Integrate Velocities", color::integrate_velocities, true);
    {
        ParallelFor(0, bodyIndex, [&](int32 i) {
            MuliProfileZoneNC(integrate_velocity, "Integrate Velocity", color::random(123987259), true);

            BodyState* s = islandBodies[i];
            RigidBody* b = s->body;
            s->motion.c0 = s->motion.c;
            s->motion.q0 = s->motion.q;
            s->motion.alpha0 = 0.0f;

            b->flag &= ~RigidBody::flag_sleeping;

            if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
                Length2(s->linearVelocity) > settings.rest_linear_tolerance || Length2(s->torque) > 0.0f ||
                Length2(s->force) > 0.0f)
            {
                s->resting = 0.0f;
            }

            if (b->GetType() == RigidBody::dynamic_body)
            {
                if (settings.apply_gravity)
                {
                    s->linearVelocity += settings.gravity * step.dt;
                }

                s->linearVelocity += s->force * s->invMass * step.dt;
                s->angularVelocity += b->GetWorldInverseInertiaTensor() * s->torque * step.dt;

                if (b->GetGyroscopicTorqueEnabled())
                {
                    s->angularVelocity = SolveGyroscopic(s->motion.q, b->inertia, s->angularVelocity, step.dt);
                }

                s->linearVelocity *= 1.0f / (1.0f + s->linearDamping * step.dt);
                s->angularVelocity *= 1.0f / (1.0f + s->angularDamping * step.dt);
            }

            MuliProfileZoneEnd(integrate_velocity);
        });
    }
    MuliProfileZoneEnd(integrate_velocities);

    // Prepare all constraints
    MuliProfileZoneNC(prepare_constraints, "Prepare Constraints", color::random(5684652), true);
    {
        MuliProfileZoneNC(prepare_contacts, "Prepare Contacts", color::random(1239087), true);
        ParallelFor(0, contactIndex, [&](int32 i) {
            MuliProfileZoneN(prepare_contact, "Prepare Contact", true);
            PrepareContact(islandContacts[i]->GetContactState());
            MuliProfileZoneEnd(prepare_contact);
        });
        MuliProfileZoneEnd(prepare_contacts);

        MuliProfileZoneNC(prepare_joints, "Prepare Joints", color::random(523546), true);
        ParallelFor(0, jointIndex, [&](int32 i) {
            MuliProfileZoneN(prepare_joint, "Prepare Joints", true);
            PrepareJoint(islandJoints[i]->GetJointState(), step);
            MuliProfileZoneEnd(prepare_joint);
        });
        MuliProfileZoneEnd(prepare_joints);
    }
    MuliProfileZoneEnd(prepare_constraints);

    MuliProfileZoneNC(warm_start_constraints, "Warm Start Constraints", color::random(912835), true);
    {
        ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

        MuliProfileZoneNC(warm_start_contacts, "Warm Start Contacts", color::random(912835), true);
        for (ContactState& state : overflow.contactStates)
        {
            MuliProfileZoneNC(warm_start_contact, "Warm Start Contact", color::random(5951211), true);
            WarmStartContact(&state);
            MuliProfileZoneEnd(warm_start_contact);
        }
        MuliProfileZoneEnd(warm_start_contacts);

        MuliProfileZoneNC(warm_start_joints, "Warm Start Joints", color::random(912835), true);
        for (JointState& state : overflow.jointStates)
        {
            MuliProfileZoneNC(warm_start_joint, "Warm Start Joint", color::random(591321), true);
            WarmStartJoint(&state);
            MuliProfileZoneEnd(warm_start_joint);
        }
        MuliProfileZoneEnd(warm_start_joints);

        for (int32 color = 0; color < constraint_overflow_index; ++color)
        {
            ConstraintBatch& batch = constraintGraph.batches[color];

            MuliProfileZoneNC(warm_start_contacts, "Warm Start Contacts", color::random(912835), true);
            ParallelFor(0, batch.contactStates.size(), [&](int32 i) {
                MuliProfileZoneNC(warm_start_contact, "Warm Start Contact", color::random(5951211), true);
                WarmStartContact(&batch.contactStates[i]);
                MuliProfileZoneEnd(warm_start_contact);
            });
            MuliProfileZoneEnd(warm_start_contacts);

            MuliProfileZoneNC(warm_start_joints, "Warm Start Contacts", color::random(912835), true);
            ParallelFor(0, batch.jointStates.size(), [&](int32 i) {
                MuliProfileZoneNC(warm_start_joint, "Warm Start Joint", color::random(591321), true);
                WarmStartJoint(&batch.jointStates[i]);
                MuliProfileZoneEnd(warm_start_joint);
            });
            MuliProfileZoneEnd(warm_start_joints);
        }
    }
    MuliProfileZoneEnd(warm_start_constraints);

    MuliProfileZoneNC(solve_velocities, "Solve Velocities", color::random(98149294), true);
    {
        for (int32 i = 0; i < step.velocity_iterations; ++i)
        {
            ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

            for (ContactState& state : overflow.contactStates)
            {
                MuliProfileZoneNC(solve_velocity_contact, "Solve Velocity Contact", color::random(9082394), true);
                SolveContactVelocityConstraints(&state);
                MuliProfileZoneEnd(solve_velocity_contact);
            }

            for (JointState& state : overflow.jointStates)
            {
                MuliProfileZoneNC(solve_velocity_joint, "Solve Velocity Joint", color::random(1287364), true);
                SolveJointVelocityConstraints(&state, step);
                MuliProfileZoneEnd(solve_velocity_joint);
            }

            MuliProfileZoneNC(solve_velocity, "Solve Velocity", color::random(465456), true);
            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];

                ParallelFor(0, batch.contactStates.size(), [&](int32 i) {
                    MuliProfileZoneNC(solve_velocity_contact, "Solve Velocity Contact", color::random(9082394), true);
                    SolveContactVelocityConstraints(&batch.contactStates[i]);
                    MuliProfileZoneEnd(solve_velocity_contact);
                });

                ParallelFor(0, batch.jointStates.size(), [&](int32 i) {
                    MuliProfileZoneNC(solve_velocity_joint, "Solve Velocity Joint", color::random(1287364), true);
                    SolveJointVelocityConstraints(&batch.jointStates[i], step);
                    MuliProfileZoneEnd(solve_velocity_joint);
                });
            }

            MuliProfileZoneEnd(solve_velocity);
        }
    }
    MuliProfileZoneEnd(solve_velocities);

    MuliProfileZoneNC(integrate_positions, "Integrate Positions", color::random(198372), true);
    {
        ParallelFor(0, bodyIndex, [&](int32 i) {
            MuliProfileZoneNC(integrate_position, "Integrate Position", color::random(1132321), true);
            BodyState* s = islandBodies[i];

            s->force = Vec3::zero;
            s->torque = Vec3::zero;

            s->motion.c += s->linearVelocity * step.dt;

            Quat w{ s->angularVelocity, 0.0f };
            s->motion.q = s->motion.q + (w * s->motion.q) * step.dt * 0.5f;
            s->motion.q.Normalize();
            MuliProfileZoneEnd(integrate_position);
        });
    }
    MuliProfileZoneEnd(integrate_positions);

    MuliProfileZoneNC(solve_positions, "Solve Positions", color::random(8976432), true);
    {
        for (int32 i = 0; i < step.position_iterations; ++i)
        {
            MuliProfileZoneNC(solve_position, "Solve Position", color::solve_position, true);

            ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

            for (ContactState& state : overflow.contactStates)
            {
                MuliProfileZoneNC(solve_position_contact, "Solve Position Contact", color::random(9082394), true);
                if (SolveContactPositionConstraints(&state) == false)
                {
                    state.s1->resting = 0.0f;
                    state.s2->resting = 0.0f;
                }
                MuliProfileZoneEnd(solve_position_contact);
            }

            for (JointState& state : overflow.jointStates)
            {
                MuliProfileZoneNC(solve_position_joint, "Solve Position Joint", color::random(1287364), true);
                if (SolveJointPositionConstraints(&state, step) == false)
                {
                    Joint* joint = state.joint;
                    joint->GetBodyA()->GetBodyState()->resting = 0.0f;
                    joint->GetBodyB()->GetBodyState()->resting = 0.0f;
                }
                MuliProfileZoneEnd(solve_position_joint);
            }

            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];

                ParallelFor(0, batch.contactStates.size(), [&](int32 j) {
                    MuliProfileZoneNC(solve_position_contact, "Solve Position Contact", color::random(9082394), true);
                    ContactState* state = &batch.contactStates[j];
                    if (SolveContactPositionConstraints(state) == false)
                    {
                        state->s1->resting = 0.0f;
                        state->s2->resting = 0.0f;
                    }
                    MuliProfileZoneEnd(solve_position_contact);
                });

                ParallelFor(0, batch.jointStates.size(), [&](int32 j) {
                    MuliProfileZoneNC(solve_position_joint, "Solve Position Joint", color::random(1287364), true);
                    JointState* state = &batch.jointStates[j];
                    if (SolveJointPositionConstraints(state, step) == false)
                    {
                        Joint* joint = state->joint;
                        joint->GetBodyA()->GetBodyState()->resting = 0.0f;
                        joint->GetBodyB()->GetBodyState()->resting = 0.0f;
                    }
                    MuliProfileZoneEnd(solve_position_joint);
                });
            }

            MuliProfileZoneEnd(solve_position);
        }
    }
    MuliProfileZoneEnd(solve_positions);

    MuliProfileZoneNC(sleep_island, "Sleep Island", color::random(12645), true);
    for (int32 i = 0; i < islandCount; ++i)
    {
        StepIsland* island = islands + i;
        bool awakeIsland = false;
        for (int32 j = 0; j < island->bodyCount; ++j)
        {
            BodyState* s = islandBodies[island->bodyStart + j];
            if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
                Length2(s->linearVelocity) > settings.rest_linear_tolerance)
            {
                awakeIsland = true;
                break;
            }
        }

        bool sleeping = false;
        if (awakeIsland)
        {
            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                islandBodies[island->bodyStart + j]->resting = 0.0f;
            }
        }
        else
        {
            sleeping = settings.sleeping;
            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                BodyState* s = islandBodies[island->bodyStart + j];
                s->resting += step.dt;
                sleeping &= s->resting > settings.sleeping_time;
            }
        }

        if (sleeping == false)
        {
            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                islandBodies[island->bodyStart + j]->body->flag &= ~RigidBody::flag_sleeping;
            }
        }
        else
        {
            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                BodyState* s = islandBodies[island->bodyStart + j];
                RigidBody* body = s->body;

                s->force = Vec3::zero;
                s->torque = Vec3::zero;
                s->linearVelocity = Vec3::zero;
                s->angularVelocity = Vec3::zero;
                s->resting = max_float;
                body->flag |= RigidBody::flag_sleeping;
            }
        }
    }
    MuliProfileZoneEnd(sleep_island);

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

    // Move solved graph contacts out of the graph before their bodies leave awakeSet.
    for (int32 i = contactIndex - 1; i >= 0; --i)
    {
        Contact* contact = islandContacts[i];

        RigidBody* bodyA = contact->GetBodyA();
        RigidBody* bodyB = contact->GetBodyB();

        contact->flag &= ~Contact::flag_island;

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

        if (targetSet != awake_set)
        {
            MuliAssert(contact->colorIndex != null_index);

            ContactState state = constraintGraph.batches[contact->colorIndex].contactStates[contact->localIndex];
            constraintGraph.RemoveContactFromGraph(contact);

            SolverSet& target = solverSets[targetSet];
            contact->setIndex = targetSet;
            contact->localIndex = int32(target.contactStates.size());
            target.contactStates.push_back(state);
            target.contactStates.back().contact = contact;
        }
    }

    // Non-touching awake contacts are not in islands, but may also need to leave awakeSet.
    for (int32 i = int32(awakeSet.contactStates.size()) - 1; i >= 0; --i)
    {
        Contact* contact = awakeSet.contactStates[i].contact;
        if (contact->flag & Contact::flag_island)
        {
            continue;
        }

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

        if (targetSet != awake_set)
        {
            ContactState state = awakeSet.contactStates[i];

            int32 last = int32(awakeSet.contactStates.size() - 1);
            if (i != last)
            {
                awakeSet.contactStates[i] = awakeSet.contactStates[last];
                awakeSet.contactStates[i].contact->localIndex = i;
            }
            awakeSet.contactStates.pop_back();

            SolverSet& target = solverSets[targetSet];
            contact->setIndex = targetSet;
            contact->localIndex = int32(target.contactStates.size());
            target.contactStates.push_back(state);
            target.contactStates.back().contact = contact;
        }
    }

    // Move solved graph joints out of the graph before their bodies leave awakeSet.
    for (int32 i = jointIndex - 1; i >= 0; --i)
    {
        Joint* joint = islandJoints[i];

        RigidBody* bodyA = joint->GetBodyA();
        RigidBody* bodyB = joint->GetBodyB();

        joint->flagIsland = false;

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

        if (targetSet != awake_set)
        {
            MuliAssert(joint->colorIndex != null_index);

            JointState state = constraintGraph.batches[joint->colorIndex].jointStates[joint->localIndex];
            constraintGraph.RemoveJointFromGraph(joint);

            SolverSet& target = solverSets[targetSet];
            joint->setIndex = targetSet;
            joint->localIndex = int32(target.jointStates.size());
            target.jointStates.push_back(state);
            target.jointStates.back().joint = joint;
        }
    }

    // Non-island awake joints can still leave awakeSet when both connected bodies slept.
    for (int32 i = int32(awakeSet.jointStates.size()) - 1; i >= 0; --i)
    {
        Joint* joint = awakeSet.jointStates[i].joint;
        if (joint->flagIsland)
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

        if (targetSet != awake_set)
        {
            JointState state = awakeSet.jointStates[i];

            int32 last = int32(awakeSet.jointStates.size() - 1);
            if (i != last)
            {
                awakeSet.jointStates[i] = awakeSet.jointStates[last];
                awakeSet.jointStates[i].joint->localIndex = i;
            }
            awakeSet.jointStates.pop_back();

            SolverSet& target = solverSets[targetSet];
            joint->setIndex = targetSet;
            joint->localIndex = int32(target.jointStates.size());
            target.jointStates.push_back(state);
            target.jointStates.back().joint = joint;
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
#ifndef NDEBUG
    Validate();
#endif

    linearAllocator.Free(islandJoints, jointCount * sizeof(Joint*));
    linearAllocator.Free(islandContacts, constraintGraph.contactCount * sizeof(Contact*));
    linearAllocator.Free(islandBodies, bodyCount * sizeof(BodyState*));
    linearAllocator.Free(islands, bodyCount * sizeof(StepIsland));
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
    joint->prev = jointListTail;
    joint->next = nullptr;
    if (jointListTail != nullptr)
    {
        jointListTail->next = joint;
    }
    else
    {
        jointList = joint;
    }
    jointListTail = joint;

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

    JointState state;
    state.joint = joint;
    state.invIA = Mat3::zero;
    state.invIB = Mat3::zero;
    state.beta = 0.0f;
    state.gamma = 0.0f;

    joint->setIndex = setIndex;
    if (setIndex == awake_set)
    {
        constraintGraph.AddJointToGraph(joint, state);
    }
    else
    {
        SolverSet& set = solverSets[setIndex];
        joint->localIndex = int32(set.jointStates.size());

        set.jointStates.push_back(state);
    }

    if (setIndex == awake_set)
    {
        if (!joint->bodyA->IsStatic() && joint->bodyA->IsSleeping())
        {
            WakeIsland(joint->bodyA);
        }
        else if (!joint->bodyB->IsStatic() && joint->bodyB->IsSleeping())
        {
            WakeIsland(joint->bodyB);
        }
    }

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
    MuliAssert(contact->colorIndex == null_index);

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
    MuliAssert(contact->colorIndex == null_index);

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
    if (contact->colorIndex != null_index)
    {
        if (targetSet == awake_set)
        {
            return;
        }

        ContactState state = constraintGraph.batches[contact->colorIndex].contactStates[contact->localIndex];
        constraintGraph.RemoveContactFromGraph(contact);

        SolverSet& target = solverSets[targetSet];
        contact->setIndex = targetSet;
        contact->localIndex = int32(target.contactStates.size());
        target.contactStates.push_back(state);
        target.contactStates.back().contact = contact;
        return;
    }

    if (contact->setIndex == targetSet)
    {
        return;
    }

    SolverSet& source = solverSets[contact->setIndex];
    SolverSet& target = solverSets[targetSet];

    int32 sourceIndex = contact->localIndex;
    ContactState state = source.contactStates[sourceIndex];

    int32 last = int32(source.contactStates.size() - 1);
    if (sourceIndex != last)
    {
        source.contactStates[sourceIndex] = source.contactStates[last];
        source.contactStates[sourceIndex].contact->localIndex = sourceIndex;
    }
    source.contactStates.pop_back();

    if (targetSet == awake_set && contact->IsTouching() && contact->IsEnabled())
    {
        contact->setIndex = awake_set;
        constraintGraph.AddContactToGraph(contact, state);
    }
    else
    {
        contact->setIndex = targetSet;
        contact->localIndex = int32(target.contactStates.size());
        target.contactStates.push_back(state);
        target.contactStates.back().contact = contact;
    }
}

JointState* World::AddJointState(Joint* joint, SolverSetIndex setIndex)
{
    MuliAssert(joint->colorIndex == null_index);

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
    MuliAssert(joint->colorIndex == null_index);

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
    if (joint->colorIndex != null_index)
    {
        if (targetSet == awake_set)
        {
            return;
        }

        JointState state = constraintGraph.batches[joint->colorIndex].jointStates[joint->localIndex];
        constraintGraph.RemoveJointFromGraph(joint);

        SolverSet& target = solverSets[targetSet];
        joint->setIndex = targetSet;
        joint->localIndex = int32(target.jointStates.size());
        target.jointStates.push_back(state);
        target.jointStates.back().joint = joint;
        return;
    }

    if (joint->setIndex == targetSet)
    {
        return;
    }

    SolverSet& source = solverSets[joint->setIndex];
    SolverSet& target = solverSets[targetSet];

    int32 sourceIndex = joint->localIndex;
    JointState state = source.jointStates[sourceIndex];

    int32 last = int32(source.jointStates.size() - 1);
    if (sourceIndex != last)
    {
        source.jointStates[sourceIndex] = source.jointStates[last];
        source.jointStates[sourceIndex].joint->localIndex = sourceIndex;
    }
    source.jointStates.pop_back();

    if (targetSet == awake_set)
    {
        joint->setIndex = awake_set;
        constraintGraph.AddJointToGraph(joint, state);
    }
    else
    {
        joint->setIndex = targetSet;
        joint->localIndex = int32(target.jointStates.size());
        target.jointStates.push_back(state);
        target.jointStates.back().joint = joint;
    }
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

    body->flag &= ~RigidBody::flag_sleeping;
    body->GetBodyState()->resting = 0.0f;

    TransferBody(body, awake_set);

    for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
    {
        Contact* contact = ce->contact;
        RigidBody* other = ce->other;

        if (other->IsEnabled() == false)
        {
            continue;
        }

        TransferContact(contact, awake_set);
    }

    for (JointEdge* je = body->jointList; je; je = je->next)
    {
        Joint* joint = je->joint;
        RigidBody* other = je->other;

        if (other->IsEnabled() == false)
        {
            continue;
        }

        TransferJoint(joint, awake_set);
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

    BodyState* state = body->GetBodyState();
    state->resting = max_float;
    state->force = Vec3::zero;
    state->torque = Vec3::zero;
    state->linearVelocity = Vec3::zero;
    state->angularVelocity = Vec3::zero;
}

void World::WakeIsland(RigidBody* body)
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

            if (!contact->IsTouching())
            {
                continue;
            }

            if (!other->IsStatic() && other->IsSleeping())
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

            if (!other->IsStatic() && other->IsSleeping())
            {
                stack.push_back(other);
            }
        }
    }
}

void World::SleepIsland(RigidBody* body)
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

void World::Validate() const
{
    std::unordered_set<Contact*> seenContacts;
    std::unordered_set<Joint*> seenJoints;

    for (int32 setIndex = 0; setIndex < solver_set_count; ++setIndex)
    {
        const SolverSet& set = solverSets[setIndex];

        for (int32 i = 0; i < int32(set.bodyStates.size()); ++i)
        {
            RigidBody* body = set.bodyStates[i].body;
            MuliAssert(body->setIndex == setIndex);
            MuliAssert(body->localIndex == i);
            MuliNotUsed(body);

            if (setIndex == static_set)
            {
                MuliAssert(body->IsStatic());
            }
            else if (setIndex == disabled_set)
            {
                MuliAssert(body->IsEnabled() == false);
            }
            else if (setIndex == awake_set)
            {
                MuliAssert(body->IsEnabled());
                MuliAssert(body->IsStatic() == false);
                MuliAssert(body->IsSleeping() == false);
            }
            else if (setIndex == sleeping_set)
            {
                MuliAssert(body->IsEnabled());
                MuliAssert(body->IsStatic() == false);
                MuliAssert(body->IsSleeping());
            }
        }

        for (int32 i = 0; i < int32(set.contactStates.size()); ++i)
        {
            Contact* contact = set.contactStates[i].contact;
            MuliAssert(seenContacts.insert(contact).second);
            MuliAssert(contact->setIndex == setIndex);
            MuliAssert(contact->colorIndex == null_index);
            MuliAssert(contact->localIndex == i);

            RigidBody* bodyA = contact->GetBodyA();
            RigidBody* bodyB = contact->GetBodyB();
            if (setIndex == static_set)
            {
                MuliAssert(bodyA->IsStatic() && bodyB->IsStatic());
            }
            else if (setIndex == disabled_set)
            {
                MuliAssert(bodyA->IsEnabled() == false || bodyB->IsEnabled() == false);
            }
            else if (setIndex == awake_set)
            {
                // Touching awake contacts should be in the constraint graph.
                MuliAssert(contact->IsEnabled() == false || contact->IsTouching() == false);
            }
            else if (setIndex == sleeping_set)
            {
                bool sleepingA = bodyA->IsStatic() || bodyA->IsSleeping();
                bool sleepingB = bodyB->IsStatic() || bodyB->IsSleeping();
                MuliAssert(sleepingA && sleepingB);
                MuliNotUsed(sleepingA);
                MuliNotUsed(sleepingB);
            }
        }

        for (int32 i = 0; i < int32(set.jointStates.size()); ++i)
        {
            Joint* joint = set.jointStates[i].joint;
            MuliAssert(seenJoints.insert(joint).second);
            MuliAssert(joint->setIndex == setIndex);
            MuliAssert(joint->colorIndex == null_index);
            MuliAssert(joint->localIndex == i);

            RigidBody* bodyA = joint->GetBodyA();
            RigidBody* bodyB = joint->GetBodyB();
            if (setIndex == static_set)
            {
                MuliAssert(bodyA->IsStatic() && bodyB->IsStatic());
            }
            else if (setIndex == disabled_set)
            {
                MuliAssert(bodyA->IsEnabled() == false || bodyB->IsEnabled() == false);
            }
            else if (setIndex == sleeping_set)
            {
                bool sleepingA = bodyA->IsStatic() || bodyA->IsSleeping();
                bool sleepingB = bodyB->IsStatic() || bodyB->IsSleeping();
                MuliAssert(sleepingA && sleepingB);
                MuliNotUsed(sleepingA);
                MuliNotUsed(sleepingB);
            }
        }
    }

    // Awake contacts and joints may be moved out of the solver set and into graph color batches.
    for (int32 colorIndex = 0; colorIndex < constraint_color_count; ++colorIndex)
    {
        const ConstraintBatch& batch = constraintGraph.batches[colorIndex];
        std::unordered_set<RigidBody*> colorBodies;
        uint32 colorBit = colorIndex == constraint_overflow_index ? 0 : 1u << colorIndex;
        MuliNotUsed(colorBit);

        for (int32 i = 0; i < int32(batch.contactStates.size()); ++i)
        {
            Contact* contact = batch.contactStates[i].contact;
            MuliAssert(seenContacts.insert(contact).second);
            MuliAssert(contact->setIndex == awake_set);
            MuliAssert(contact->colorIndex == colorIndex);
            MuliAssert(contact->localIndex == i);
            MuliAssert(contact->IsEnabled());
            MuliAssert(contact->IsTouching());

            RigidBody* bodyA = contact->GetBodyA();
            RigidBody* bodyB = contact->GetBodyB();
            MuliAssert(bodyA->IsEnabled());
            MuliAssert(bodyB->IsEnabled());

            if (colorIndex != constraint_overflow_index)
            {
                if (bodyA->IsStatic() == false)
                {
                    MuliAssert(bodyA->IsSleeping() == false);
                    MuliAssert((bodyA->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyA).second);
                }
                if (bodyB->IsStatic() == false)
                {
                    MuliAssert(bodyB->IsSleeping() == false);
                    MuliAssert((bodyB->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyB).second);
                }
            }
            else
            {
                MuliAssert(bodyA->IsStatic() || bodyA->IsSleeping() == false);
                MuliAssert(bodyB->IsStatic() || bodyB->IsSleeping() == false);
            }
        }

        for (int32 i = 0; i < int32(batch.jointStates.size()); ++i)
        {
            Joint* joint = batch.jointStates[i].joint;
            MuliAssert(seenJoints.insert(joint).second);
            MuliAssert(joint->setIndex == awake_set);
            MuliAssert(joint->colorIndex == colorIndex);
            MuliAssert(joint->localIndex == i);
            MuliAssert(joint->IsEnabled());

            RigidBody* bodyA = joint->GetBodyA();
            RigidBody* bodyB = joint->GetBodyB();
            MuliAssert(bodyA->IsEnabled());
            MuliAssert(bodyB->IsEnabled());

            if (colorIndex != constraint_overflow_index)
            {
                if (bodyA->IsStatic() == false)
                {
                    MuliAssert(bodyA->IsSleeping() == false);
                    MuliAssert((bodyA->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyA).second);
                }
                if (bodyB->IsStatic() == false)
                {
                    MuliAssert(bodyB->IsSleeping() == false);
                    MuliAssert((bodyB->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyB).second);
                }
            }
            else
            {
                MuliAssert(bodyA->IsStatic() || bodyA->IsSleeping() == false);
                MuliAssert(bodyB->IsStatic() || bodyB->IsSleeping() == false);
            }
        }
    }

    int32 contactCount = 0;
    for (Contact* contact = constraintGraph.contactList; contact; contact = contact->next)
    {
        MuliAssert(seenContacts.contains(contact));
        ++contactCount;
    }
    MuliAssert(contactCount == int32(seenContacts.size()));
    MuliAssert(contactCount == constraintGraph.contactCount);
    MuliNotUsed(contactCount);

    int32 jointCount = 0;
    for (Joint* joint = jointList; joint; joint = joint->next)
    {
        MuliAssert(seenJoints.contains(joint));
        ++jointCount;
    }
    MuliAssert(jointCount == int32(seenJoints.size()));
    MuliAssert(jointCount == this->jointCount);
    MuliNotUsed(jointCount);

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        if (body->IsStatic())
        {
            continue;
        }

        uint32 usedColors = 0;
        for (int32 colorIndex = 0; colorIndex < constraint_overflow_index; ++colorIndex)
        {
            const ConstraintBatch& batch = constraintGraph.batches[colorIndex];
            uint32 colorBit = 1u << colorIndex;

            for (const ContactState& state : batch.contactStates)
            {
                Contact* contact = state.contact;
                if (contact->GetBodyA() == body || contact->GetBodyB() == body)
                {
                    usedColors |= colorBit;
                }
            }

            for (const JointState& state : batch.jointStates)
            {
                Joint* joint = state.joint;
                if (joint->GetBodyA() == body || joint->GetBodyB() == body)
                {
                    usedColors |= colorBit;
                }
            }
        }

        MuliNotUsed(usedColors);
        MuliAssert(body->usedColors == usedColors);
    }
}

} // namespace muli3
