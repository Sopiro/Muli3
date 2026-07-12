#include "muli3/world.h"
#include "muli3/callbacks.h"
#include "muli3/capsule_shape.h"
#include "muli3/collider.h"
#include "muli3/contact_solver.h"
#include "muli3/parallel_for.h"
#include "muli3/raycast.h"
#include "muli3/shapes.h"

// #define VALIDATE_WORLD

namespace muli3
{

World::World(const WorldSettings* settings)
    : settings{ *settings }
    , constraintGraph{ this }
{
    poolAllocator.Register<Body>(512);
    poolAllocator.Register<Collider>(512);
    poolAllocator.Register<Contact>(1024);
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

    step.index = 0;
}

Body* World::CreateEmptyBody(const Transform& transform, Body::Type type)
{
    Body* b = poolAllocator.New<Body>(transform, type);
    AddBody(b);
    return b;
}

Body* World::CreateSphere(float radius, const Transform& transform, Body::Type type, float density)
{
    Body* b = CreateEmptyBody(transform, type);
    b->CreateSphereCollider(radius, identity, density);
    return b;
}

Body* World::CreateCapsule(float height, float radius, const Transform& transform, Body::Type type, float density)
{
    Body* b = CreateEmptyBody(transform, type);
    b->CreateCapsuleCollider(height, radius, identity, density);
    return b;
}

Body* World::CreateCapsule(
    const Vec3& point1, const Vec3& point2, float radius, const Transform& tf, Body::Type type, bool resetPosition, float density
)
{
    Body* b = CreateEmptyBody(tf, type);

    Vec3 center = (point1 + point2) * 0.5f;
    CapsuleShape capsule = CapsuleShape{ point1 - center, point2 - center, radius };
    b->CreateCollider(&capsule, identity, density);

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

Body* World::CreateBox(
    float width, float height, float depth, const Transform& transform, Body::Type type, float radius, float density
)
{
    Body* b = CreateEmptyBody(transform, type);
    b->CreateBoxCollider(width, height, depth, identity, radius, density);
    return b;
}

Body* World::CreateBox(const Vec3& size, const Transform& transform, Body::Type type, float radius, float density)
{
    return CreateBox(size.x, size.y, size.z, transform, type, radius, density);
}

Body* World::CreateBox(float size, const Transform& transform, Body::Type type, float radius, float density)
{
    return CreateBox(size, size, size, transform, type, radius, density);
}

Body* World::CreateConvex(
    std::span<const Vec3> vertices, const Transform& transform, Body::Type type, float radius, float density
)
{
    Body* b = CreateEmptyBody(transform, type);
    b->CreateConvexCollider(vertices, identity, radius, density);
    return b;
}

Body* World::CreateTriangle(
    const Vec3& a, const Vec3& b, const Vec3& c, const Transform& transform, Body::Type type, float radius, float density
)
{
    Body* body = CreateEmptyBody(transform, type);
    body->CreateTriangleCollider(a, b, c, identity, radius, density);
    return body;
}

Body* World::CreateTriangle(const Vec3 vertices[3], const Transform& transform, Body::Type type, float radius, float density)
{
    return CreateTriangle(vertices[0], vertices[1], vertices[2], transform, type, radius, density);
}

Body* World::CreateQuad(float width, float height, const Transform& transform, Body::Type type, float radius, float density)
{
    Vec3 vertices[4] = {
        Vec3{ 0.0f, 0.0f, 0.0f },
        Vec3{ width, 0.0f, 0.0f },
        Vec3{ width, height, 0.0f },
        Vec3{ 0.0f, height, 0.0f },
    };

    return CreatePolygon(vertices, transform, type, radius, density);
}

Body* World::CreatePolygon(
    std::span<const Vec3> vertices, const Transform& transform, Body::Type type, float radius, float density
)
{
    Body* body = CreateEmptyBody(transform, type);
    body->CreatePolygonCollider(vertices, identity, radius, density);
    return body;
}

Body* World::CreateHeightField(
    int32 sampleCountX,
    int32 sampleCountZ,
    std::span<const float> heightSamples,
    float cellSizeX,
    float cellSizeZ,
    const Transform& transform,
    const Vec3& offset,
    int32 blockSize
)
{
    Body* b = CreateEmptyBody(transform, Body::static_body);
    b->CreateHeightFieldCollider(sampleCountX, sampleCountZ, heightSamples, cellSizeX, cellSizeZ, offset, blockSize, identity);
    return b;
}

float World::Step(float dt)
{
    profile = {};
    MuliProfileZoneNR(world_step, "Step", true);

    MuliAssert(dt > 0.0f);

    if (dt <= 0.0f)
    {
        MuliProfileZoneEnd(world_step);
        return 0.0f;
    }

    step.index += 1;
    step.dt = dt;
    step.inv_dt = 1 / dt;

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
        ProfileScope profile_destroy_buffer{ &profile.post_solve };
        MuliProfileZoneNC(post_solve, "Post Solve", color::post_solve, true);

        for (Body* body : destroyBodyBuffer)
        {
            Destroy(body);
        }
        for (Joint* joint : destroyJointBuffer)
        {
            Destroy(joint);
        }

        destroyBodyBuffer.clear();
        destroyJointBuffer.clear();
        MuliProfileZoneEnd(post_solve);
    }

    MuliProfileZoneEnd(world_step);
    return 1.0f;
}

void World::Destroy(Body* body)
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

void World::Destroy(std::span<Body*> bodies)
{
    std::unordered_set<Body*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Body* b = bodies[i];

        if (!destroyed.contains(b))
        {
            destroyed.insert(b);
            Destroy(b);
        }
    }
}

void World::BufferDestroy(Body* body)
{
    MuliAssert(body != nullptr);
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<Body*> bodies)
{
    for (Body* body : bodies)
    {
        BufferDestroy(body);
    }
}

void World::Destroy(Joint* joint)
{
    Body* bodyA = joint->bodyA;
    Body* bodyB = joint->bodyB;

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

void World::RayCastAny(const Vec3& from, const Vec3& to, RayCastAnyCallback* callback) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.SetZero();

    struct TempCallback
    {
        RayCastAnyCallback* callback;

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput rayInput;
            rayInput.from = subInput.from;
            rayInput.to = subInput.to;
            rayInput.maxFraction = subInput.maxFraction;

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

bool World::RayCastClosest(const Vec3& from, const Vec3& to, RayCastClosestCallback* callback) const
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

    RayCastAny(from, to, &tempCallback);

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

            const Shape* colliderShape = collider->GetShape();
            const Transform& colliderTransform = collider->GetBody()->GetTransform();

            bool hit;
            if (colliderShape->GetType() == Shape::height_field)
            {
                const HeightFieldShape* heightField = (const HeightFieldShape*)colliderShape;
                hit = heightField->ShapeCast(colliderTransform, shape, tf, translation * input.maxFraction, &output);
            }
            else
            {
                hit =
                    ShapeCast(shape, tf, colliderShape, colliderTransform, translation * input.maxFraction, Vec3::zero, &output);
            }
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
    input.from = aabb.GetCenter();
    input.to = input.from + translation;
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
    const Vec3& from, const Vec3& to, std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.SetZero();

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
    const Vec3& from, const Vec3& to, std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
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

    RayCastAny(from, to, &tempCallback);

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
    input.from = aabb.GetCenter();
    input.to = input.from + translation;
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

            const Shape* colliderShape = collider->GetShape();
            const Transform& colliderTransform = collider->GetBody()->GetTransform();

            bool hit;
            if (colliderShape->GetType() == Shape::height_field)
            {
                const HeightFieldShape* heightField = (const HeightFieldShape*)colliderShape;
                hit = heightField->ShapeCast(colliderTransform, shape, tf, translation * input.maxFraction, &output);
            }
            else
            {
                hit =
                    ShapeCast(shape, tf, colliderShape, colliderTransform, translation * input.maxFraction, Vec3::zero, &output);
            }
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
    islandCount = 0;
    sleepingBodyCount = 0;

    SolverSet& awakeSet = solverSets[awake_set];
    int32 awakeBodyCount = int32(awakeSet.bodyStates.size());
    if (awakeBodyCount == 0)
    {
        return;
    }

    struct Island
    {
        int32 bodyStart;
        int32 contactStart;
        int32 jointStart;
        int32 bodyCount;
        int32 contactCount;
        int32 jointCount;
    };

    int32 stackPointer = 0;
    Body** stack = (Body**)linearAllocator.Allocate(awakeBodyCount * sizeof(Body*));

    Island* islands = (Island*)linearAllocator.Allocate(awakeBodyCount * sizeof(Island));

    int32 contactIndex0 = 0, bodyIndex0 = 0, jointIndex0 = 0;
    int32 contactIndex = 0, bodyIndex = 0, jointIndex = 0;
    BodyState** islandBodies = (BodyState**)linearAllocator.Allocate(awakeBodyCount * sizeof(BodyState*));
    Contact** islandContacts = (Contact**)linearAllocator.Allocate(constraintGraph.contactCount * sizeof(Contact*));
    Joint** islandJoints = (Joint**)linearAllocator.Allocate(jointCount * sizeof(Joint*));

    MuliProfileZoneNC(build_islands, "Build Islands", color::build_islands, true);
    ProfileScope profile_build_islands{ &profile.build_islands };

    for (size_t i = 0; i < awakeSet.bodyStates.size(); ++i)
    {
        Body* b = awakeSet.bodyStates[i].body;
        if (b->flag & Body::flag_island)
        {
            continue;
        }

        MuliAssert(!b->IsSleeping());
        MuliAssert(!b->IsStatic());
        MuliAssert(b->IsEnabled());

        MuliProfileZoneN(build_island, "Build Island", true);
        stack[stackPointer++] = b;
        b->flag |= Body::flag_island;

        while (stackPointer > 0)
        {
            Body* t = stack[--stackPointer];

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

                Body* other = ce->other;

                islandContacts[contactIndex++] = c;
                c->flag |= Contact::flag_island;

                if (other->flag & Body::flag_island)
                {
                    continue;
                }

                if (other->IsStatic())
                {
                    continue;
                }

                MuliAssert(stackPointer < awakeBodyCount);
                stack[stackPointer++] = other;
                other->flag |= Body::flag_island;
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                Joint* j = je->joint;

                if (j->flagIsland == true)
                {
                    continue;
                }

                Body* other = je->other;

                if (other->IsEnabled() == false)
                {
                    continue;
                }

                islandJoints[jointIndex++] = j;
                j->flagIsland = true;

                if (other->flag & Body::flag_island)
                {
                    continue;
                }

                if (other->IsStatic())
                {
                    continue;
                }

                MuliAssert(stackPointer < awakeBodyCount);
                stack[stackPointer++] = other;
                other->flag |= Body::flag_island;
            }
        }

        int32 islandContactCount = contactIndex - contactIndex0;
        int32 islandBodyCount = bodyIndex - bodyIndex0;
        int32 islandJointCount = jointIndex - jointIndex0;

        Island* island = &islands[islandCount++];
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

    SpinScope spinScope{ settings.thread_pool };

    const int32 minBodyRange = 64;
    const int32 minConstraintRange = 32;

    // Integrate velocities for all awake bodies
    MuliProfileZoneNC(integrate_velocities, "Integrate Velocities", color::integrate_velocities, true);
    {
        ProfileScope profile_integrate_velocities{ &profile.integrate_velocities };
        ParallelFor(
            0, bodyIndex, minBodyRange,
            [&](int32 i0, int32 i1) {
                MuliProfileZoneN(integrate_velocity, "Integrate Velocity", true);

                for (int32 i = i0; i < i1; ++i)
                {
                    BodyState* s = islandBodies[i];
                    Body* b = s->body;
                    s->motion.c0 = s->motion.c;
                    s->motion.q0 = s->motion.q;
                    s->motion.alpha0 = 0.0f;

                    b->flag &= ~Body::flag_sleeping;

                    if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
                        Length2(s->linearVelocity) > settings.rest_linear_tolerance || Length2(s->torque) > 0.0f ||
                        Length2(s->force) > 0.0f)
                    {
                        s->resting = 0.0f;
                    }

                    if (b->GetType() == Body::dynamic_body)
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
                }

                MuliProfileZoneEnd(integrate_velocity);
            },
            settings.thread_pool
        );
    }
    MuliProfileZoneEnd(integrate_velocities);

    // Prepare all constraints
    MuliProfileZoneNC(prepare_constraints, "Prepare Constraints", color::prepare_constraints, true);
    {
        ProfileScope profile_prepare{ &profile.prepare_constraints };
        int32 constraintCount = contactIndex + jointIndex;
        ParallelFor(
            0, constraintCount, minConstraintRange,
            [&](int32 i0, int32 i1) {
                MuliProfileZoneN(prepare_constraint, "Prepare Constraint", true);
                for (int32 i = i0; i < i1; ++i)
                {
                    if (i < contactIndex)
                    {
                        PrepareContact(islandContacts[i]->GetContactState());
                    }
                    else
                    {
                        PrepareJoint(islandJoints[i - contactIndex]->GetJointState(), step);
                    }
                }
                MuliProfileZoneEnd(prepare_constraint);
            },
            settings.thread_pool
        );
    }
    MuliProfileZoneEnd(prepare_constraints);

    MuliProfileZoneNC(warm_start_constraints, "Warm Start Constraints", color::warm_start, true);
    {
        ProfileScope profile_warm_start{ &profile.warm_start };

        ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

        MuliProfileZoneN(warm_start_contacts, "Warm Start Contacts Overflow", true);
        for (ContactState& state : overflow.contactStates)
        {
            WarmStartContact(&state);
        }
        MuliProfileZoneEnd(warm_start_contacts);

        MuliProfileZoneN(warm_start_joints, "Warm Start Joints Overflow", true);
        for (JointState& state : overflow.jointStates)
        {
            WarmStartJoint(&state);
        }
        MuliProfileZoneEnd(warm_start_joints);

        for (int32 color = 0; color < constraint_overflow_index; ++color)
        {
            ConstraintBatch& batch = constraintGraph.batches[color];
            int32 contactCount = int32(batch.contactStates.size());
            int32 constraintCount = contactCount + int32(batch.jointStates.size());
            ParallelFor(
                0, constraintCount, minConstraintRange,
                [&](int32 i0, int32 i1) {
                    MuliProfileZoneN(warm_start_constraint, "Warm Start Constraint", true);
                    for (int32 i = i0; i < i1; ++i)
                    {
                        if (i < contactCount)
                        {
                            WarmStartContact(&batch.contactStates[i]);
                        }
                        else
                        {
                            WarmStartJoint(&batch.jointStates[i - contactCount]);
                        }
                    }
                    MuliProfileZoneEnd(warm_start_constraint);
                },
                settings.thread_pool
            );
        }
    }
    MuliProfileZoneEnd(warm_start_constraints);

    MuliProfileZoneNC(solve_velocities, "Solve Velocities", color::solve_velocities, true);
    {
        ProfileScope profile_solve_velocities{ &profile.solve_velocities };

        for (int32 i = 0; i < settings.velocity_iterations; ++i)
        {
            ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

            MuliProfileZoneN(solve_velocity_contacts, "Solve Velocity Contact Overflow", true);
            for (ContactState& state : overflow.contactStates)
            {
                SolveContactVelocityConstraints(&state);
            }
            MuliProfileZoneEnd(solve_velocity_contacts);

            MuliProfileZoneN(solve_velocity_joints, "Solve Velocity Joint Overflow", true);
            for (JointState& state : overflow.jointStates)
            {
                SolveJointVelocityConstraints(&state, step);
            }
            MuliProfileZoneEnd(solve_velocity_joints);

            MuliProfileZoneN(solve_velocity, "Solve Velocity", true);
            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];
                int32 contactCount = int32(batch.contactStates.size());
                int32 constraintCount = contactCount + int32(batch.jointStates.size());

                ParallelFor(
                    0, constraintCount, minConstraintRange,
                    [&](int32 i0, int32 i1) {
                        MuliProfileZoneN(solve_velocity_constraint, "Solve Velocity Constraint", true);
                        for (int32 i = i0; i < i1; ++i)
                        {
                            if (i < contactCount)
                            {
                                SolveContactVelocityConstraints(&batch.contactStates[i]);
                            }
                            else
                            {
                                SolveJointVelocityConstraints(&batch.jointStates[i - contactCount], step);
                            }
                        }
                        MuliProfileZoneEnd(solve_velocity_constraint);
                    },
                    settings.thread_pool
                );
            }

            MuliProfileZoneEnd(solve_velocity);
        }
    }
    MuliProfileZoneEnd(solve_velocities);

    MuliProfileZoneNC(integrate_positions, "Integrate Positions", color::integrate_positions, true);
    {
        ProfileScope profile_integrate_positions{ &profile.integrate_positions };

        ParallelFor(
            0, bodyIndex, minBodyRange,
            [&](int32 i0, int32 i1) {
                MuliProfileZoneN(integrate_position, "Integrate Position", true);
                for (int32 i = i0; i < i1; ++i)
                {
                    BodyState* s = islandBodies[i];

                    s->force = Vec3::zero;
                    s->torque = Vec3::zero;

                    s->motion.c += s->linearVelocity * step.dt;

                    Quat w{ s->angularVelocity, 0.0f };
                    s->motion.q = s->motion.q + (w * s->motion.q) * step.dt * 0.5f;
                    s->motion.q.Normalize();
                }
                MuliProfileZoneEnd(integrate_position);
            },
            settings.thread_pool
        );
    }
    MuliProfileZoneEnd(integrate_positions);

    MuliProfileZoneNC(solve_positions, "Solve Positions", color::solve_positions, true);
    {
        ProfileScope profile_solve_positions{ &profile.solve_positions };

        for (int32 i = 0; i < settings.position_iterations; ++i)
        {
            MuliProfileZoneN(solve_position, "Solve Position", true);

            ConstraintBatch& overflow = constraintGraph.batches[constraint_overflow_index];

            MuliProfileZoneN(solve_position_contact, "Solve Position Contacts Overflow", true);
            for (ContactState& state : overflow.contactStates)
            {
                if (SolveContactPositionConstraints(&state) == false)
                {
                    if (!state.bodyA->body->IsStatic())
                    {
                        state.bodyA->resting = 0.0f;
                    }
                    if (!state.bodyB->body->IsStatic())
                    {
                        state.bodyB->resting = 0.0f;
                    }
                }
            }
            MuliProfileZoneEnd(solve_position_contact);

            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];

                ParallelFor(
                    0, batch.contactStates.size(), minConstraintRange,
                    [&](int32 i0, int32 i1) {
                        MuliProfileZoneN(solve_position_contact, "Solve Position Contact", true);
                        for (int32 i = i0; i < i1; ++i)
                        {
                            ContactState* state = &batch.contactStates[i];
                            if (SolveContactPositionConstraints(state) == false)
                            {
                                if (!state->bodyA->body->IsStatic())
                                {
                                    state->bodyA->resting = 0.0f;
                                }
                                if (!state->bodyB->body->IsStatic())
                                {
                                    state->bodyB->resting = 0.0f;
                                }
                            }
                        }
                        MuliProfileZoneEnd(solve_position_contact);
                    },
                    settings.thread_pool
                );
            }

            MuliProfileZoneEnd(solve_position);
        }
    }
    MuliProfileZoneEnd(solve_positions);

    MuliProfileZoneNC(sleep_and_sync, "Sleep And Sync", color::sleep_and_sync, true);
    {
        ProfileScope profile_sleep_and_sync{ &profile.sleep_and_sync };

        // Collider updates are computed in parallel and committed to the tree in order.
        struct ColliderSync
        {
            Collider* collider;
            AABB aabb;
            Vec3 displacement;
        };

        // Build per-body collider spans(prefix sums) so workers can write without synchronization.
        int32* colliderStarts = (int32*)linearAllocator.Allocate((bodyIndex + 1) * sizeof(int32));
        int32 colliderSyncCount = 0;
        for (int32 i = 0; i < bodyIndex; ++i)
        {
            Body* body = islandBodies[i]->body;
            colliderStarts[i] = colliderSyncCount;
            colliderSyncCount += body->GetColliderCount();
        }
        colliderStarts[bodyIndex] = colliderSyncCount;

        ColliderSync* colliderSyncs = (ColliderSync*)linearAllocator.Allocate(colliderSyncCount * sizeof(ColliderSync));

        int32 workerCount = settings.thread_pool ? settings.thread_pool->WorkerCount() : 1;
        int32 islandWordCount = (islandCount + 63) / 64;
        int32 bodyWordCount = (bodyIndex + 63) / 64;

        // Cache-line separated worker strides avoid false sharing
        int32 islandWordStride = (islandWordCount + 7) & ~7;
        int32 bodyWordStride = (bodyWordCount + 7) & ~7;

        int32 awakeIslandBitSize = workerCount * islandWordStride * sizeof(uint64);
        int32 destroyBodyBitSize = workerCount * bodyWordStride * sizeof(uint64);

        // Worker-local bits avoid atomics while collecting body results.
        uint64* awakeIslandBits = (uint64*)linearAllocator.Allocate(awakeIslandBitSize);
        uint64* destroyBodyBits = (uint64*)linearAllocator.Allocate(destroyBodyBitSize);
        std::memset(awakeIslandBits, 0, awakeIslandBitSize);
        std::memset(destroyBodyBits, 0, destroyBodyBitSize);

        const auto SetBit = [](uint64* bits, int32 bit) { bits[bit >> 6] |= uint64(1) << (bit & 63); };
        const auto GetBit = [](const uint64* bits, int32 bit) { return (bits[bit >> 6] & (uint64(1) << (bit & 63))) != 0; };

        // Compute body transforms and collider bounds in parallel.
        // The broad phase tree is updated below in order.
        ParallelFor(
            0, bodyIndex, minBodyRange,
            [&](int32 i0, int32 i1, int32 workerIndex) {
                MuliProfileZoneN(sync_bodies, "Sync Bodies", true);
                MuliAssert(workerIndex < workerCount);

                uint64* awakeBits = awakeIslandBits + workerIndex * islandWordStride;
                uint64* destroyBits = destroyBodyBits + workerIndex * bodyWordStride;

                for (int32 i = i0; i < i1; ++i)
                {
                    BodyState* s = islandBodies[i];
                    Body* body = s->body;
                    MuliAssert(body->IsStatic() == false);

                    if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
                        Length2(s->linearVelocity) > settings.rest_linear_tolerance)
                    {
                        if (GetBit(awakeBits, body->islandIndex) == false)
                        {
                            SetBit(awakeBits, body->islandIndex);
                        }
                    }

                    Transform transform0;
                    s->motion.GetTransform(0.0f, &transform0);
                    body->SynchronizeTransform();

                    if (settings.world_bounds.TestPoint(body->transform.p) == false)
                    {
                        SetBit(destroyBits, i);
                        continue;
                    }

                    int32 syncIndex = colliderStarts[i];
                    for (Collider* collider = body->colliderList; collider; collider = collider->next)
                    {
                        ColliderSync* sync = colliderSyncs + syncIndex++;
                        if (collider->IsEnabled() == false)
                        {
                            sync->collider = nullptr;
                            continue;
                        }

                        AABB aabb0;
                        AABB aabb1;
                        collider->GetShape()->ComputeAABB(transform0, &aabb0);
                        collider->GetShape()->ComputeAABB(body->transform, &aabb1);

                        Vec3 prediction = aabb1.GetCenter() - aabb0.GetCenter();
                        aabb1.min += prediction;
                        aabb1.max += prediction;

                        AABB aabb = AABB::Union(aabb0, aabb1);
                        if (constraintGraph.broadPhase.tree.GetAABB(collider->node).Contains(aabb))
                        {
                            sync->collider = nullptr;
                        }
                        else
                        {
                            sync->collider = collider;
                            sync->aabb = aabb;
                            sync->displacement = prediction;
                        }
                    }
                }

                MuliProfileZoneEnd(sync_bodies);
            },
            settings.thread_pool
        );

        spinScope.Close();

        // Merge worker-local results into worker 0 storage.
        uint64* awakeBits = awakeIslandBits;
        uint64* destroyBits = destroyBodyBits;
        for (int32 worker = 1; worker < workerCount; ++worker)
        {
            uint64* otherAwakeBits = awakeIslandBits + worker * islandWordStride;
            for (int32 i = 0; i < islandWordCount; ++i)
            {
                awakeBits[i] |= otherAwakeBits[i];
            }

            uint64* otherDestroyBits = destroyBodyBits + worker * bodyWordStride;
            for (int32 i = 0; i < bodyWordCount; ++i)
            {
                destroyBits[i] |= otherDestroyBits[i];
            }
        }

        // The broad phase tree and move buffer are not thread-safe, so commit serially.
        MuliProfileZoneNR(sync_colliders, "Sync Colliders", true);
        for (int32 i = 0; i < islandCount; ++i)
        {
            Island* island = islands + i;
            bool awakeIsland = GetBit(awakeBits, i);
            bool sleeping = settings.sleeping && awakeIsland == false;

            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                int32 b = island->bodyStart + j;
                BodyState* s = islandBodies[b];
                Body* body = s->body;

                if (GetBit(destroyBits, b))
                {
                    BufferDestroy(body);
                }
                else
                {
                    for (int32 k = colliderStarts[b]; k < colliderStarts[b + 1]; ++k)
                    {
                        const ColliderSync& sync = colliderSyncs[k];
                        if (sync.collider)
                        {
                            constraintGraph.broadPhase.Update(sync.collider, sync.aabb, sync.displacement, false);
                        }
                    }
                }

                if (awakeIsland)
                {
                    s->resting = 0.0f;
                }
                else
                {
                    s->resting += step.dt;
                    sleeping &= s->resting > settings.sleeping_time;
                }
            }

            if (sleeping == false)
            {
                continue;
            }

            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                BodyState* s = islandBodies[island->bodyStart + j];

                s->force = Vec3::zero;
                s->torque = Vec3::zero;
                s->linearVelocity = Vec3::zero;
                s->angularVelocity = Vec3::zero;
                s->resting = max_float;
                s->body->flag |= Body::flag_sleeping;
            }
        }
        MuliProfileZoneEnd(sync_colliders);

        linearAllocator.Free(destroyBodyBits, destroyBodyBitSize);
        linearAllocator.Free(awakeIslandBits, awakeIslandBitSize);
        linearAllocator.Free(colliderSyncs, colliderSyncCount * sizeof(ColliderSync));
        linearAllocator.Free(colliderStarts, (bodyIndex + 1) * sizeof(int32));
    }
    MuliProfileZoneEnd(sleep_and_sync);

    MuliProfileZoneNC(finalize, "Finalize", color::finalize, true);
    ProfileScope profile_finalize{ &profile.finalize };

    // Move solved graph contacts out of the graph before their bodies leave awakeSet.
    for (int32 i = contactIndex - 1; i >= 0; --i)
    {
        Contact* contact = islandContacts[i];

        Body* bodyA = contact->GetBodyA();
        Body* bodyB = contact->GetBodyB();

        contact->flag &= ~Contact::flag_island;

        SolverSetIndex targetSet;
        if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
        {
            targetSet = disabled_set;
        }
        else
        {
            bool awakeA = !bodyA->IsStatic() && !bodyA->IsSleeping();
            bool awakeB = !bodyB->IsStatic() && !bodyB->IsSleeping();
            targetSet = awakeA || awakeB ? awake_set : sleeping_set;
        }

        if (targetSet != awake_set)
        {
            MuliAssert(contact->colorIndex != null_index);

            ContactState state = std::move(constraintGraph.batches[contact->colorIndex].contactStates[contact->localIndex]);
            constraintGraph.RemoveContactFromGraph(contact);

            SolverSet& target = solverSets[targetSet];
            contact->setIndex = targetSet;
            contact->localIndex = int32(target.contactStates.size());
            target.contactStates.push_back(std::move(state));
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

        Body* bodyA = contact->GetBodyA();
        Body* bodyB = contact->GetBodyB();

        SolverSetIndex targetSet;
        if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
        {
            targetSet = disabled_set;
        }
        else
        {
            bool awakeA = !bodyA->IsStatic() && !bodyA->IsSleeping();
            bool awakeB = !bodyB->IsStatic() && !bodyB->IsSleeping();
            targetSet = awakeA || awakeB ? awake_set : sleeping_set;
        }

        if (targetSet != awake_set)
        {
            ContactState state = std::move(awakeSet.contactStates[i]);

            int32 last = int32(awakeSet.contactStates.size() - 1);
            if (i != last)
            {
                awakeSet.contactStates[i] = std::move(awakeSet.contactStates[last]);
                awakeSet.contactStates[i].contact->localIndex = i;
            }
            awakeSet.contactStates.pop_back();

            SolverSet& target = solverSets[targetSet];
            contact->setIndex = targetSet;
            contact->localIndex = int32(target.contactStates.size());
            target.contactStates.push_back(std::move(state));
            target.contactStates.back().contact = contact;
        }
    }

    // Move solved graph joints out of the graph before their bodies leave awakeSet.
    for (int32 i = jointIndex - 1; i >= 0; --i)
    {
        Joint* joint = islandJoints[i];

        Body* bodyA = joint->GetBodyA();
        Body* bodyB = joint->GetBodyB();

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
            bool awakeA = !bodyA->IsStatic() && !bodyA->IsSleeping();
            bool awakeB = !bodyB->IsStatic() && !bodyB->IsSleeping();
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

        Body* bodyA = joint->GetBodyA();
        Body* bodyB = joint->GetBodyB();

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
            bool awakeA = !bodyA->IsStatic() && !bodyA->IsSleeping();
            bool awakeB = !bodyB->IsStatic() && !bodyB->IsSleeping();
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
        Body* body = awakeSet.bodyStates[i].body;
        if (body->flag & Body::flag_island)
        {
            body->flag &= ~Body::flag_island;
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
#ifdef VALIDATE_WORLD
    Validate();
#endif

    linearAllocator.Free(islandJoints, jointCount * sizeof(Joint*));
    linearAllocator.Free(islandContacts, constraintGraph.contactCount * sizeof(Contact*));
    linearAllocator.Free(islandBodies, awakeBodyCount * sizeof(BodyState*));
    linearAllocator.Free(islands, awakeBodyCount * sizeof(Island));
    linearAllocator.Free(stack, awakeBodyCount * sizeof(Body*));
    MuliProfileZoneEnd(finalize);
}

// Joint factory functions

GrabJoint* World::CreateGrabJoint(Body* body, const Vec3& anchor, const Vec3& target, float frequency, float dampingRatio)
{
    if (body->world != this)
    {
        return nullptr;
    }

    GrabJoint* gj = poolAllocator.New<GrabJoint>(body, anchor, target, frequency, dampingRatio);

    AddJoint(gj);
    return gj;
}

FixedRotationJoint* World::CreateFixedRotationJoint(Body* body, float frequency, float dampingRatio)
{
    if (body->world != this)
    {
        return nullptr;
    }

    FixedRotationJoint* frj = poolAllocator.New<FixedRotationJoint>(body, frequency, dampingRatio);

    AddJoint(frj);
    return frj;
}

ConeSwingJoint* World::CreateConeSwingJoint(
    Body* bodyA, Body* bodyB, const Vec3& axis, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    ConeSwingJoint* csj = poolAllocator.New<ConeSwingJoint>(bodyA, bodyB, axis, maxAngle, frequency, dampingRatio);

    AddJoint(csj);
    return csj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& axis, float frequency, float dampingRatio
)
{
    return CreateLimitedRevoluteJoint(bodyA, bodyB, anchor, axis, -pi, pi, frequency, dampingRatio);
}

RevoluteJoint* World::CreateLimitedRevoluteJoint(
    Body* bodyA,
    Body* bodyB,
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

    RevoluteJoint* rj = poolAllocator.New<RevoluteJoint>(bodyA, bodyB, anchor, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(rj);
    return rj;
}

RevoluteAngleJoint* World::CreateRevoluteAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& axis, float frequency, float dampingRatio
)
{
    return CreateLimitedRevoluteAngleJoint(bodyA, bodyB, axis, -pi, pi, frequency, dampingRatio);
}

RevoluteAngleJoint* World::CreateLimitedRevoluteAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    RevoluteAngleJoint* raj =
        poolAllocator.New<RevoluteAngleJoint>(bodyA, bodyB, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(raj);
    return raj;
}

TwistAngleJoint* World::CreateTwistAngleJoint(
    Body* bodyA, Body* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    TwistAngleJoint* taj = poolAllocator.New<TwistAngleJoint>(bodyA, bodyB, axis, minAngle, maxAngle, frequency, dampingRatio);

    AddJoint(taj);
    return taj;
}

BallSocketJoint* World::CreateBallSocketJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    BallSocketJoint* bsj = poolAllocator.New<BallSocketJoint>(bodyA, bodyB, anchor, frequency, dampingRatio);

    AddJoint(bsj);
    return bsj;
}

DistanceJoint* World::CreateDistanceJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchorA, const Vec3& anchorB, float length, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    DistanceJoint* dj = poolAllocator.New<DistanceJoint>(bodyA, bodyB, anchorA, anchorB, length, length, frequency, dampingRatio);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(Body* bodyA, Body* bodyB, float length, float frequency, float dampingRatio)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, frequency, dampingRatio);
}

DistanceJoint* World::CreateLimitedDistanceJoint(
    Body* bodyA,
    Body* bodyB,
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

    DistanceJoint* dj =
        poolAllocator.New<DistanceJoint>(bodyA, bodyB, anchorA, anchorB, minLength, maxLength, frequency, dampingRatio);

    AddJoint(dj);
    return dj;
}

WeldJoint* World::CreateWeldJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    WeldJoint* wj = poolAllocator.New<WeldJoint>(bodyA, bodyB, anchor, frequency, dampingRatio);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    LineJoint* lj = poolAllocator.New<LineJoint>(bodyA, bodyB, anchor, dir, frequency, dampingRatio);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(Body* bodyA, Body* bodyB, float frequency, float dampingRatio)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), frequency, dampingRatio
    );
}

PrismaticJoint* World::CreatePrismaticJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    PrismaticJoint* pj = poolAllocator.New<PrismaticJoint>(bodyA, bodyB, anchor, dir, frequency, dampingRatio);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(Body* bodyA, Body* bodyB, float frequency, float dampingRatio)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), frequency, dampingRatio
    );
}

PulleyJoint* World::CreatePulleyJoint(
    Body* bodyA,
    Body* bodyB,
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

    PulleyJoint* pj = poolAllocator.New<PulleyJoint>(
        bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, ratio, frequency, dampingRatio
    );

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, float maxForce, float maxTorque, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    MotorJoint* mj = poolAllocator.New<MotorJoint>(bodyA, bodyB, anchor, maxForce, maxTorque, frequency, dampingRatio);

    AddJoint(mj);
    return mj;
}

void World::AddBody(Body* body)
{
    body->world = this;
    body->prev = bodyListTail;
    body->next = nullptr;
    body->colliderList = nullptr;
    body->colliderCount = 0;
    body->contactList = nullptr;
    body->jointList = nullptr;
    body->flag &= ~Body::flag_island;
    body->flag |= Body::flag_enabled;

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

    SolverSetIndex setIndex = body->type == Body::static_body ? static_set : awake_set;
    AddBodyState(body, setIndex);
    ++bodyCount;
}

void World::FreeBody(Body* body)
{
    poolAllocator.Delete(body);
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
        poolAllocator.Delete((GrabJoint*)joint);
        break;
    case Joint::Type::fixed_rotation_joint:
        poolAllocator.Delete((FixedRotationJoint*)joint);
        break;
    case Joint::Type::cone_swing_joint:
        poolAllocator.Delete((ConeSwingJoint*)joint);
        break;
    case Joint::Type::revolute_joint:
        poolAllocator.Delete((RevoluteJoint*)joint);
        break;
    case Joint::Type::revolute_angle_joint:
        poolAllocator.Delete((RevoluteAngleJoint*)joint);
        break;
    case Joint::Type::twist_angle_joint:
        poolAllocator.Delete((TwistAngleJoint*)joint);
        break;
    case Joint::Type::ball_socket_joint:
        poolAllocator.Delete((BallSocketJoint*)joint);
        break;
    case Joint::Type::distance_joint:
        poolAllocator.Delete((DistanceJoint*)joint);
        break;
    case Joint::Type::weld_joint:
        poolAllocator.Delete((WeldJoint*)joint);
        break;
    case Joint::Type::line_joint:
        poolAllocator.Delete((LineJoint*)joint);
        break;
    case Joint::Type::prismatic_joint:
        poolAllocator.Delete((PrismaticJoint*)joint);
        break;
    case Joint::Type::pulley_joint:
        poolAllocator.Delete((PulleyJoint*)joint);
        break;
    case Joint::Type::motor_joint:
        poolAllocator.Delete((MotorJoint*)joint);
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
        return poolAllocator.New<SphereShape>(*(const SphereShape*)shape, transform);
    }
    case Shape::capsule:
    {
        return poolAllocator.New<CapsuleShape>(*(const CapsuleShape*)shape, transform);
    }
    case Shape::box:
    {
        return poolAllocator.New<BoxShape>(*(const BoxShape*)shape, transform);
    }
    case Shape::convex:
    {
        return poolAllocator.New<ConvexShape>(*(const ConvexShape*)shape, transform);
    }
    case Shape::triangle:
    {
        return poolAllocator.New<TriangleShape>(*(const TriangleShape*)shape, transform);
    }
    case Shape::polygon:
    {
        return poolAllocator.New<PolygonShape>(*(const PolygonShape*)shape, transform);
    }
    case Shape::height_field:
    {
        return poolAllocator.New<HeightFieldShape>(*(const HeightFieldShape*)shape, transform);
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
        poolAllocator.Delete((SphereShape*)shape);
        break;
    case Shape::capsule:
        poolAllocator.Delete((CapsuleShape*)shape);
        break;
    case Shape::box:
        poolAllocator.Delete((BoxShape*)shape);
        break;
    case Shape::convex:
        poolAllocator.Delete((ConvexShape*)shape);
        break;
    case Shape::triangle:
        poolAllocator.Delete((TriangleShape*)shape);
        break;
    case Shape::polygon:
        poolAllocator.Delete((PolygonShape*)shape);
        break;
    case Shape::height_field:
        poolAllocator.Delete((HeightFieldShape*)shape);
        break;
    default:
        MuliAssert(false);
        break;
    }
}

BodyState* World::AddBodyState(Body* body, SolverSetIndex setIndex)
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

void World::RemoveBodyState(Body* body)
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

void World::TransferBody(Body* body, SolverSetIndex targetSet)
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

    set.contactStates.push_back(std::move(state));
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
        set.contactStates[index] = std::move(set.contactStates[last]);
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

        ContactState state = std::move(constraintGraph.batches[contact->colorIndex].contactStates[contact->localIndex]);
        constraintGraph.RemoveContactFromGraph(contact);

        SolverSet& target = solverSets[targetSet];
        contact->setIndex = targetSet;
        contact->localIndex = int32(target.contactStates.size());
        target.contactStates.push_back(std::move(state));
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
    ContactState state = std::move(source.contactStates[sourceIndex]);

    int32 last = int32(source.contactStates.size() - 1);
    if (sourceIndex != last)
    {
        source.contactStates[sourceIndex] = std::move(source.contactStates[last]);
        source.contactStates[sourceIndex].contact->localIndex = sourceIndex;
    }
    source.contactStates.pop_back();

    if (targetSet == awake_set && contact->IsTouching() && contact->IsEnabled())
    {
        contact->setIndex = awake_set;
        constraintGraph.AddContactToGraph(contact, std::move(state));
    }
    else
    {
        contact->setIndex = targetSet;
        contact->localIndex = int32(target.contactStates.size());
        target.contactStates.push_back(std::move(state));
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

void World::WakeBody(Body* body)
{
    if (body == nullptr || body->IsStatic() || body->IsEnabled() == false)
    {
        return;
    }

    if (body->IsSleeping() == false && body->setIndex == awake_set)
    {
        return;
    }

    body->flag &= ~Body::flag_sleeping;
    body->GetBodyState()->resting = 0.0f;

    TransferBody(body, awake_set);

    for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
    {
        Contact* contact = ce->contact;
        Body* other = ce->other;

        if (other->IsEnabled() == false)
        {
            continue;
        }

        TransferContact(contact, awake_set);
    }

    for (JointEdge* je = body->jointList; je; je = je->next)
    {
        Joint* joint = je->joint;
        Body* other = je->other;

        if (other->IsEnabled() == false)
        {
            continue;
        }

        TransferJoint(joint, awake_set);
    }
}

void World::SleepBody(Body* body)
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

void World::WakeIsland(Body* body)
{
    if (body == nullptr || body->IsStatic() || body->IsEnabled() == false)
    {
        return;
    }

    if (body->IsSleeping() == false && body->setIndex == awake_set)
    {
        return;
    }

    GrowableStack<Body*, 64> stack;
    stack.push_back(body);

    while (stack.size() > 0)
    {
        Body* b = stack.back();
        stack.pop_back();

        if (b->IsStatic() || b->IsEnabled() == false || (b->IsSleeping() == false && b->setIndex == awake_set))
        {
            continue;
        }

        b->flag &= ~Body::flag_sleeping;
        b->GetBodyState()->resting = 0.0f;
        TransferBody(b, awake_set);

        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            Body* other = ce->other;

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
            Body* other = je->other;

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

void World::SleepIsland(Body* body)
{
    if (body == nullptr || body->IsStatic() || body->IsEnabled() == false)
    {
        return;
    }

    if (body->IsSleeping() && body->setIndex == sleeping_set)
    {
        return;
    }

    GrowableStack<Body*, 64> stack;
    GrowableStack<Body*, 64> bodies;

    stack.push_back(body);
    body->flag |= Body::flag_island;

    while (stack.size() > 0)
    {
        Body* b = stack.back();
        stack.pop_back();
        bodies.push_back(b);

        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            Body* other = ce->other;

            if ((contact->flag & Contact::flag_touching) == 0 || (contact->flag & Contact::flag_enabled) == 0)
            {
                continue;
            }

            if (other->IsStatic() || other->IsEnabled() == false || (other->flag & Body::flag_island))
            {
                continue;
            }

            other->flag |= Body::flag_island;
            stack.push_back(other);
        }

        for (JointEdge* je = b->jointList; je; je = je->next)
        {
            Body* other = je->other;

            if (other->IsStatic() || other->IsEnabled() == false || (other->flag & Body::flag_island))
            {
                continue;
            }

            other->flag |= Body::flag_island;
            stack.push_back(other);
        }
    }

    for (Body* b : bodies)
    {
        BodyState* state = b->GetBodyState();
        state->resting = max_float;
        state->force = Vec3::zero;
        state->torque = Vec3::zero;
        state->linearVelocity = Vec3::zero;
        state->angularVelocity = Vec3::zero;

        b->flag |= Body::flag_sleeping;
    }

    for (Body* b : bodies)
    {
        for (ContactEdge* ce = b->contactList; ce; ce = ce->next)
        {
            Contact* contact = ce->contact;
            Body* bodyA = contact->GetBodyA();
            Body* bodyB = contact->GetBodyB();

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
            Body* bodyA = joint->GetBodyA();
            Body* bodyB = joint->GetBodyB();

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

    for (Body* b : bodies)
    {
        b->flag &= ~Body::flag_island;
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
            Body* body = set.bodyStates[i].body;
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

            Body* bodyA = contact->GetBodyA();
            Body* bodyB = contact->GetBodyB();
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

            Body* bodyA = joint->GetBodyA();
            Body* bodyB = joint->GetBodyB();
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
        std::unordered_set<Body*> colorBodies;
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

            Body* bodyA = contact->GetBodyA();
            Body* bodyB = contact->GetBodyB();
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
                if (bodyA != bodyB && bodyB->IsStatic() == false)
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

            Body* bodyA = joint->GetBodyA();
            Body* bodyB = joint->GetBodyB();
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
                if (bodyA != bodyB && bodyB->IsStatic() == false)
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

    int32 numContacts = 0;
    for (Contact* contact = constraintGraph.contactList; contact; contact = contact->next)
    {
        MuliAssert(seenContacts.contains(contact));
        ++numContacts;
    }
    MuliAssert(numContacts == int32(seenContacts.size()));
    MuliAssert(numContacts == constraintGraph.contactCount);
    MuliNotUsed(numContacts);

    int32 numJoints = 0;
    for (Joint* joint = jointList; joint; joint = joint->next)
    {
        MuliAssert(seenJoints.contains(joint));
        ++numJoints;
    }
    MuliAssert(numJoints == int32(seenJoints.size()));
    MuliAssert(numJoints == this->jointCount);
    MuliNotUsed(numJoints);

    for (Body* body = bodyList; body; body = body->next)
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
