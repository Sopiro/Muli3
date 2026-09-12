#include "muli3/world.h"
#include "muli3/bitset.h"
#include "muli3/callbacks.h"
#include "muli3/collider.h"
#include "muli3/constraint.h"
#include "muli3/parallel_for.h"
#include "muli3/raycast.h"
#include "muli3/shapes.h"

#include "dynamics/contact/contact_solver.h"

// #define VALIDATE_WORLD

namespace muli3
{

World::World(const WorldSettings* settings)
    : settings{ *settings }
    , constraintGraph{ this }
    , solverSets{ solver_set_count }
{
    poolAllocator.Register<Body>(512);
    poolAllocator.Register<Collider>(512);
}

World::~World()
{
    Reset();
}

void World::Reset()
{
    while (bodies.empty() == false)
    {
        Destroy(bodies.back());
    }

    MuliAssert(bodies.empty());
    MuliAssert(joints.empty());
    MuliAssert(constraintGraph.contacts.empty());

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
        MuliAssert(constraintGraph.batches[i].blockContacts.Empty());
        MuliAssert(constraintGraph.batches[i].scalarContacts.Empty());
        MuliAssert(constraintGraph.batches[i].scalarJoints.Empty());
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

Body* World::CreateCylinder(
    float height,
    float topRadius,
    float bottomRadius,
    uint16 segmentCount,
    const Transform& transform,
    Body::Type type,
    float convexRadius,
    float density
)
{
    MuliAssert(3 <= segmentCount && segmentCount <= (std::numeric_limits<uint16>::max() / 6));
    MuliAssert(height > 0.0f);
    MuliAssert(topRadius >= 0.0f && bottomRadius >= 0.0f);

    float halfHeight = height * 0.5f;
    bool hasTop = topRadius > linear_slop;
    bool hasBottom = bottomRadius > linear_slop;
    MuliAssert(hasTop || hasBottom);

    if (hasTop && hasBottom)
    {
        std::vector<Vec3> vertices(segmentCount * 2);
        std::vector<int32> indices(segmentCount * 6);
        std::vector<Face> faces(segmentCount + 2);

        // Keep the complete polygon caps and quad sides for stable manifold clipping.
        for (int32 i = 0; i < segmentCount; ++i)
        {
            float angle = two_pi * i / segmentCount;
            float x = std::cos(angle);
            float z = std::sin(angle);
            vertices[i * 2] = Vec3{ x * bottomRadius, -halfHeight, z * bottomRadius };
            vertices[i * 2 + 1] = Vec3{ x * topRadius, halfHeight, z * topRadius };
        }

        uint16 indexCount = 0;

        faces[0].vertexStart = indexCount;
        faces[0].vertexCount = segmentCount;
        for (int32 i = 0; i < segmentCount; ++i)
        {
            indices[indexCount++] = i * 2;
        }

        faces[1].vertexStart = indexCount;
        faces[1].vertexCount = segmentCount;
        for (int32 i = segmentCount - 1; i >= 0; --i)
        {
            indices[indexCount++] = i * 2 + 1;
        }

        for (int32 i = 0; i < segmentCount; ++i)
        {
            int32 next = (i + 1) % segmentCount;
            Face& face = faces[i + 2];
            face.vertexStart = indexCount;
            face.vertexCount = 4;
            indices[indexCount++] = i * 2;
            indices[indexCount++] = i * 2 + 1;
            indices[indexCount++] = next * 2 + 1;
            indices[indexCount++] = next * 2;
        }

        MuliAssert(indexCount == segmentCount * 6);
        ConvexShape convex{ vertices, indices, faces, convexRadius };
        Body* body = CreateEmptyBody(transform, type);
        body->CreateCollider(&convex, identity, density);
        return body;
    }

    std::vector<Vec3> vertices(segmentCount + 1);
    std::vector<int32> indices(segmentCount * 4);
    std::vector<Face> faces(segmentCount + 1);
    float ringRadius = hasBottom ? bottomRadius : topRadius;
    float ringY = hasBottom ? -halfHeight : halfHeight;
    float apexY = -ringY;

    for (int32 i = 0; i < segmentCount; ++i)
    {
        float angle = two_pi * i / segmentCount;
        vertices[i] = Vec3{ ringRadius * std::cos(angle), ringY, ringRadius * std::sin(angle) };
    }
    int32 apex = segmentCount;
    vertices[apex] = Vec3{ 0.0f, apexY, 0.0f };

    uint16 indexCount = 0;
    faces[0].vertexStart = indexCount;
    faces[0].vertexCount = segmentCount;
    if (hasBottom)
    {
        for (int32 i = 0; i < segmentCount; ++i)
        {
            indices[indexCount++] = i;
        }
    }
    else
    {
        for (int32 i = segmentCount - 1; i >= 0; --i)
        {
            indices[indexCount++] = i;
        }
    }

    for (int32 i = 0; i < segmentCount; ++i)
    {
        int32 next = (i + 1) % segmentCount;
        Face& face = faces[i + 1];
        face.vertexStart = indexCount;
        face.vertexCount = 3;
        if (hasBottom)
        {
            indices[indexCount++] = i;
            indices[indexCount++] = apex;
            indices[indexCount++] = next;
        }
        else
        {
            indices[indexCount++] = apex;
            indices[indexCount++] = i;
            indices[indexCount++] = next;
        }
    }

    MuliAssert(indexCount == segmentCount * 4);
    ConvexShape convex{ vertices, indices, faces, convexRadius };
    Body* body = CreateEmptyBody(transform, type);
    body->CreateCollider(&convex, identity, density);
    return body;
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

Body* World::CreateMesh(std::span<const Vec3> vertices, std::span<const int32> indices, const Transform& transform)
{
    Body* body = CreateEmptyBody(transform, Body::static_body);
    body->CreateMeshCollider(vertices, indices);
    return body;
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
    while (body->joints.empty() == false)
    {
        Joint* joint = body->joints.back();
        Body* other = joint->bodyA == body ? joint->bodyB : joint->bodyA;
        other->Awake();

        Destroy(joint);
    }

    while (body->colliders.empty() == false)
    {
        body->DestroyCollider(body->colliders.back());
    }

    int32 index = body->worldIndex;
    MuliAssert(0 <= index && index < int32(bodies.size()));
    MuliAssert(bodies[index] == body);

    Body* moved = bodies.back();
    bodies[index] = moved;
    moved->worldIndex = index;
    bodies.pop_back();

    RemoveBodyState(body);
    FreeBody(body);
}

void World::Destroy(std::span<Body*> inBodies)
{
    for (Body* body : inBodies)
    {
        Destroy(body);
    }
}

void World::BufferDestroy(Body* body)
{
    MuliAssert(body != nullptr);
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<Body*> inBodies)
{
    for (Body* body : inBodies)
    {
        BufferDestroy(body);
    }
}

void World::Destroy(Joint* joint)
{
    Body* bodyA = joint->bodyA;
    Body* bodyB = joint->bodyB;

    MuliAssert(bodyA->world == this);
    MuliAssert(bodyB->world == this);

    // Remove from the world.
    int32 index = joint->worldIndex;
    MuliAssert(0 <= index && index < int32(joints.size()));
    MuliAssert(joints[index] == joint);

    Joint* moved = joints.back();
    joints[index] = moved;
    moved->worldIndex = index;
    joints.pop_back();

    // Remove from body A.
    index = joint->bodyIndexA;
    MuliAssert(0 <= index && index < int32(bodyA->joints.size()));
    MuliAssert(bodyA->joints[index] == joint);
    moved = bodyA->joints.back();
    bodyA->joints[index] = moved;
    if (moved->bodyA == bodyA)
    {
        moved->bodyIndexA = index;
    }
    else
    {
        moved->bodyIndexB = index;
    }
    bodyA->joints.pop_back();

    // A self joint is stored once.
    if (bodyA != bodyB)
    {
        index = joint->bodyIndexB;
        MuliAssert(0 <= index && index < int32(bodyB->joints.size()));
        MuliAssert(bodyB->joints[index] == joint);
        moved = bodyB->joints.back();
        bodyB->joints[index] = moved;
        if (moved->bodyA == bodyB)
        {
            moved->bodyIndexA = index;
        }
        else
        {
            moved->bodyIndexB = index;
        }
        bodyB->joints.pop_back();
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
}

void World::Destroy(std::span<Joint*> inJoints)
{
    for (Joint* joint : inJoints)
    {
        Destroy(joint);
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyJointBuffer.push_back(joint);
}

void World::BufferDestroy(std::span<Joint*> inJoints)
{
    for (Joint* joint : inJoints)
    {
        BufferDestroy(joint);
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

            if (collider->shape->GetType() > Shape::triangle)
            {
                if (Collide2(collider->shape, collider->body->transform, &region, transform))
                {
                    return callback->OnQuery(collider);
                }
            }
            else
            {
                if (Collide(collider->shape, collider->body->transform, &region, transform))
                {
                    return callback->OnQuery(collider);
                }
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
            else if (colliderShape->GetType() == Shape::mesh)
            {
                const MeshShape* mesh = (const MeshShape*)colliderShape;
                hit = mesh->ShapeCast(colliderTransform, shape, tf, translation * input.maxFraction, &output);
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
            else if (colliderShape->GetType() == Shape::mesh)
            {
                const MeshShape* mesh = (const MeshShape*)colliderShape;
                hit = mesh->ShapeCast(colliderTransform, shape, tf, translation * input.maxFraction, &output);
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
    SolverSet& awakeSet = solverSets[awake_set];
    int32 awakeBodyCount = int32(awakeSet.bodyStates.size());
    if (awakeBodyCount == 0)
    {
        return;
    }

    islandCount = 0;
    sleepingBodyCount = 0;

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

    int32 totalContactCount = int32(constraintGraph.contacts.size());
    int32 totalJointCount = int32(joints.size());
    Contact** islandContacts = (Contact**)linearAllocator.Allocate(totalContactCount * sizeof(Contact*));
    Joint** islandJoints = (Joint**)linearAllocator.Allocate(totalJointCount * sizeof(Joint*));

    MuliProfileZoneNC(build_islands, "Build Islands", color::build_islands, true);
    ProfileScope profile_build_islands{ &profile.build_islands };

    for (int32 i = 0; i < awakeBodyCount; ++i)
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

            for (Contact* c : t->contacts)
            {
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

                Body* other = c->GetBodyA() == t ? c->GetBodyB() : c->GetBodyA();

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

            for (Joint* j : t->joints)
            {
                if (j->flagIsland == true)
                {
                    continue;
                }

                Body* other = j->bodyA == t ? j->bodyB : j->bodyA;

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
                    b->flag &= ~Body::flag_sleeping;

                    if (Length2(s->torque) > 0.0f || Length2(s->force) > 0.0f)
                    {
                        s->resting = 0.0f;
                    }

                    s->motion.c0 = s->motion.c;
                    s->motion.q0 = s->motion.q;
                    s->motion.alpha0 = 0.0f;

                    if (b->GetType() == Body::dynamic_body)
                    {
                        if (settings.apply_gravity)
                        {
                            s->linearVelocity += settings.gravity * s->gravityScale * step.dt;
                        }

                        s->linearVelocity += s->force * s->invMass * step.dt;
                        s->angularVelocity += b->GetWorldInverseInertiaTensor() * s->torque * step.dt;
                        s->angularVelocity = SolveGyroscopic(s->motion.q, b->inertia, s->angularVelocity, step.dt);

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

#ifdef VALIDATE_WORLD
    int32 storedContactCount = 0;
    for (int32 color = 0; color < constraint_color_count; ++color)
    {
        ConstraintBatch& batch = constraintGraph.batches[color];
        storedContactCount += batch.blockContacts.Count() + batch.scalarContacts.Count();
    }
    MuliAssert(storedContactCount == contactIndex);
    MuliNotUsed(storedContactCount);
#endif

    // Prepare all constraints
    MuliProfileZoneNC(prepare_constraints, "Prepare Constraints", color::prepare_constraints, true);
    {
        ProfileScope profile_prepare{ &profile.prepare_constraints };

        for (int32 color = 0; color < constraint_color_count; ++color)
        {
            ConstraintBatch& batch = constraintGraph.batches[color];
            int32 blockCount = batch.blockContacts.BlockCount();
            int32 scalarCount = batch.scalarContacts.Count();
            int32 contactCount = blockCount + scalarCount;

            ParallelFor(
                0, contactCount, minConstraintRange,
                [&](int32 i0, int32 i1) {
                    MuliProfileZoneN(prepare_contact, "Prepare Contact", true);
                    for (int32 i = i0; i < i1; ++i)
                    {
                        if (i < blockCount)
                        {
                            PrepareContactBlock(&batch.blockContacts, solverSets.data(), i);
                        }
                        else
                        {
                            int32 scalarIndex = i - blockCount;
                            PrepareContactScalar(
                                &batch.scalarContacts.states[scalarIndex], &batch.scalarContacts.constraints[scalarIndex]
                            );
                        }
                    }
                    MuliProfileZoneEnd(prepare_contact);
                },
                settings.thread_pool
            );
        }

        ParallelFor(
            0, jointIndex, minConstraintRange,
            [&](int32 i0, int32 i1) {
                MuliProfileZoneN(prepare_joint, "Prepare Joint", true);
                for (int32 i = i0; i < i1; ++i)
                {
                    PrepareJoint(islandJoints[i]->GetJointState(), step);
                }
                MuliProfileZoneEnd(prepare_joint);
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
        for (int32 i = 0; i < overflow.scalarContacts.Count(); ++i)
        {
            WarmStartContactScalar(&overflow.scalarContacts.states[i], &overflow.scalarContacts.constraints[i]);
        }
        MuliProfileZoneEnd(warm_start_contacts);

        MuliProfileZoneN(warm_start_joints, "Warm Start Joints Overflow", true);
        for (JointState& state : overflow.scalarJoints.states)
        {
            WarmStartJoint(&state);
        }
        MuliProfileZoneEnd(warm_start_joints);

        for (int32 color = 0; color < constraint_overflow_index; ++color)
        {
            ConstraintBatch& batch = constraintGraph.batches[color];
            int32 blockCount = batch.blockContacts.BlockCount();
            int32 scalarCount = batch.scalarContacts.Count();
            int32 contactCount = blockCount + scalarCount;
            int32 constraintCount = contactCount + batch.scalarJoints.Count();
            ParallelFor(
                0, constraintCount, minConstraintRange,
                [&](int32 i0, int32 i1) {
                    MuliProfileZoneN(warm_start_constraint, "Warm Start Constraint", true);
                    for (int32 i = i0; i < i1; ++i)
                    {
                        if (i < blockCount)
                        {
                            WarmStartContactBlock(&batch.blockContacts, i);
                        }
                        else if (i < contactCount)
                        {
                            int32 scalarIndex = i - blockCount;
                            WarmStartContactScalar(
                                &batch.scalarContacts.states[scalarIndex], &batch.scalarContacts.constraints[scalarIndex]
                            );
                        }
                        else
                        {
                            WarmStartJoint(&batch.scalarJoints.states[i - contactCount]);
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
            for (int32 j = 0; j < overflow.scalarContacts.Count(); ++j)
            {
                SolveContactVelocityScalar(&overflow.scalarContacts.states[j], &overflow.scalarContacts.constraints[j]);
            }
            MuliProfileZoneEnd(solve_velocity_contacts);

            MuliProfileZoneN(solve_velocity_joints, "Solve Velocity Joint Overflow", true);
            for (JointState& state : overflow.scalarJoints.states)
            {
                SolveJointVelocity(&state, step);
            }
            MuliProfileZoneEnd(solve_velocity_joints);

            MuliProfileZoneN(solve_velocity, "Solve Velocity", true);
            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];
                int32 blockCount = batch.blockContacts.BlockCount();
                int32 scalarCount = batch.scalarContacts.Count();
                int32 contactCount = blockCount + scalarCount;
                int32 constraintCount = contactCount + batch.scalarJoints.Count();

                ParallelFor(
                    0, constraintCount, minConstraintRange,
                    [&](int32 i0, int32 i1) {
                        MuliProfileZoneN(solve_velocity_constraint, "Solve Velocity Constraint", true);
                        for (int32 i = i0; i < i1; ++i)
                        {
                            if (i < blockCount)
                            {
                                SolveContactVelocityBlock(&batch.blockContacts, i);
                            }
                            else if (i < contactCount)
                            {
                                int32 scalarIndex = i - blockCount;
                                SolveContactVelocityScalar(
                                    &batch.scalarContacts.states[scalarIndex], &batch.scalarContacts.constraints[scalarIndex]
                                );
                            }
                            else
                            {
                                SolveJointVelocity(&batch.scalarJoints.states[i - contactCount], step);
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
            for (int32 j = 0; j < overflow.scalarContacts.Count(); ++j)
            {
                ContactState& state = overflow.scalarContacts.states[j];
                ScalarContactConstraint& constraint = overflow.scalarContacts.constraints[j];
                SolveContactPositionScalar(&state, &constraint);
            }
            MuliProfileZoneEnd(solve_position_contact);

            for (int32 color = 0; color < constraint_overflow_index; ++color)
            {
                ConstraintBatch& batch = constraintGraph.batches[color];
                int32 blockCount = batch.blockContacts.BlockCount();
                int32 scalarCount = batch.scalarContacts.Count();
                int32 constraintCount = blockCount + scalarCount;

                ParallelFor(
                    0, constraintCount, minConstraintRange,
                    [&](int32 i0, int32 i1) {
                        MuliProfileZoneN(solve_position_contact, "Solve Position Contact", true);
                        for (int32 i = i0; i < i1; ++i)
                        {
                            if (i < blockCount)
                            {
                                BlockContactArray& contacts = batch.blockContacts;
                                SolveContactPositionBlock(&contacts, i);
                            }
                            else
                            {
                                ScalarContactConstraint& constraint = batch.scalarContacts.constraints[i - blockCount];
                                ContactState& state = batch.scalarContacts.states[i - blockCount];
                                SolveContactPositionScalar(&state, &constraint);
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
            colliderStarts[i] = colliderSyncCount;
            colliderSyncCount += islandBodies[i]->body->GetColliderCount();
        }
        colliderStarts[bodyIndex] = colliderSyncCount;

        ColliderSync* colliderSyncs = (ColliderSync*)linearAllocator.Allocate(colliderSyncCount * sizeof(ColliderSync));

        int32 workerCount = settings.thread_pool ? settings.thread_pool->WorkerCount() : 1;

        // Worker-local bits avoid atomics while collecting body results.
        Bitset activeIslandBits = AllocateBitset(islandCount, workerCount, &linearAllocator);
        Bitset destroyBodyBits = AllocateBitset(bodyIndex, workerCount, &linearAllocator);

        // Compute body transforms and collider bounds in parallel.
        // The broad phase tree is updated below in order.
        ParallelFor(
            0, bodyIndex, minBodyRange,
            [&](int32 i0, int32 i1, int32 workerIndex) {
                MuliProfileZoneN(sync_bodies, "Sync Bodies", true);
                MuliAssert(workerIndex < workerCount);

                for (int32 i = i0; i < i1; ++i)
                {
                    BodyState* s = islandBodies[i];
                    Body* body = s->body;
                    MuliAssert(body->IsStatic() == false);

                    Quat q0 = s->motion.q0;
                    Quat q = s->motion.q;

                    // q and -q represent the same orientation. Pick the sign that gives the shortest delta.
                    if (Dot(q0, q) < 0.0f)
                    {
                        q = -q;
                    }

                    // Convert displacement of COM to the average linear velocity for this step.
                    Vec3 deltaPosition = s->motion.c - s->motion.c0;
                    Vec3 deltaLinearVelocity = deltaPosition * step.inv_dt;

                    // q0 and q bracket the whole step. A rotation quaternion is (axis * sin(theta / 2), cos(theta / 2)).
                    // For small theta, sin(theta / 2) ~= theta / 2, so 2 * imaginary / dt approximates angular velocity.
                    Quat deltaRotation = q * q0.GetConjugate();
                    Vec3 deltaAngularVelocity = Abs(q0.RotateInv(deltaRotation.GetImaginaryPart())) * (2.0f * step.inv_dt);

                    // Bound the angular velocity at the farthest point of the body.
                    Vec3 pointVelocity = AbsCross(deltaAngularVelocity, body->halfExtent);
                    float sleepVelocity = Length(deltaLinearVelocity) + Length(pointVelocity);

                    if (sleepVelocity > settings.sleep_velocity_threshold)
                    {
                        if (GetBit(&activeIslandBits, workerIndex, body->islandIndex) == false)
                        {
                            SetBit(&activeIslandBits, workerIndex, body->islandIndex);
                        }
                    }

                    Transform transform0;
                    s->motion.GetTransform(0.0f, &transform0);
                    body->SynchronizeTransform();

                    if (settings.world_bounds.TestPoint(body->transform.p) == false)
                    {
                        SetBit(&destroyBodyBits, workerIndex, i);
                        continue;
                    }

                    int32 syncIndex = colliderStarts[i];
                    for (Collider* collider : body->colliders)
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
        MergeBitset(&activeIslandBits);
        MergeBitset(&destroyBodyBits);

        // The broad phase tree and move buffer are not thread-safe, so commit serially.
        MuliProfileZoneNR(sync_colliders, "Sync Colliders", true);
        for (int32 i = 0; i < islandCount; ++i)
        {
            Island* island = islands + i;
            bool activeIsland = GetBit(&activeIslandBits, 0, i);
            bool sleeping = settings.sleeping && (activeIsland == false);

            for (int32 j = 0; j < island->bodyCount; ++j)
            {
                int32 b = island->bodyStart + j;
                BodyState* s = islandBodies[b];
                Body* body = s->body;

                if (GetBit(&destroyBodyBits, 0, b))
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

                if (activeIsland)
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

        FreeBitset(&destroyBodyBits, &linearAllocator);
        FreeBitset(&activeIslandBits, &linearAllocator);
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

            ContactState state = constraintGraph.RemoveContactFromGraph(contact);

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

            JointState state = constraintGraph.RemoveJointFromGraph(joint);

            SolverSet& target = solverSets[targetSet];
            joint->setIndex = targetSet;
            joint->localIndex = int32(target.jointStates.size());
            target.jointStates.push_back(std::move(state));
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

    // Transfer bodies last so contact body lane indices remain valid for the whole solve.
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

    linearAllocator.Free(islandJoints, totalJointCount * sizeof(Joint*));
    linearAllocator.Free(islandContacts, totalContactCount * sizeof(Contact*));
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
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float linearFrequency,
    float linearDampingRatio,
    float swingFrequency,
    float swingDampingRatio,
    float angleFrequency,
    float angleDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    RevoluteJoint* rj = poolAllocator.New<RevoluteJoint>(
        bodyA, bodyB, anchor, axis, minAngle, maxAngle, linearFrequency, linearDampingRatio, swingFrequency, swingDampingRatio,
        angleFrequency, angleDampingRatio
    );

    AddJoint(rj);
    return rj;
}

RevoluteAngleJoint* World::CreateRevoluteAngleJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& axis,
    float minAngle,
    float maxAngle,
    float swingFrequency,
    float swingDampingRatio,
    float angleFrequency,
    float angleDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    RevoluteAngleJoint* raj = poolAllocator.New<RevoluteAngleJoint>(
        bodyA, bodyB, axis, minAngle, maxAngle, swingFrequency, swingDampingRatio, angleFrequency, angleDampingRatio
    );

    AddJoint(raj);
    return raj;
}

UniversalAngleJoint* World::CreateUniversalAngleJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& axisA,
    const Vec3& axisB,
    float perpendicularFrequency,
    float perpendicularDampingRatio,
    float steeringFrequency,
    float steeringDampingRatio,
    float spinFrequency,
    float spinDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    UniversalAngleJoint* uaj = poolAllocator.New<UniversalAngleJoint>(
        bodyA, bodyB, axisA, axisB, perpendicularFrequency, perpendicularDampingRatio, steeringFrequency, steeringDampingRatio,
        spinFrequency, spinDampingRatio
    );

    AddJoint(uaj);
    return uaj;
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

WeldJoint* World::CreateWeldJoint(
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    float linearFrequency,
    float linearDampingRatio,
    float angularFrequency,
    float angularDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    WeldJoint* wj = poolAllocator.New<WeldJoint>(
        bodyA, bodyB, anchor, linearFrequency, linearDampingRatio, angularFrequency, angularDampingRatio
    );

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& direction, float frequency, float dampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    LineJoint* lj = poolAllocator.New<LineJoint>(bodyA, bodyB, anchor, direction, frequency, dampingRatio);

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
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    const Vec3& direction,
    float linearFrequency,
    float linearDampingRatio,
    float angularFrequency,
    float angularDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    PrismaticJoint* pj = poolAllocator.New<PrismaticJoint>(
        bodyA, bodyB, anchor, direction, linearFrequency, linearDampingRatio, angularFrequency, angularDampingRatio
    );

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    Body* bodyA, Body* bodyB, float linearFrequency, float linearDampingRatio, float angularFrequency, float angularDampingRatio
)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), linearFrequency,
        linearDampingRatio, angularFrequency, angularDampingRatio
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
    Body* bodyA,
    Body* bodyB,
    const Vec3& anchor,
    float maxForce,
    float maxTorque,
    float linearFrequency,
    float linearDampingRatio,
    float angularFrequency,
    float angularDampingRatio
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    MotorJoint* mj = poolAllocator.New<MotorJoint>(
        bodyA, bodyB, anchor, maxForce, maxTorque, linearFrequency, linearDampingRatio, angularFrequency, angularDampingRatio
    );

    AddJoint(mj);
    return mj;
}

void World::AddBody(Body* body)
{
    body->world = this;
    body->worldIndex = int32(bodies.size());
    body->colliders.clear();
    body->contacts.clear();
    body->joints.clear();
    body->flag &= ~Body::flag_island;
    body->flag |= Body::flag_enabled;

    bodies.push_back(body);

    SolverSetIndex setIndex = body->type == Body::static_body ? static_set : awake_set;
    AddBodyState(body, setIndex);
}

void World::FreeBody(Body* body)
{
    poolAllocator.Delete(body);
}

void World::AddJoint(Joint* joint)
{
    // Insert into the world.
    joint->worldIndex = int32(joints.size());
    joints.push_back(joint);

    // Connect to island graph

    // Connect joint to body A.
    joint->bodyIndexA = int32(joint->bodyA->joints.size());
    joint->bodyA->joints.push_back(joint);

    // Connect joint to body B.
    if (joint->bodyA != joint->bodyB)
    {
        joint->bodyIndexB = int32(joint->bodyB->joints.size());
        joint->bodyB->joints.push_back(joint);
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
        constraintGraph.AddJointToGraph(joint, std::move(state));
    }
    else
    {
        SolverSet& set = solverSets[setIndex];
        joint->localIndex = int32(set.jointStates.size());

        set.jointStates.push_back(std::move(state));
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
    case Joint::Type::universal_angle_joint:
        poolAllocator.Delete((UniversalAngleJoint*)joint);
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
    case Shape::mesh:
    {
        return poolAllocator.New<MeshShape>(*(const MeshShape*)shape, transform);
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
    case Shape::mesh:
        poolAllocator.Delete((MeshShape*)shape);
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
    state.gravityScale = 1.0f;
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
        Body* movedBody = set.bodyStates[index].body;
        movedBody->localIndex = index;
        UpdateContactBodyIndices(movedBody);
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

    BodyState state = source.bodyStates[sourceIndex];

    int32 last = int32(source.bodyStates.size() - 1);
    if (sourceIndex != last)
    {
        source.bodyStates[sourceIndex] = source.bodyStates[last];
        Body* movedBody = source.bodyStates[sourceIndex].body;
        movedBody->localIndex = sourceIndex;
        UpdateContactBodyIndices(movedBody);
    }
    source.bodyStates.pop_back();

    target.bodyStates.push_back(state);

    body->setIndex = targetSet;
    body->localIndex = targetIndex;
    UpdateContactBodyIndices(body);
}

void World::UpdateContactBodyIndices(Body* body)
{
    for (Contact* contact : body->contacts)
    {
        if (contact->IsSimpleContact() == false)
        {
            continue;
        }

        BlockContactState& state = constraintGraph.batches[contact->colorIndex].blockContacts.state;
        int32 block = contact->localIndex / simd_width;
        int32 lane = contact->localIndex % simd_width;
        if (contact->GetBodyA() == body)
        {
            state.bodySetA[block].lane[lane] = body->setIndex;
            state.bodyIndexA[block].lane[lane] = body->localIndex;
        }
        else
        {
            state.bodySetB[block].lane[lane] = body->setIndex;
            state.bodyIndexB[block].lane[lane] = body->localIndex;
        }
    }
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

        ContactState state = constraintGraph.RemoveContactFromGraph(contact);

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

        JointState state = constraintGraph.RemoveJointFromGraph(joint);

        SolverSet& target = solverSets[targetSet];
        joint->setIndex = targetSet;
        joint->localIndex = int32(target.jointStates.size());
        target.jointStates.push_back(std::move(state));
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
        constraintGraph.AddJointToGraph(joint, std::move(state));
    }
    else
    {
        joint->setIndex = targetSet;
        joint->localIndex = int32(target.jointStates.size());
        target.jointStates.push_back(std::move(state));
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

    for (Contact* contact : body->contacts)
    {
        Body* other = contact->GetBodyA() == body ? contact->GetBodyB() : contact->GetBodyA();

        if (other->IsEnabled() == false)
        {
            continue;
        }

        TransferContact(contact, awake_set);
    }

    for (Joint* joint : body->joints)
    {
        Body* other = joint->bodyA == body ? joint->bodyB : joint->bodyA;

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

        for (Contact* contact : b->contacts)
        {
            Body* other = contact->GetBodyA() == b ? contact->GetBodyB() : contact->GetBodyA();

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

        for (Joint* joint : b->joints)
        {
            Body* other = joint->bodyA == b ? joint->bodyB : joint->bodyA;

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
    GrowableStack<Body*, 64> island;

    stack.push_back(body);
    body->flag |= Body::flag_island;

    while (stack.size() > 0)
    {
        Body* b = stack.back();
        stack.pop_back();
        island.push_back(b);

        for (Contact* contact : b->contacts)
        {
            Body* other = contact->GetBodyA() == b ? contact->GetBodyB() : contact->GetBodyA();

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

        for (Joint* joint : b->joints)
        {
            Body* other = joint->bodyA == b ? joint->bodyB : joint->bodyA;

            if (other->IsStatic() || other->IsEnabled() == false || (other->flag & Body::flag_island))
            {
                continue;
            }

            other->flag |= Body::flag_island;
            stack.push_back(other);
        }
    }

    for (Body* b : island)
    {
        BodyState* state = b->GetBodyState();
        state->resting = max_float;
        state->force = Vec3::zero;
        state->torque = Vec3::zero;
        state->linearVelocity = Vec3::zero;
        state->angularVelocity = Vec3::zero;

        b->flag |= Body::flag_sleeping;
    }

    for (Body* b : island)
    {
        for (Contact* contact : b->contacts)
        {
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

        for (Joint* joint : b->joints)
        {
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

    for (Body* b : island)
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

        for (int32 i = 0; i < batch.blockContacts.Count(); ++i)
        {
            Contact* contact = batch.blockContacts.GetContact(i);
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
            MuliAssert(colorIndex != constraint_overflow_index);

            int32 block = i / simd_width;
            int32 lane = i % simd_width;
            const BlockContactState& state = batch.blockContacts.state;
            MuliAssert(state.bodySetA[block].lane[lane] == bodyA->setIndex);
            MuliAssert(state.bodyIndexA[block].lane[lane] == bodyA->localIndex);
            MuliAssert(state.bodySetB[block].lane[lane] == bodyB->setIndex);
            MuliAssert(state.bodyIndexB[block].lane[lane] == bodyB->localIndex);
            MuliNotUsed(state);
            MuliNotUsed(block);
            MuliNotUsed(lane);

            if (bodyA->IsDynamic())
            {
                MuliAssert(bodyA->IsSleeping() == false);
                MuliAssert((bodyA->usedColors & colorBit) != 0);
                MuliAssert(colorBodies.insert(bodyA).second);
            }
            if (bodyA != bodyB && bodyB->IsDynamic())
            {
                MuliAssert(bodyB->IsSleeping() == false);
                MuliAssert((bodyB->usedColors & colorBit) != 0);
                MuliAssert(colorBodies.insert(bodyB).second);
            }
        }

        MuliAssert(batch.scalarContacts.states.size() == batch.scalarContacts.constraints.size());
        for (int32 i = 0; i < batch.scalarContacts.Count(); ++i)
        {
            Contact* contact = batch.scalarContacts.GetContact(i);
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
                if (bodyA->IsDynamic())
                {
                    MuliAssert(bodyA->IsSleeping() == false);
                    MuliAssert((bodyA->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyA).second);
                }
                if (bodyA != bodyB && bodyB->IsDynamic())
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

        for (int32 i = 0; i < batch.scalarJoints.Count(); ++i)
        {
            Joint* joint = batch.scalarJoints.GetJoint(i);
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
                if (bodyA->IsDynamic())
                {
                    MuliAssert(bodyA->IsSleeping() == false);
                    MuliAssert((bodyA->usedColors & colorBit) != 0);
                    MuliAssert(colorBodies.insert(bodyA).second);
                }
                if (bodyA != bodyB && bodyB->IsDynamic())
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
    for (Contact* contact : constraintGraph.contacts)
    {
        MuliAssert(seenContacts.contains(contact));
        MuliNotUsed(contact);
        ++numContacts;
    }
    MuliAssert(numContacts == int32(seenContacts.size()));
    MuliNotUsed(numContacts);

    int32 numJoints = 0;
    for (Joint* joint : joints)
    {
        MuliAssert(seenJoints.contains(joint));
        MuliNotUsed(joint);
        ++numJoints;
    }
    MuliAssert(numJoints == int32(seenJoints.size()));
    MuliAssert(numJoints == int32(joints.size()));
    MuliNotUsed(numJoints);

    for (Body* body : bodies)
    {
        if (body->IsDynamic() == false)
        {
            continue;
        }

        uint32 usedColors = 0;
        for (int32 colorIndex = 0; colorIndex < constraint_overflow_index; ++colorIndex)
        {
            const ConstraintBatch& batch = constraintGraph.batches[colorIndex];
            uint32 colorBit = 1u << colorIndex;

            for (int32 i = 0; i < batch.blockContacts.Count(); ++i)
            {
                Contact* contact = batch.blockContacts.GetContact(i);
                if (contact->GetBodyA() == body || contact->GetBodyB() == body)
                {
                    usedColors |= colorBit;
                }
            }

            for (const ContactState& state : batch.scalarContacts.states)
            {
                Contact* contact = state.contact;
                if (contact->GetBodyA() == body || contact->GetBodyB() == body)
                {
                    usedColors |= colorBit;
                }
            }

            for (const JointState& state : batch.scalarJoints.states)
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
