#pragma once

#include "allocators.h"
#include "body.h"
#include "callbacks.h"
#include "constraint_graph.h"
#include "contact.h"
#include "joints.h"
#include "profile.h"
#include "settings.h"

namespace muli3
{

class World
{
public:
    World(const WorldSettings* settings);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    float Step(float dt);
    void Reset();

    void Destroy(Body* body);
    void Destroy(std::span<Body*> bodies);
    void Destroy(Joint* joint);
    void Destroy(std::span<Joint*> joints);

    // Buffered bodies are destroyed at the end of the step.
    // Useful when called during a world step.
    void BufferDestroy(Body* body);
    void BufferDestroy(std::span<Body*> bodies);
    void BufferDestroy(Joint* joint);
    void BufferDestroy(std::span<Joint*> joints);

    // clang-format off

    // Body factory functions
    Body* CreateEmptyBody(
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body
    );
    Body* CreateSphere(
        float radius,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float density = default_density
    );
    Body* CreateCapsule(
        float height,
        float radius,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float density = default_density
    );
    Body* CreateCapsule(
        const Vec3& point1,
        const Vec3& point2,
        float radius,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        bool resetPosition = false,
        float density = default_density
    );
    Body* CreateBox(
        float width,
        float height,
        float depth,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateBox(
        const Vec3& size,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateBox(
        float size,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateConvex(
        std::span<const Vec3> vertices,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateCylinder(
        float height,
        float topRadius,
        float bottomRadius,
        uint16 segmentCount = 16,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float convexRadius = default_radius,
        float density = default_density
    );
    Body* CreateTriangle(
        const Vec3& a,
        const Vec3& b,
        const Vec3& c,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateTriangle(
        const Vec3 vertices[3],
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateQuad(
        float width,
        float height,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreatePolygon(
        std::span<const Vec3> vertices,
        const Transform& transform = identity,
        Body::Type type = Body::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    Body* CreateHeightField(
        int32 sampleCountX,
        int32 sampleCountZ,
        std::span<const float> heightSamples,
        float cellSizeX = 1.0f,
        float cellSizeZ = 1.0f,
        const Transform& transform = identity,
        const Vec3& offset = Vec3{ 0.0f },
        int32 blockSize = 4
    );
    Body* CreateMesh(
        std::span<const Vec3> vertices,
        std::span<const int32> indices,
        const Transform& transform = identity
    );

    // Joint factory functions
    GrabJoint* CreateGrabJoint(
        Body* body,
        const Vec3& anchor,
        const Vec3& target,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    FixedRotationJoint* CreateFixedRotationJoint(
        Body* body,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    ConeSwingJoint* CreateConeSwingJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axis,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    RevoluteJoint* CreateRevoluteJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    RevoluteAngleJoint* CreateRevoluteAngleJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    UniversalAngleJoint* CreateUniversalAngleJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axisA,
        const Vec3& axisB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    TwistAngleJoint* CreateTwistAngleJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    BallSocketJoint* CreateBallSocketJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float minLength = -1.0f,
        float maxLength = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    WeldJoint* CreateWeldJoint(
        Body* bodyA, Body* bodyB,
        const Vec3& anchor,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    LineJoint* CreateLineJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& direction,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    LineJoint* CreateLineJoint(
        Body* bodyA,
        Body* bodyB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& direction,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(Body* bodyA,
        Body* bodyB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    PulleyJoint* CreatePulleyJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio = 1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    MotorJoint* CreateMotorJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        float maxForce = 1000.0f,
        float maxTorque = 1000.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );

    void Query(
        const Vec3& point,
        WorldQueryCallback* callback
    ) const;
    void Query(
        const AABB& aabb,
        WorldQueryCallback* callback
    ) const;
    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        RayCastAnyCallback* callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        RayCastClosestCallback* callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& transform,
        const Vec3& translation,
        ShapeCastAnyCallback* callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& transform,
        const Vec3& translation,
        ShapeCastClosestCallback* callback
    ) const;

    void Query(
        const Vec3& point,
        std::function<bool(Collider* collider)> callback
    ) const;
    void Query(
        const AABB& aabb,
        std::function<bool(Collider* collider)> callback
    ) const;
    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& transform,
        const Vec3& translation,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& transform,
        const Vec3& translation,
        std::function<void(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
    ) const;
    // clang-format on

    std::span<Body* const> GetBodies() const;
    int32 GetBodyCount() const;

    std::span<Joint* const> GetJoints() const;
    int32 GetJointCount() const;

    std::span<Contact* const> GetContacts() const;
    int32 GetContactCount() const;

    int32 GetSleepingBodyCount() const;
    int32 GetAwakeIslandCount() const;
    int32 GetConstraintCount(int32 colorIndex) const;

    const AABBTree& GetDynamicTree() const;

    const WorldSettings& GetSettings() const;
    const WorldProfile& GetProfile() const;

    uint64 GetStepIndex() const;

    void Awake();

private:
    friend class Body;
    friend class Collider;
    friend class BroadPhase;
    friend class ConstraintGraph;
    friend class Contact;
    friend class Joint;

    void Solve();

    void AddBody(Body* body);
    void FreeBody(Body* body);

    void AddJoint(Joint* joint);
    void FreeJoint(Joint* joint);

    Shape* CloneShape(const Shape* shape, const Transform& transform = identity);
    void FreeShape(Shape* shape);

    BodyState* AddBodyState(Body* body, SolverSetIndex setIndex);
    void RemoveBodyState(Body* body);
    void TransferBody(Body* body, SolverSetIndex targetSet);
    void UpdateContactBodyIndices(Body* body);

    ContactState* AddContactState(Contact* contact, SolverSetIndex setIndex);
    void RemoveContactState(Contact* contact);
    void TransferContact(Contact* contact, SolverSetIndex targetSet);

    JointState* AddJointState(Joint* joint, SolverSetIndex setIndex);
    void RemoveJointState(Joint* joint);
    void TransferJoint(Joint* joint, SolverSetIndex targetSet);

    void WakeBody(Body* body);
    void SleepBody(Body* body);

    void WakeIsland(Body* body);
    void SleepIsland(Body* body);

    void Validate() const;

    const WorldSettings& settings;

    WorldProfile profile;
    Timestep step;

    std::vector<Body*> bodies;
    std::vector<Joint*> joints;

    ConstraintGraph constraintGraph;
    std::vector<SolverSet> solverSets;

    int32 islandCount = 0;
    int32 sleepingBodyCount = 0;

    std::vector<Body*> destroyBodyBuffer;
    std::vector<Joint*> destroyJointBuffer;

    LinearAllocator linearAllocator;
    PoolAllocator poolAllocator;
};

inline std::span<Body* const> World::GetBodies() const
{
    return bodies;
}

inline int32 World::GetBodyCount() const
{
    return int32(bodies.size());
}

inline std::span<Joint* const> World::GetJoints() const
{
    return joints;
}

inline int32 World::GetJointCount() const
{
    return int32(joints.size());
}

inline std::span<Contact* const> World::GetContacts() const
{
    return constraintGraph.contacts;
}

inline int32 World::GetContactCount() const
{
    return constraintGraph.GetContactCount();
}

inline int32 World::GetSleepingBodyCount() const
{
    return sleepingBodyCount;
}

inline int32 World::GetAwakeIslandCount() const
{
    return islandCount;
}

inline int32 World::GetConstraintCount(int32 colorIndex) const
{
    MuliAssert(0 <= colorIndex && colorIndex < constraint_color_count);
    const ConstraintBatch& batch = constraintGraph.batches[colorIndex];
    return batch.blockContacts.Count() + batch.scalarContacts.Count() + batch.scalarJoints.Count();
}

inline const AABBTree& World::GetDynamicTree() const
{
    return constraintGraph.broadPhase.tree;
}

inline const WorldSettings& World::GetSettings() const
{
    return settings;
}

inline const WorldProfile& World::GetProfile() const
{
    return profile;
}

inline uint64 World::GetStepIndex() const
{
    return step.index;
}

inline void World::Awake()
{
    for (Body* b : bodies)
    {
        b->Awake();
    }
}

} // namespace muli3
