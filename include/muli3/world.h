#pragma once

#include "rigidbody.h"

#include "ball_socket_joint.h"
#include "distance_joint.h"
#include "grab_joint.h"
#include "joint.h"
#include "line_joint.h"
#include "motor_joint.h"
#include "prismatic_joint.h"
#include "pulley_joint.h"
#include "weld_joint.h"

#include "callbacks.h"
#include "contact.h"
#include "contact_graph.h"
#include "settings.h"

#include "block_allocator.h"
#include "linear_allocator.h"

namespace muli3
{

class World
{
public:
    World(const WorldSettings& settings);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    float Step(float dt);
    void Reset();

    void Destroy(RigidBody* body);
    void Destroy(std::span<RigidBody*> bodies);
    void Destroy(Joint* joint);
    void Destroy(std::span<Joint*> joints);

    // Buffered body will be destroy at the end of the step
    void BufferDestroy(RigidBody* body);
    void BufferDestroy(std::span<RigidBody*> bodies);
    void BufferDestroy(Joint* joint);
    void BufferDestroy(std::span<Joint*> joints);

    // clang-format off
    // Factory functions for bodies
    RigidBody* CreateEmptyBody(const Transform& transform = identity, RigidBody::Type type = RigidBody::dynamic_body);
    RigidBody* CreateSphere(
        float radius,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float density = default_density
    );
    RigidBody* CreateCapsule(
        float height,
        float radius,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float density = default_density
    );
    RigidBody* CreateBox(
        float width,
        float height,
        float depth,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    RigidBody* CreateBox(
        const Vec3& size,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    RigidBody* CreateBox(
        float size,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );
    // clang-format on

    // clang-format off
    // Factory functions for joints
    GrabJoint* CreateGrabJoint(
        RigidBody* body,
        const Vec3& anchor,
        const Vec3& target,
        float frequency = 1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    BallSocketJoint* CreateBallSocketJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float length = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float length = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    DistanceJoint* CreateLimitedDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float minLength = -1.0f,
        float maxLength = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    WeldJoint* CreateWeldJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    LineJoint* CreateLineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    LineJoint* CreateLineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    PulleyJoint* CreatePulleyJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio = 1.0f,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    MotorJoint* CreateMotorJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        float maxForce = 1000.0f,
        float maxTorque = 1000.0f,
        float frequency = -1.0f,
        float dampingRatio = 1.0f,
        float jointMass = 1.0f
    );
    // clang-format on

    // clang-format off
    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        float radius,
        RayCastAnyCallback* callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        float radius,
        RayCastClosestCallback* callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        ShapeCastAnyCallback* callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        ShapeCastClosestCallback* callback
    ) const;

    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<float(RigidBody* body, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<void(RigidBody* body, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        std::function<float(RigidBody* body, Vec3 point, Vec3 normal, float t)> callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        std::function<void(RigidBody* body, Vec3 point, Vec3 normal, float t)> callback
    ) const;
    // clang-format on

    RigidBody* GetBodyList() const;
    RigidBody* GetBodyListTail() const;
    int32 GetBodyCount() const;

    Joint* GetJoints() const;
    int32 GetJointCount() const;

    const Contact* GetContacts() const;
    int32 GetContactCount() const;

    int32 GetSleepingBodyCount() const;
    int32 GetAwakeIslandCount() const;

    const AABBTree& GetDynamicTree() const;
    void RebuildDynamicTree();

    const WorldSettings& GetWorldSettings() const;

    void Awake();

private:
    friend class RigidBody;
    friend class Island;
    friend class ContactGraph;
    friend class BroadPhase;

    void Solve();
    void FreeBody(RigidBody* body);
    void AddJoint(Joint* joint);
    void FreeJoint(Joint* joint);
    Shape* CloneShape(const Shape* shape, const Transform& transform = identity);
    void FreeShape(Shape* shape);

    const WorldSettings& settings;
    ContactGraph contactGraph;

    RigidBody* bodyList = nullptr;
    RigidBody* bodyListTail = nullptr;
    int32 bodyCount = 0;

    Joint* jointList = nullptr;
    int32 jointCount = 0;

    int32 islandCount = 0;
    int32 sleepingBodyCount = 0;

    std::vector<RigidBody*> destroyBodyBuffer;
    std::vector<Joint*> destroyJointBuffer;

    LinearAllocator linearAllocator;
    BlockAllocator blockAllocator;
};

inline RigidBody* World::GetBodyList() const
{
    return bodyList;
}

inline RigidBody* World::GetBodyListTail() const
{
    return bodyListTail;
}

inline int32 World::GetBodyCount() const
{
    return bodyCount;
}

inline Joint* World::GetJoints() const
{
    return jointList;
}

inline int32 World::GetJointCount() const
{
    return jointCount;
}

inline const Contact* World::GetContacts() const
{
    return contactGraph.contactList;
}

inline int32 World::GetContactCount() const
{
    return contactGraph.contactCount;
}

inline int32 World::GetSleepingBodyCount() const
{
    return sleepingBodyCount;
}

inline int32 World::GetAwakeIslandCount() const
{
    return islandCount;
}

inline const AABBTree& World::GetDynamicTree() const
{
    return contactGraph.broadPhase.tree;
}

inline void World::RebuildDynamicTree()
{
    contactGraph.broadPhase.tree.Rebuild();
}

inline const WorldSettings& World::GetWorldSettings() const
{
    return settings;
}

inline void World::Awake()
{
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        b->Awake();
    }
}

} // namespace muli3
