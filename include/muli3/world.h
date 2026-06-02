#pragma once

#include "callbacks.h"
#include "constraint_graph.h"
#include "contact.h"
#include "joints.h"
#include "profile.h"
#include "rigidbody.h"
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

    void BufferDestroy(RigidBody* body);
    void BufferDestroy(std::span<RigidBody*> bodies);
    void BufferDestroy(Joint* joint);
    void BufferDestroy(std::span<Joint*> joints);

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
    RigidBody* CreateCapsule(
        const Vec3& point1,
        const Vec3& point2,
        float radius,
        const Transform& tf = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        bool resetPosition = false,
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
    RigidBody* CreateConvex(
        std::span<const Vec3> vertices,
        const Transform& transform = identity,
        RigidBody::Type type = RigidBody::dynamic_body,
        float radius = default_radius,
        float density = default_density
    );

    GrabJoint* CreateGrabJoint(
        RigidBody* body, const Vec3& anchor, const Vec3& target, float frequency = 10.0f, float dampingRatio = 1.0f
    );
    FixedRotationJoint* CreateFixedRotationJoint(RigidBody* body, float frequency = -1.0f, float dampingRatio = 1.0f);
    ConeSwingJoint* CreateConeSwingJoint(
        RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float maxAngle, float frequency = -1.0f, float dampingRatio = 1.0f
    );
    RevoluteJoint* CreateRevoluteJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& axis,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    RevoluteJoint* CreateLimitedRevoluteJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    RevoluteAngleJoint* CreateRevoluteAngleJoint(
        RigidBody* bodyA, RigidBody* bodyB, const Vec3& axis, float frequency = 10.0f, float dampingRatio = 1.0f
    );
    RevoluteAngleJoint* CreateLimitedRevoluteAngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    TwistAngleJoint* CreateTwistAngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    BallSocketJoint* CreateBallSocketJoint(
        RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency = 10.0f, float dampingRatio = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float length = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    DistanceJoint* CreateDistanceJoint(
        RigidBody* bodyA, RigidBody* bodyB, float length = -1.0f, float frequency = 10.0f, float dampingRatio = 1.0f
    );
    DistanceJoint* CreateLimitedDistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float minLength = -1.0f,
        float maxLength = -1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    WeldJoint* CreateWeldJoint(
        RigidBody* bodyA, RigidBody* bodyB, const Vec3& anchor, float frequency = -1.0f, float dampingRatio = 1.0f
    );
    LineJoint* CreateLineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    LineJoint* CreateLineJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency = 10.0f, float dampingRatio = 1.0f);
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    PrismaticJoint* CreatePrismaticJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency = -1.0f, float dampingRatio = 1.0f);
    PulleyJoint* CreatePulleyJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio = 1.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );
    MotorJoint* CreateMotorJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& anchor,
        float maxForce = 1000.0f,
        float maxTorque = 1000.0f,
        float frequency = 10.0f,
        float dampingRatio = 1.0f
    );

    void Query(const Vec3& point, WorldQueryCallback* callback) const;
    void Query(const AABB& aabb, WorldQueryCallback* callback) const;
    void RayCastAny(const Vec3& from, const Vec3& to, float radius, RayCastAnyCallback* callback) const;
    bool RayCastClosest(const Vec3& from, const Vec3& to, float radius, RayCastClosestCallback* callback) const;
    void ShapeCastAny(const Shape* shape, const Transform& tf, const Vec3& translation, ShapeCastAnyCallback* callback) const;
    bool ShapeCastClosest(
        const Shape* shape, const Transform& tf, const Vec3& translation, ShapeCastClosestCallback* callback
    ) const;

    void Query(const Vec3& point, std::function<bool(Collider* collider)> callback) const;
    void Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const;
    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    void ShapeCastAny(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
    ) const;
    bool ShapeCastClosest(
        const Shape* shape,
        const Transform& tf,
        const Vec3& translation,
        std::function<void(Collider* collider, Vec3 point, Vec3 normal, float t)> callback
    ) const;

    RigidBody* GetBodyList() const;
    int32 GetBodyCount() const;

    Joint* GetJoints() const;
    int32 GetJointCount() const;

    const Contact* GetContacts() const;
    int32 GetContactCount() const;

    int32 GetSleepingBodyCount() const;
    int32 GetAwakeIslandCount() const;

    const AABBTree& GetDynamicTree() const;

    const WorldSettings& GetSettings() const;
    const WorldProfile& GetProfile() const;

    void Awake();

private:
    friend class RigidBody;
    friend class Collider;
    friend class Contact;
    friend class Joint;
    friend class Island;
    friend class ConstraintGraph;
    friend class BroadPhase;

    void Solve();

    void AddBody(RigidBody* body);
    void FreeBody(RigidBody* body);

    void AddJoint(Joint* joint);
    void FreeJoint(Joint* joint);

    Shape* CloneShape(const Shape* shape, const Transform& transform = identity);
    void FreeShape(Shape* shape);

    BodyState* AddBodyState(RigidBody* body, SolverSetIndex setIndex);
    void RemoveBodyState(RigidBody* body);
    void TransferBody(RigidBody* body, SolverSetIndex targetSet);

    ContactState* AddContactState(Contact* contact, SolverSetIndex setIndex);
    void RemoveContactState(Contact* contact);
    void TransferContact(Contact* contact, SolverSetIndex targetSet);

    JointState* AddJointState(Joint* joint, SolverSetIndex setIndex);
    void RemoveJointState(Joint* joint);
    void TransferJoint(Joint* joint, SolverSetIndex targetSet);

    void WakeBody(RigidBody* body);
    void SleepBody(RigidBody* body);

    void WakeIsland(RigidBody* body);
    void SleepIsland(RigidBody* body);

    void Validate() const;

    const WorldSettings& settings;
    WorldProfile profile;

    RigidBody* bodyList = nullptr;
    RigidBody* bodyListTail = nullptr;
    int32 bodyCount = 0;

    Joint* jointList = nullptr;
    Joint* jointListTail = nullptr;
    int32 jointCount = 0;

    ConstraintGraph constraintGraph;

    SolverSet solverSets[solver_set_count];

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
    return constraintGraph.contactList;
}

inline int32 World::GetContactCount() const
{
    return constraintGraph.contactCount;
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

inline void World::Awake()
{
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        b->Awake();
    }
}

} // namespace muli3
