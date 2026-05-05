#pragma once

#include "block_allocator.h"
#include "contact.h"
#include "contact_graph.h"
#include "linear_allocator.h"
#include "settings.h"

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

    void BufferDestroy(RigidBody* body);
    void BufferDestroy(std::span<RigidBody*> bodies);

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

    RigidBody* GetBodyList() const;
    RigidBody* GetBodyListTail() const;
    int32 GetBodyCount() const;

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
    Shape* CloneShape(const Shape* shape, const Transform& transform = identity);
    void FreeShape(Shape* shape);

    const WorldSettings& settings;
    ContactGraph contactGraph;

    std::vector<RigidBody*> destroyBodyBuffer;

    RigidBody* bodyList = nullptr;
    RigidBody* bodyListTail = nullptr;
    int32 bodyCount = 0;

    int32 islandCount = 0;
    int32 sleepingBodyCount = 0;

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
