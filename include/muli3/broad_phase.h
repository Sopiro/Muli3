#pragma once

#include "aabb_tree.h"

namespace muli3
{
class ContactGraph;

class BroadPhase
{
public:
    BroadPhase(ContactGraph* contactGraph);
    ~BroadPhase();

    void FindNewContacts();
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider, const AABB& aabb);
    void Remove(Collider* collider);
    void Update(Collider* collider, const AABB& aabb, const Vec3& displacement);
    void Refresh(Collider* collider);

    bool QueryCallback(NodeIndex node, Collider* collider);

protected:
    friend class World;
    friend class ContactGraph;

    ContactGraph* contactGraph;
    AABBTree tree;

private:
    NodeIndex* moveBuffer;
    int32 moveCapacity;
    int32 moveCount;

    NodeIndex nodeA;
    Collider* colliderA;
    RigidBody* bodyA;
    Shape::Type typeA;

    void BufferMove(NodeIndex node);
    void UnBufferMove(NodeIndex node);
};

inline bool BroadPhase::TestOverlap(Collider* inColliderA, Collider* inColliderB) const
{
    return tree.TestOverlap(inColliderA->node, inColliderB->node);
}

} // namespace muli3
