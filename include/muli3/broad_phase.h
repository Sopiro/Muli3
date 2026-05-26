#pragma once

#include "aabb_tree.h"

namespace muli3
{

class ContactGraph;

class BroadPhase
{
public:
    BroadPhase();
    ~BroadPhase();

    void FindNewContacts(ContactGraph* contactGraph);
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider, const AABB& aabb);
    void Remove(Collider* collider);
    void Update(Collider* collider, const AABB& aabb, const Vec3& displacement, bool reset);
    void Refresh(Collider* collider);

private:
    friend class World;
    friend class ContactGraph;

    AABBTree tree;

    NodeIndex* moveBuffer;
    int32 moveCapacity;
    int32 moveCount;

    void BufferMove(NodeIndex node);
    void UnBufferMove(NodeIndex node);

    struct TreeCallback;
};

inline bool BroadPhase::TestOverlap(Collider* inColliderA, Collider* inColliderB) const
{
    return tree.TestOverlap(inColliderA->node, inColliderB->node);
}

} // namespace muli3
