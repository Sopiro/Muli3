#pragma once

#include "aabb_tree.h"
#include "shape.h"

namespace muli3
{
class ContactGraph;

class BroadPhase
{
public:
    BroadPhase(ContactGraph* contactGraph);
    ~BroadPhase();

    void FindNewContacts();
    bool TestOverlap(RigidBody* bodyA, RigidBody* bodyB) const;

    void Add(RigidBody* body, const AABB& aabb);
    void Remove(RigidBody* body);
    void Update(RigidBody* body, const AABB& aabb, const Vec3& displacement);
    void Refresh(RigidBody* body);

    bool QueryCallback(NodeIndex node, RigidBody* body);

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
    RigidBody* bodyA;
    ShapeType typeA;

    void BufferMove(NodeIndex node);
    void UnBufferMove(NodeIndex node);
};

inline bool BroadPhase::TestOverlap(RigidBody* inBodyA, RigidBody* inBodyB) const
{
    return tree.TestOverlap(inBodyA->node, inBodyB->node);
}

} // namespace muli3
