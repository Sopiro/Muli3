#include "muli3/broad_phase.h"
#include "muli3/contact_graph.h"
#include "muli3/world.h"

namespace muli3
{

BroadPhase::BroadPhase(ContactGraph* contactGraph)
    : contactGraph{ contactGraph }
    , moveCapacity{ 16 }
    , moveCount{ 0 }
{
    moveBuffer = (NodeIndex*)muli3::Alloc(moveCapacity * sizeof(NodeIndex));
}

BroadPhase::~BroadPhase()
{
    muli3::Free(moveBuffer);
}

void BroadPhase::BufferMove(NodeIndex node)
{
    if (moveCount == moveCapacity)
    {
        NodeIndex* old = moveBuffer;
        moveCapacity *= 2;
        moveBuffer = (NodeIndex*)muli3::Alloc(moveCapacity * sizeof(NodeIndex));
        memcpy(moveBuffer, old, moveCount * sizeof(NodeIndex));
        muli3::Free(old);
    }

    moveBuffer[moveCount] = node;
    ++moveCount;
}

void BroadPhase::UnBufferMove(NodeIndex node)
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        if (moveBuffer[i] == node)
        {
            moveBuffer[i] = AABBTree::nullNode;
        }
    }
}

void BroadPhase::FindNewContacts()
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        nodeA = moveBuffer[i];
        if (nodeA == AABBTree::nullNode)
        {
            continue;
        }

        colliderA = tree.GetData(nodeA);
        bodyA = colliderA->body;
        typeA = colliderA->GetType();

        const AABB& treeAABB = tree.GetAABB(colliderA->node);
        tree.Query(treeAABB, this);
    }

    for (int32 i = 0; i < moveCount; ++i)
    {
        NodeIndex node = moveBuffer[i];
        if (node != AABBTree::nullNode)
        {
            tree.ClearMoved(node);
        }
    }

    moveCount = 0;
}

void BroadPhase::Add(Collider* collider, const AABB& aabb)
{
    NodeIndex node = tree.CreateNode(collider, aabb);
    collider->node = node;
    BufferMove(node);
}

void BroadPhase::Remove(Collider* collider)
{
    NodeIndex node = collider->node;
    if (node == AABBTree::nullNode)
    {
        return;
    }

    tree.RemoveNode(node);
    UnBufferMove(node);
}

void BroadPhase::Update(Collider* collider, const AABB& aabb, const Vec3& displacement)
{
    NodeIndex node = collider->node;
    bool rested = collider->body->resting > contactGraph->world->settings.sleeping_time;

    if (tree.MoveNode(node, aabb, displacement, rested))
    {
        BufferMove(node);
    }
}

void BroadPhase::Refresh(Collider* collider)
{
    NodeIndex node = collider->node;
    AABB aabb = collider->GetAABB();

    tree.MoveNode(node, aabb, Vec3::zero, true);
    BufferMove(node);
}

bool BroadPhase::QueryCallback(NodeIndex nodeB, Collider* colliderB)
{
    if (nodeA == nodeB)
    {
        return true;
    }

    RigidBody* bodyB = colliderB->body;
    if (bodyA == bodyB)
    {
        return true;
    }

    if (tree.WasMoved(nodeB) && nodeA < nodeB)
    {
        return true;
    }

    Shape::Type typeB = colliderB->GetType();
    if (typeA <= typeB)
    {
        contactGraph->OnNewContact(colliderB, colliderA);
    }
    else
    {
        contactGraph->OnNewContact(colliderA, colliderB);
    }

    return true;
}

} // namespace muli3
