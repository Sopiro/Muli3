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

        bodyA = tree.GetData(nodeA);
        if (!bodyA->shape)
        {
            continue;
        }

        typeA = bodyA->shape->GetType();

        const AABB& treeAABB = tree.GetAABB(bodyA->node);
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

void BroadPhase::Add(RigidBody* body, const AABB& aabb)
{
    NodeIndex node = tree.CreateNode(body, aabb);
    body->node = node;

    BufferMove(node);
}

void BroadPhase::Remove(RigidBody* body)
{
    NodeIndex node = body->node;
    if (node == AABBTree::nullNode)
    {
        return;
    }

    tree.RemoveNode(node);
    UnBufferMove(node);
}

void BroadPhase::Update(RigidBody* body, const AABB& aabb, const Vec3& displacement)
{
    NodeIndex node = body->node;
    bool rested = body->resting > contactGraph->world->settings.sleeping_time;

    bool nodeMoved = tree.MoveNode(node, aabb, displacement, rested);
    if (nodeMoved)
    {
        BufferMove(node);
    }
}

void BroadPhase::Refresh(RigidBody* body)
{
    NodeIndex node = body->node;
    AABB aabb;
    body->shape->ComputeAABB(body->transform, &aabb);

    tree.MoveNode(node, aabb, Vec3::zero, true);
    BufferMove(node);
}

bool BroadPhase::QueryCallback(NodeIndex nodeB, RigidBody* bodyB)
{
    if (nodeA == nodeB)
    {
        return true;
    }

    if (bodyA == bodyB)
    {
        return true;
    }

    if (tree.WasMoved(nodeB) && nodeA < nodeB)
    {
        return true;
    }

    Shape::Type typeB = bodyB->shape->GetType();
    if (typeA <= typeB)
    {
        contactGraph->OnNewContact(bodyB, bodyA);
    }
    else
    {
        contactGraph->OnNewContact(bodyA, bodyB);
    }

    return true;
}

} // namespace muli3
