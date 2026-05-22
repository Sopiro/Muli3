#include "muli3/broad_phase.h"
#include "muli3/contact_graph.h"
#include "muli3/parallel_for.h"
#include "muli3/world.h"

namespace muli3
{

BroadPhase::BroadPhase(ContactGraph* contactGraph)
    : contactGraph{ contactGraph }
    , moveCapacity{ 64 }
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
        moveCapacity = int32(1.5f * moveCapacity);
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

struct ColliderPair
{
    Collider* colliderA;
    Collider* colliderB;
};

struct MoveResult
{
    GrowableArray<ColliderPair, 8> pairs;
};

struct BroadPhase::TreeCallback
{
    const AABBTree* tree;

    NodeIndex nodeA;
    Collider* colliderA;
    RigidBody* bodyA;
    Shape::Type typeA;

    MoveResult* moveResult;

    bool QueryCallback(NodeIndex nodeB, Collider* colliderB)
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

        if (tree->WasMoved(nodeB) && nodeA < nodeB)
        {
            return true;
        }

        Shape::Type typeB = colliderB->GetType();

        if (typeA <= typeB)
        {
            moveResult->pairs.emplace_back(colliderB, colliderA);
        }
        else
        {
            moveResult->pairs.emplace_back(colliderA, colliderB);
        }

        return true;
    }
};

void BroadPhase::FindNewContacts()
{
    if (moveCount == 0)
    {
        return;
    }

    LinearAllocator& allocator = contactGraph->world->linearAllocator;

    // Allocate moveResults array for thread-isolated results
    int32 size = moveCount * sizeof(MoveResult);
    MoveResult* moveResults = (MoveResult*)allocator.Allocate(size);

    // Parallel Stage: Query tree for each moved proxy in parallel
    // The AABB tree query is read-only and fully thread-safe
    ParallelFor(0, moveCount, [this, moveResults](int32 i) {
        MuliProfileZoneNC(broad_phase_tree_query, "TreeQuery", color::random(123), true);

        NodeIndex node = moveBuffer[i];

        MoveResult* moveResult = moveResults + i;
        moveResult->pairs.reset();

        if (node == AABBTree::nullNode)
        {
            MuliProfileZoneEnd(broad_phase_tree_query);
            return;
        }

        Collider* colliderA = tree.GetData(node);
        RigidBody* bodyA = colliderA->body;
        Shape::Type tfA = colliderA->GetType();

        const AABB& treeAABB = tree.GetAABB(node);

        TreeCallback callback{ &tree, node, colliderA, bodyA, tfA, moveResult };
        tree.Query(treeAABB, &callback);

        // Reset move flags
        if (node != AABBTree::nullNode)
        {
            tree.ClearMoved(node);
        }

        MuliProfileZoneEnd(broad_phase_tree_query);
    });

    // Serial Stage: Deterministic contact creation
    // Sequential iteration guarantees deterministic contact ordering
    for (int32 i = 0; i < moveCount; ++i)
    {
        const MoveResult& result = moveResults[i];
        for (int32 j = 0; j < result.pairs.size(); ++j)
        {
            const ColliderPair& pair = result.pairs[j];
            contactGraph->OnNewContact(pair.colliderA, pair.colliderB);
        }

        result.pairs.~GrowableArray();
    }

    allocator.Free(moveResults, size);
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

} // namespace muli3
