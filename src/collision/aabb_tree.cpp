#include "muli3/aabb_tree.h"
#include "muli3/settings.h"

namespace muli3
{

AABBTree::AABBTree()
    : root{ nullNode }
    , nodeCapacity{ 32 }
    , nodeCount{ 0 }
{
    nodes = (Node*)muli3::Alloc(nodeCapacity * sizeof(Node));
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
        nodes[i].parent = i;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

    freeList = 0;
}

AABBTree::~AABBTree()
{
    muli3::Free(nodes);
    root = nullNode;
    nodeCount = 0;
}

NodeIndex AABBTree::InsertLeaf(NodeIndex leaf)
{
    MuliAssert(0 <= leaf && leaf < nodeCapacity);
    MuliAssert(nodes[leaf].IsLeaf());

    if (root == nullNode)
    {
        root = leaf;
        return leaf;
    }

    AABB aabb = nodes[leaf].aabb;
    NodeIndex bestSibling = root;
    float bestCost = SurfaceArea(AABB::Union(nodes[root].aabb, aabb));

    struct Candidate
    {
        NodeIndex node;
        float inheritedCost;
    };

    GrowableArray<Candidate, 256> stack;
    stack.EmplaceBack(root, 0.0f);

    while (stack.Count() != 0)
    {
        NodeIndex current = stack.Back().node;
        float inheritedCost = stack.Back().inheritedCost;
        stack.PopBack();

        AABB combined = AABB::Union(nodes[current].aabb, aabb);
        float directCost = SurfaceArea(combined);

        float cost = directCost + inheritedCost;
        if (cost < bestCost)
        {
            bestCost = cost;
            bestSibling = current;
        }

        inheritedCost += directCost - SurfaceArea(nodes[current].aabb);

        float lowerBoundCost = SurfaceArea(aabb) + inheritedCost;
        if (lowerBoundCost < bestCost && nodes[current].IsLeaf() == false)
        {
            stack.EmplaceBack(nodes[current].child1, inheritedCost);
            stack.EmplaceBack(nodes[current].child2, inheritedCost);
        }
    }

    NodeIndex oldParent = nodes[bestSibling].parent;
    NodeIndex newParent = AllocateNode();
    nodes[newParent].aabb = AABB::Union(aabb, nodes[bestSibling].aabb);
    nodes[newParent].data = nullptr;
    nodes[newParent].parent = oldParent;

    nodes[newParent].child1 = leaf;
    nodes[newParent].child2 = bestSibling;
    nodes[leaf].parent = newParent;
    nodes[bestSibling].parent = newParent;

    if (oldParent != nullNode)
    {
        if (nodes[oldParent].child1 == bestSibling)
        {
            nodes[oldParent].child1 = newParent;
        }
        else
        {
            nodes[oldParent].child2 = newParent;
        }
    }
    else
    {
        root = newParent;
    }

    NodeIndex ancestor = newParent;
    while (ancestor != nullNode)
    {
        NodeIndex child1 = nodes[ancestor].child1;
        NodeIndex child2 = nodes[ancestor].child2;

        nodes[ancestor].aabb = AABB::Union(nodes[child1].aabb, nodes[child2].aabb);

        Rotate(ancestor);

        ancestor = nodes[ancestor].parent;
    }

    return leaf;
}

void AABBTree::RemoveLeaf(NodeIndex leaf)
{
    MuliAssert(0 <= leaf && leaf < nodeCapacity);
    MuliAssert(nodes[leaf].IsLeaf());

    NodeIndex parent = nodes[leaf].parent;
    if (parent == nullNode)
    {
        MuliAssert(root == leaf);
        root = nullNode;
        return;
    }

    NodeIndex grandParent = nodes[parent].parent;
    NodeIndex sibling = nodes[parent].child1 == leaf ? nodes[parent].child2 : nodes[parent].child1;

    FreeNode(parent);

    if (grandParent != nullNode)
    {
        nodes[sibling].parent = grandParent;

        if (nodes[grandParent].child1 == parent)
        {
            nodes[grandParent].child1 = sibling;
        }
        else
        {
            nodes[grandParent].child2 = sibling;
        }

        NodeIndex ancestor = grandParent;
        while (ancestor != nullNode)
        {
            NodeIndex child1 = nodes[ancestor].child1;
            NodeIndex child2 = nodes[ancestor].child2;

            nodes[ancestor].aabb = AABB::Union(nodes[child1].aabb, nodes[child2].aabb);

            Rotate(ancestor);

            ancestor = nodes[ancestor].parent;
        }
    }
    else
    {
        root = sibling;
        nodes[sibling].parent = nullNode;
    }
}

NodeIndex AABBTree::CreateNode(Data* data, const AABB& aabb)
{
    NodeIndex newNode = AllocateNode();

    nodes[newNode].aabb.max = aabb.max + aabb_margin;
    nodes[newNode].aabb.min = aabb.min - aabb_margin;
    nodes[newNode].data = data;
    nodes[newNode].parent = nullNode;
    nodes[newNode].moved = true;

    InsertLeaf(newNode);

    return newNode;
}

bool AABBTree::MoveNode(NodeIndex node, AABB aabb, const Vec3& displacement, bool forceMove)
{
    MuliAssert(0 <= node && node < nodeCapacity);
    MuliAssert(nodes[node].IsLeaf());

    const AABB& treeAABB = nodes[node].aabb;
    if (treeAABB.Contains(aabb) && forceMove == false)
    {
        return false;
    }

    Vec3 d = displacement * aabb_multiplier;

    if (d.x > 0.0f)
        aabb.max.x += d.x;
    else
        aabb.min.x += d.x;

    if (d.y > 0.0f)
        aabb.max.y += d.y;
    else
        aabb.min.y += d.y;

    if (d.z > 0.0f)
        aabb.max.z += d.z;
    else
        aabb.min.z += d.z;

    aabb.max = aabb.max + aabb_margin;
    aabb.min = aabb.min - aabb_margin;

    RemoveLeaf(node);
    nodes[node].aabb = aabb;
    InsertLeaf(node);

    nodes[node].moved = true;

    return true;
}

void AABBTree::RemoveNode(NodeIndex node)
{
    MuliAssert(0 <= node && node < nodeCapacity);
    MuliAssert(nodes[node].IsLeaf());

    RemoveLeaf(node);
    FreeNode(node);
}

void AABBTree::Rotate(NodeIndex node)
{
    if (nodes[node].IsLeaf())
    {
        return;
    }

    NodeIndex child1 = nodes[node].child1;
    NodeIndex child2 = nodes[node].child2;

    float costDiffs[4] = { 0.0f };

    if (nodes[child1].IsLeaf() == false)
    {
        float area1 = SurfaceArea(nodes[child1].aabb);
        costDiffs[0] = SurfaceArea(AABB::Union(nodes[nodes[child1].child1].aabb, nodes[child2].aabb)) - area1;
        costDiffs[1] = SurfaceArea(AABB::Union(nodes[nodes[child1].child2].aabb, nodes[child2].aabb)) - area1;
    }

    if (nodes[child2].IsLeaf() == false)
    {
        float area2 = SurfaceArea(nodes[child2].aabb);
        costDiffs[2] = SurfaceArea(AABB::Union(nodes[nodes[child2].child1].aabb, nodes[child1].aabb)) - area2;
        costDiffs[3] = SurfaceArea(AABB::Union(nodes[nodes[child2].child2].aabb, nodes[child1].aabb)) - area2;
    }

    int32 bestDiffIndex = 0;
    for (int32 i = 1; i < 4; ++i)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex])
        {
            bestDiffIndex = i;
        }
    }

    if (costDiffs[bestDiffIndex] >= 0.0f)
    {
        return;
    }

    switch (bestDiffIndex)
    {
    case 0:
        nodes[nodes[child1].child2].parent = node;
        nodes[node].child2 = nodes[child1].child2;
        nodes[child1].child2 = child2;
        nodes[child2].parent = child1;
        nodes[child1].aabb = AABB::Union(nodes[nodes[child1].child1].aabb, nodes[nodes[child1].child2].aabb);
        break;
    case 1:
        nodes[nodes[child1].child1].parent = node;
        nodes[node].child2 = nodes[child1].child1;
        nodes[child1].child1 = child2;
        nodes[child2].parent = child1;
        nodes[child1].aabb = AABB::Union(nodes[nodes[child1].child1].aabb, nodes[nodes[child1].child2].aabb);
        break;
    case 2:
        nodes[nodes[child2].child2].parent = node;
        nodes[node].child1 = nodes[child2].child2;
        nodes[child2].child2 = child1;
        nodes[child1].parent = child2;
        nodes[child2].aabb = AABB::Union(nodes[nodes[child2].child1].aabb, nodes[nodes[child2].child2].aabb);
        break;
    case 3:
        nodes[nodes[child2].child1].parent = node;
        nodes[node].child1 = nodes[child2].child1;
        nodes[child2].child1 = child1;
        nodes[child1].parent = child2;
        nodes[child2].aabb = AABB::Union(nodes[nodes[child2].child1].aabb, nodes[nodes[child2].child2].aabb);
        break;
    }
}

void AABBTree::Traverse(std::function<void(const Node*)> callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeIndex, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeIndex current = stack.PopBack();

        if (nodes[current].IsLeaf() == false)
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
        }

        callback(nodes + current);
    }
}

void AABBTree::Query(const Vec3& point, std::function<bool(NodeIndex, Data*)> callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeIndex, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeIndex current = stack.PopBack();

        if (nodes[current].aabb.TestPoint(point) == false)
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback(current, nodes[current].data);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
        }
    }
}

void AABBTree::Query(const AABB& aabb, std::function<bool(NodeIndex, Data*)> callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeIndex, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeIndex current = stack.PopBack();

        if (nodes[current].aabb.TestOverlap(aabb) == false)
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback(current, nodes[current].data);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
        }
    }
}

void AABBTree::AABBCast(const AABBCastInput& input, std::function<float(const AABBCastInput& input, Data* data)> callback) const
{
    if (root == nullNode)
    {
        return;
    }

    const Vec3 p1 = input.from;
    const Vec3 p2 = input.to;
    const Vec3 halfExtents = input.halfExtents;

    float maxFraction = input.maxFraction;
    Vec3 d = p2 - p1;

    if (Length2(d) == 0.0f)
    {
        return;
    }

    Ray ray{ p1, d };

    GrowableArray<NodeIndex, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() > 0)
    {
        NodeIndex current = stack.PopBack();
        if (current == nullNode)
        {
            continue;
        }

        const Node* node = nodes + current;

        if (node->IsLeaf())
        {
            AABBCastInput subInput;
            subInput.from = p1;
            subInput.to = p2;
            subInput.maxFraction = maxFraction;
            subInput.halfExtents = halfExtents;

            float newFraction = callback(subInput, node->data);
            if (newFraction == 0.0f)
            {
                return;
            }

            if (newFraction > 0.0f)
            {
                maxFraction = newFraction;
            }
        }
        else
        {
            NodeIndex child1 = node->child1;
            NodeIndex child2 = node->child2;

            AABB aabb1{
                nodes[child1].aabb.min - halfExtents,
                nodes[child1].aabb.max + halfExtents,
            };
            AABB aabb2{
                nodes[child2].aabb.min - halfExtents,
                nodes[child2].aabb.max + halfExtents,
            };

            float dist1 = max_float;
            float dist2 = max_float;

            if (!aabb1.Intersect(ray, 0.0f, maxFraction, &dist1))
            {
                dist1 = max_float;
            }
            if (!aabb2.Intersect(ray, 0.0f, maxFraction, &dist2))
            {
                dist2 = max_float;
            }

            if (dist2 < dist1)
            {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }

            if (dist1 == max_float)
            {
                continue;
            }

            if (dist2 != max_float)
            {
                stack.EmplaceBack(child2);
            }
            stack.EmplaceBack(child1);
        }
    }
}

void AABBTree::Reset()
{
    root = nullNode;
    nodeCount = 0;
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
        nodes[i].parent = i;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

    freeList = 0;
}

NodeIndex AABBTree::AllocateNode()
{
    if (freeList == nullNode)
    {
        MuliAssert(nodeCount == nodeCapacity);

        Node* oldNodes = nodes;
        nodeCapacity += nodeCapacity / 2;
        nodes = (Node*)muli3::Alloc(nodeCapacity * sizeof(Node));
        memcpy(nodes, oldNodes, nodeCount * sizeof(Node));
        memset(nodes + nodeCount, 0, (nodeCapacity - nodeCount) * sizeof(Node));
        muli3::Free(oldNodes);

        for (int32 i = nodeCount; i < nodeCapacity - 1; ++i)
        {
            nodes[i].next = i + 1;
            nodes[i].parent = i;
        }
        nodes[nodeCapacity - 1].next = nullNode;
        nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

        freeList = nodeCount;
    }

    NodeIndex node = freeList;
    freeList = nodes[node].next;
    nodes[node].parent = nullNode;
    nodes[node].child1 = nullNode;
    nodes[node].child2 = nullNode;
    nodes[node].moved = false;
    ++nodeCount;

    return node;
}

void AABBTree::FreeNode(NodeIndex node)
{
    MuliAssert(0 <= node && node <= nodeCapacity);
    MuliAssert(0 < nodeCount);

    nodes[node].parent = node;
    nodes[node].next = freeList;
    freeList = node;

    --nodeCount;
}

void AABBTree::Rebuild()
{
    NodeIndex* leaves = (NodeIndex*)muli3::Alloc(nodeCount * sizeof(NodeIndex));
    int32 count = 0;

    for (int32 i = 0; i < nodeCapacity; ++i)
    {
        if (nodes[i].parent == i)
        {
            continue;
        }

        if (nodes[i].IsLeaf())
        {
            nodes[i].parent = nullNode;
            leaves[count++] = i;
        }
        else
        {
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        float minCost = max_float;
        int32 minI = -1;
        int32 minJ = -1;

        for (int32 i = 0; i < count; ++i)
        {
            AABB aabbI = nodes[leaves[i]].aabb;

            for (int32 j = i + 1; j < count; ++j)
            {
                AABB aabbJ = nodes[leaves[j]].aabb;
                float cost = SurfaceArea(AABB::Union(aabbI, aabbJ));

                if (cost < minCost)
                {
                    minCost = cost;
                    minI = i;
                    minJ = j;
                }
            }
        }

        NodeIndex index1 = leaves[minI];
        NodeIndex index2 = leaves[minJ];

        NodeIndex parentIndex = AllocateNode();
        Node* parent = nodes + parentIndex;

        parent->child1 = index1;
        parent->child2 = index2;
        parent->aabb = AABB::Union(nodes[index1].aabb, nodes[index2].aabb);
        parent->parent = nullNode;

        nodes[index1].parent = parentIndex;
        nodes[index2].parent = parentIndex;

        leaves[minI] = parentIndex;
        leaves[minJ] = leaves[count - 1];
        --count;
    }

    root = count == 0 ? nullNode : leaves[0];
    muli3::Free(leaves);
}

} // namespace muli3
