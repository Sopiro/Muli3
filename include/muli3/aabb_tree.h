#pragma once

#include "bounding_box.h"
#include "growable_array.h"
#include "rigidbody.h"

namespace muli3
{

using NodeIndex = int32;

inline float SurfaceArea(const AABB& aabb)
{
    return aabb.GetSurfaceArea();
}

class AABBTree
{
    using Data = RigidBody;

public:
    static constexpr inline int32 nullNode = -1;

    struct Node
    {
        bool IsLeaf() const
        {
            return child1 == nullNode;
        }

        AABB aabb;

        NodeIndex parent;
        NodeIndex child1;
        NodeIndex child2;

        NodeIndex next;
        bool moved;

        Data* data;
    };

    AABBTree();
    ~AABBTree();

    AABBTree(const AABBTree&) = delete;
    AABBTree& operator=(const AABBTree&) = delete;

    void Reset();

    NodeIndex CreateNode(Data* data, const AABB& aabb);
    bool MoveNode(NodeIndex node, AABB aabb, const Vec3& displacement, bool forceMove);
    void RemoveNode(NodeIndex node);

    bool TestOverlap(NodeIndex nodeA, NodeIndex nodeB) const;
    const AABB& GetAABB(NodeIndex node) const;
    void ClearMoved(NodeIndex node) const;
    bool WasMoved(NodeIndex node) const;
    Data* GetData(NodeIndex node) const;

    template <typename T>
    void Traverse(T* callback) const;
    template <typename T>
    void Query(const Vec3& point, T* callback) const;
    template <typename T>
    void Query(const AABB& aabb, T* callback) const;

    void Traverse(std::function<void(const Node*)> callback) const;
    void Query(const Vec3& point, std::function<bool(NodeIndex, Data*)> callback) const;
    void Query(const AABB& aabb, std::function<bool(NodeIndex, Data*)> callback) const;

    float ComputeTreeCost() const;
    void Rebuild();

private:
    NodeIndex root;

    Node* nodes;
    int32 nodeCapacity;
    int32 nodeCount;

    NodeIndex freeList;

    NodeIndex AllocateNode();
    void FreeNode(NodeIndex node);

    NodeIndex InsertLeaf(NodeIndex leaf);
    void RemoveLeaf(NodeIndex leaf);

    void Rotate(NodeIndex node);
};

inline bool AABBTree::TestOverlap(NodeIndex nodeA, NodeIndex nodeB) const
{
    MuliAssert(0 <= nodeA && nodeA < nodeCapacity);
    MuliAssert(0 <= nodeB && nodeB < nodeCapacity);

    return nodes[nodeA].aabb.TestOverlap(nodes[nodeB].aabb);
}

inline const AABB& AABBTree::GetAABB(NodeIndex node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].aabb;
}

inline void AABBTree::ClearMoved(NodeIndex node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    nodes[node].moved = false;
}

inline bool AABBTree::WasMoved(NodeIndex node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].moved;
}

inline AABBTree::Data* AABBTree::GetData(NodeIndex node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].data;
}

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&cost](const Node* node) -> void { cost += SurfaceArea(node->aabb); });

    return cost;
}

template <typename T>
void AABBTree::Traverse(T* callback) const
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

        const Node* node = nodes + current;
        callback->TraverseCallback(node);
    }
}

template <typename T>
void AABBTree::Query(const Vec3& point, T* callback) const
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
            bool proceed = callback->QueryCallback(current, nodes[current].data);
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

template <typename T>
void AABBTree::Query(const AABB& aabb, T* callback) const
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
            bool proceed = callback->QueryCallback(current, nodes[current].data);
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

} // namespace muli3
