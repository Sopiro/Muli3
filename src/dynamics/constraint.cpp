#include "muli3/constraint.h"
#include "muli3/simd.h"

namespace muli3
{

static void SetLane(FloatBlock* block, int32 lane, Float value)
{
    block->lane[lane] = value;
}

static void SetLane(IntBlock* block, int32 lane, int32 value)
{
    block->lane[lane] = value;
}

static void SetLane(Vec2Block* block, int32 lane, const Vec2& value)
{
    block->x.lane[lane] = value.x;
    block->y.lane[lane] = value.y;
}

static void SetLane(Vec3Block* block, int32 lane, const Vec3& value)
{
    block->x.lane[lane] = value.x;
    block->y.lane[lane] = value.y;
    block->z.lane[lane] = value.z;
}

static Float GetLane(const FloatBlock& block, int32 lane)
{
    return block.lane[lane];
}

static int32 GetLane(const IntBlock& block, int32 lane)
{
    return block.lane[lane];
}

static Vec2 GetLane(const Vec2Block& block, int32 lane)
{
    return { block.x.lane[lane], block.y.lane[lane] };
}

static Vec3 GetLane(const Vec3Block& block, int32 lane)
{
    return { block.x.lane[lane], block.y.lane[lane], block.z.lane[lane] };
}

template <typename T>
static void CopyLane(std::vector<T>* blocks, int32 dstIndex, int32 srcIndex)
{
    int32 dstBlock = dstIndex / simd_width;
    int32 dstLane = dstIndex % simd_width;
    int32 srcBlock = srcIndex / simd_width;
    int32 srcLane = srcIndex % simd_width;
    SetLane(&(*blocks)[dstBlock], dstLane, GetLane((*blocks)[srcBlock], srcLane));
}

template <typename T>
static void ResizeBlocks(std::vector<T>* blocks, int32 blockCount)
{
    blocks->resize(blockCount);
}

void BlockContactState::Resize(int32 blockCount)
{
    ResizeBlocks(&contacts, blockCount);
    ResizeBlocks(&bodySetA, blockCount);
    ResizeBlocks(&bodyIndexA, blockCount);
    ResizeBlocks(&bodySetB, blockCount);
    ResizeBlocks(&bodyIndexB, blockCount);
    ResizeBlocks(&friction, blockCount);
    ResizeBlocks(&restitution, blockCount);
    ResizeBlocks(&restitutionThreshold, blockCount);
    ResizeBlocks(&surfaceSpeed, blockCount);
    ResizeBlocks(&manifoldId, blockCount);
    ResizeBlocks(&pointCount, blockCount);
    ResizeBlocks(&normal, blockCount);
    ResizeBlocks(&linearImpulse, blockCount);
    ResizeBlocks(&angularImpulse, blockCount);

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        ResizeBlocks(&pointId[i], blockCount);
        ResizeBlocks(&anchorA[i], blockCount);
        ResizeBlocks(&anchorB[i], blockCount);
        ResizeBlocks(&normalImpulse[i], blockCount);
    }
}

void BlockContactConstraint::Resize(int32 blockCount)
{
    ResizeBlocks(&bodyA, blockCount);
    ResizeBlocks(&bodyB, blockCount);
    ResizeBlocks(&invMassA, blockCount);
    ResizeBlocks(&invMassB, blockCount);
    ResizeBlocks(&invIA, blockCount);
    ResizeBlocks(&invIB, blockCount);
    ResizeBlocks(&localInvIA, blockCount);
    ResizeBlocks(&localInvIB, blockCount);
    ResizeBlocks(&localNormal, blockCount);
    ResizeBlocks(&tangent1, blockCount);
    ResizeBlocks(&tangent2, blockCount);
    ResizeBlocks(&frictionWA1, blockCount);
    ResizeBlocks(&frictionWA2, blockCount);
    ResizeBlocks(&frictionWB1, blockCount);
    ResizeBlocks(&frictionWB2, blockCount);
    ResizeBlocks(&linearMass, blockCount);
    ResizeBlocks(&tangentBias, blockCount);
    ResizeBlocks(&tangentImpulse, blockCount);
    ResizeBlocks(&angularMass, blockCount);

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        ResizeBlocks(&normalWA[i], blockCount);
        ResizeBlocks(&normalWB[i], blockCount);
        ResizeBlocks(&normalMass[i], blockCount);
        ResizeBlocks(&normalBias[i], blockCount);
        ResizeBlocks(&leverArm[i], blockCount);
        ResizeBlocks(&localPointA[i], blockCount);
        ResizeBlocks(&localPointB[i], blockCount);
    }
}

int32 ScalarContactArray::Add(Contact* contact, ContactState&& state)
{
    int32 index = int32(states.size());
    state.contact = contact;
    states.push_back(std::move(state));
    constraints.emplace_back();
    return index;
}

Contact* ScalarContactArray::Remove(int32 index, ContactState* removed)
{
    MuliAssert(0 <= index && index < int32(states.size()));
    *removed = std::move(states[index]);
    int32 last = int32(states.size() - 1);

    Contact* movedContact;
    if (index != last)
    {
        states[index] = std::move(states[last]);
        constraints[index] = std::move(constraints[last]);

        movedContact = states[index].contact;
    }
    else
    {
        movedContact = nullptr;
    }

    states.pop_back();
    constraints.pop_back();

    return movedContact;
}

int32 ScalarJointArray::Add(Joint* joint, JointState&& state)
{
    int32 index = int32(states.size());
    state.joint = joint;
    states.push_back(std::move(state));
    return index;
}

Joint* ScalarJointArray::Remove(int32 index, JointState* removed)
{
    MuliAssert(0 <= index && index < int32(states.size()));
    *removed = std::move(states[index]);
    int32 last = int32(states.size() - 1);

    Joint* movedJoint;
    if (index != last)
    {
        states[index] = std::move(states[last]);
        movedJoint = states[index].joint;
    }
    else
    {
        movedJoint = nullptr;
    }

    states.pop_back();
    return movedJoint;
}

int32 BlockContactArray::Add(Contact* contact, int32 setA, int32 indexA, int32 setB, int32 indexB, ContactState&& inState)
{
    MuliAssert(inState.manifolds.size() == 1);
    int32 index = count++;
    int32 block = index / simd_width;
    int32 lane = index % simd_width;

    int32 requiredBlockCount = block + 1;
    if (requiredBlockCount > blockCapacity)
    {
        int32 newCapacity = blockCapacity > 0 ? blockCapacity + blockCapacity / 2 : 16;
        newCapacity = std::max(newCapacity, requiredBlockCount);
        state.Resize(newCapacity);
        constraint.Resize(newCapacity);
        blockCapacity = newCapacity;
    }

    const ContactManifold& manifold = inState.manifolds[0];
    state.contacts[block].lane[lane] = contact;
    state.bodySetA[block].lane[lane] = setA;
    state.bodyIndexA[block].lane[lane] = indexA;
    state.bodySetB[block].lane[lane] = setB;
    state.bodyIndexB[block].lane[lane] = indexB;
    constraint.bodyA[block].lane[lane] = nullptr;
    constraint.bodyB[block].lane[lane] = nullptr;
    SetLane(&state.friction[block], lane, inState.friction);
    SetLane(&state.restitution[block], lane, inState.restitution);
    SetLane(&state.restitutionThreshold[block], lane, inState.restitutionThreshold);
    SetLane(&state.surfaceSpeed[block], lane, inState.surfaceSpeed);
    SetLane(&state.manifoldId[block], lane, manifold.id);
    SetLane(&state.pointCount[block], lane, Float(manifold.contactCount));
    SetLane(&state.normal[block], lane, manifold.normal);
    SetLane(&state.linearImpulse[block], lane, manifold.linearImpulse);
    SetLane(&state.angularImpulse[block], lane, manifold.angularImpulse);

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        if (i < manifold.contactCount)
        {
            const ContactPoint& point = manifold.contactPoints[i];
            SetLane(&state.pointId[i][block], lane, point.id);
            SetLane(&state.anchorA[i][block], lane, point.anchorA);
            SetLane(&state.anchorB[i][block], lane, point.anchorB);
            SetLane(&state.normalImpulse[i][block], lane, point.impulse);
        }
        else
        {
            SetLane(&state.pointId[i][block], lane, 0);
            SetLane(&state.anchorA[i][block], lane, Vec3::zero);
            SetLane(&state.anchorB[i][block], lane, Vec3::zero);
            SetLane(&state.normalImpulse[i][block], lane, Float(0));
        }
    }

    return index;
}

Contact* BlockContactArray::Remove(int32 index, ContactState* removed)
{
    MuliAssert(0 <= index && index < count);
    int32 block = index / simd_width;
    int32 lane = index % simd_width;

    removed->contact = state.contacts[block].lane[lane];
    removed->friction = GetLane(state.friction[block], lane);
    removed->restitution = GetLane(state.restitution[block], lane);
    removed->restitutionThreshold = GetLane(state.restitutionThreshold[block], lane);
    removed->surfaceSpeed = GetLane(state.surfaceSpeed[block], lane);

    ContactManifold& manifold = removed->manifolds.emplace_back();
    manifold.id = GetLane(state.manifoldId[block], lane);
    manifold.contactCount = int32(GetLane(state.pointCount[block], lane));
    manifold.normal = GetLane(state.normal[block], lane);
    manifold.linearImpulse = GetLane(state.linearImpulse[block], lane);
    manifold.angularImpulse = GetLane(state.angularImpulse[block], lane);
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        ContactPoint& point = manifold.contactPoints[i];
        point.id = GetLane(state.pointId[i][block], lane);
        point.anchorA = GetLane(state.anchorA[i][block], lane);
        point.anchorB = GetLane(state.anchorB[i][block], lane);
        point.impulse = GetLane(state.normalImpulse[i][block], lane);
    }

    int32 last = count - 1;

    Contact* movedContact;
    if (index != last)
    {
        int32 dstBlock = index / simd_width;
        int32 dstLane = index % simd_width;
        int32 srcBlock = last / simd_width;
        int32 srcLane = last % simd_width;

        state.contacts[dstBlock].lane[dstLane] = state.contacts[srcBlock].lane[srcLane];
        CopyLane(&state.bodySetA, index, last);
        CopyLane(&state.bodyIndexA, index, last);
        CopyLane(&state.bodySetB, index, last);
        CopyLane(&state.bodyIndexB, index, last);
        CopyLane(&state.friction, index, last);
        CopyLane(&state.restitution, index, last);
        CopyLane(&state.restitutionThreshold, index, last);
        CopyLane(&state.surfaceSpeed, index, last);
        CopyLane(&state.manifoldId, index, last);
        CopyLane(&state.pointCount, index, last);
        CopyLane(&state.normal, index, last);
        CopyLane(&state.linearImpulse, index, last);
        CopyLane(&state.angularImpulse, index, last);

        for (int32 i = 0; i < max_contact_point_count; ++i)
        {
            CopyLane(&state.pointId[i], index, last);
            CopyLane(&state.anchorA[i], index, last);
            CopyLane(&state.anchorB[i], index, last);
            CopyLane(&state.normalImpulse[i], index, last);
        }

        movedContact = state.contacts[dstBlock].lane[dstLane];
    }
    else
    {
        movedContact = nullptr;
    }

    --count;
    if (count > 0 && count % simd_width != 0)
    {
        int32 tailBlock = count / simd_width;
        int32 tailLane = count % simd_width;
        state.contacts[tailBlock].lane[tailLane] = nullptr;
        state.pointCount[tailBlock].lane[tailLane] = 0.0f;
        constraint.bodyA[tailBlock].lane[tailLane] = nullptr;
        constraint.bodyB[tailBlock].lane[tailLane] = nullptr;
        for (int32 i = 0; i < max_contact_point_count; ++i)
        {
            state.normalImpulse[i][tailBlock].lane[tailLane] = 0.0f;
        }
    }

    return movedContact;
}

} // namespace muli3
