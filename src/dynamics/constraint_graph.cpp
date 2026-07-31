#include "muli3/parallel_for.h"
#include "muli3/shapes.h"
#include "muli3/world.h"

namespace muli3
{

extern void InitializeDetectionFunctionMap();

ConstraintGraph::ConstraintGraph(World* world)
    : world{ world }
{
    InitializeDetectionFunctionMap();
}

ConstraintGraph::~ConstraintGraph()
{
    MuliAssert(contacts.empty());
}

void ConstraintGraph::EvaluateContacts()
{
    MuliProfileZoneNR(gather_contacts, "Gather Contacts", true);
    SolverSet& awakeSet = world->solverSets[awake_set];

    struct ContactSpan
    {
        ContactState* states;
        int32 count;
        int32 start;
    };

    int32 spanCount = 0;
    int32 activeCount = 0;
    ContactSpan spans[constraint_color_count + 1];

    for (int32 i = 0; i < constraint_color_count; ++i)
    {
        int32 count = int32(batches[i].contactStates.size());
        if (count > 0)
        {
            spans[spanCount++] = { batches[i].contactStates.data(), count, activeCount };
            activeCount += count;
        }
    }

    int32 awakeContactCount = int32(awakeSet.contactStates.size());
    if (awakeContactCount > 0)
    {
        spans[spanCount++] = { awakeSet.contactStates.data(), awakeContactCount, activeCount };
        activeCount += awakeContactCount;
    }

    if (activeCount == 0)
    {
        MuliProfileZoneEnd(gather_contacts);
        return;
    }

    int32 workerCount = world->settings.thread_pool ? world->settings.thread_pool->WorkerCount() : 1;
    int32 contactSlotCount = contactPool.GetCapacity();
    int32 contactWordCount = (contactSlotCount + 63) / 64;
    int32 contactWordStride = (contactWordCount + 7) & ~7;
    int32 contactBitSize = workerCount * contactWordStride * int32(sizeof(uint64));

    uint64* contactBits = (uint64*)world->linearAllocator.Allocate(contactBitSize);
    memset(contactBits, 0, contactBitSize);

    const auto SetBit = [](uint64* bits, int32 bit) { bits[bit >> 6] |= uint64(1) << (bit & 63); };

    MuliProfileZoneEnd(gather_contacts);

    // 1. Parallel Stage: Update manifolds in parallel.
    // The broad-phase overlap test (AABB query) and narrow-phase collision math (manifold calculations)
    // are strictly thread-safe as they read from body transforms and write only to their own Contact instances.
    ParallelFor(
        0, activeCount, 64,
        [&](int32 begin, int32 end, int32 workerIndex) {
            MuliAssert(workerIndex < workerCount);
            uint64* changedBits = contactBits + workerIndex * contactWordStride;

            int32 spanIndex = 0;
            while (spanIndex + 1 < spanCount && spans[spanIndex + 1].start <= begin)
            {
                ++spanIndex;
            }

            MuliProfileZoneNR(narrow_phase_collision, "Collide", true);
            for (int32 i = begin; i < end; ++i)
            {
                while (spanIndex + 1 < spanCount && spans[spanIndex + 1].start <= i)
                {
                    ++spanIndex;
                }

                Contact* contact = spans[spanIndex].states[i - spans[spanIndex].start].contact;

                // Perform broad phase overlap test.
                if (broadPhase.TestOverlap(contact->colliderA, contact->colliderB) == false)
                {
                    contact->flag |= Contact::flag_disjoint;
                    SetBit(changedBits, contact->poolIndex);
                    continue;
                }

                // Compute contact manifold and warm starting impulses.
                contact->Update();
                bool graphContact = contact->IsTouching() && contact->IsEnabled();
                bool inGraph = contact->colorIndex != null_index;
                if (graphContact != inGraph)
                {
                    SetBit(changedBits, contact->poolIndex);
                }
            }
            MuliProfileZoneEnd(narrow_phase_collision);
        },
        world->settings.thread_pool
    );

    MuliProfileZoneNR(post_narrow_phase, "Post Narrow Phase", true);

    // 2. Serial Stage: Integrate states, execute user callbacks, and destroy disjoint contacts.
    // Sequential execution on the main thread guarantees deterministic order of events.
    uint64* changedBits = contactBits;

    for (int32 s = 0; s < spanCount; ++s)
    {
        ContactSpan& span = spans[s];
        for (int32 i = 0; i < span.count; ++i)
        {
            Contact* contact = span.states[i].contact;

            if ((contact->flag & Contact::flag_disjoint) != 0)
            {
                if (contact->flag & Contact::flag_touching)
                {
                    contact->flag &= ~Contact::flag_touching;
                    contact->flag |= Contact::flag_was_touching;
                    contact->TriggerCallbacks();
                }
                continue;
            }

            // Trigger contact begin/end/touching listener callbacks sequentially.
            contact->TriggerCallbacks();

            bool graphContact = contact->IsTouching() && contact->IsEnabled();
            bool inGraph = contact->colorIndex != null_index;

            if (graphContact != inGraph)
            {
                SetBit(changedBits, contact->poolIndex);
            }
        }
    }

    // Merge worker-local contact state changes into worker 0 storage.
    for (int32 worker = 1; worker < workerCount; ++worker)
    {
        uint64* otherBits = contactBits + worker * contactWordStride;
        for (int32 i = 0; i < contactWordCount; ++i)
        {
            changedBits[i] |= otherBits[i];
        }
    }

    for (int32 word = 0; word < contactWordCount; ++word)
    {
        uint64 bits = changedBits[word];
        while (bits != 0)
        {
            int32 bit = int32(std::countr_zero(bits));
            int32 contactId = 64 * word + bit;
            Contact* contact = contactPool.Get(contactId);

            // Destroy disjoint contacts
            if ((contact->flag & Contact::flag_disjoint) != 0)
            {
                contact->flag &= ~Contact::flag_disjoint;
                Destroy(contact);

                bits &= bits - 1;
                continue;
            }

            bool graphContact = contact->IsTouching() && contact->IsEnabled();
            bool inGraph = contact->colorIndex != null_index;

            if (graphContact == inGraph)
            {
                bits &= bits - 1;
                continue;
            }

            if (graphContact)
            {
                Body* bodyA = contact->GetBodyA();
                Body* bodyB = contact->GetBodyB();

                if (!bodyA->IsStatic() && bodyA->IsSleeping())
                {
                    world->WakeIsland(bodyA);
                }

                if (!bodyB->IsStatic() && bodyB->IsSleeping())
                {
                    world->WakeIsland(bodyB);
                }

                // The contact just became active while it was stored as an awake non-touching contact.
                // Move state into the constraint graph so the solver can color it.
                int32 sourceIndex = contact->localIndex;
                ContactState state = std::move(awakeSet.contactStates[sourceIndex]);

                int32 last = int32(awakeSet.contactStates.size() - 1);
                if (sourceIndex != last)
                {
                    awakeSet.contactStates[sourceIndex] = std::move(awakeSet.contactStates[last]);
                    awakeSet.contactStates[sourceIndex].contact->localIndex = sourceIndex;
                }
                awakeSet.contactStates.pop_back();

                AddContactToGraph(contact, std::move(state));
            }
            else
            {
                // The contact is still awake, but no longer contributes constraints.
                // Keep it in awakeSet so the narrow phase can continue testing it.
                ContactState state = std::move(batches[contact->colorIndex].contactStates[contact->localIndex]);
                RemoveContactFromGraph(contact);

                int32 newIndex = int32(awakeSet.contactStates.size());
                awakeSet.contactStates.push_back(std::move(state));
                contact->setIndex = awake_set;
                contact->colorIndex = null_index;
                contact->localIndex = newIndex;
            }

            bits &= bits - 1;
        }
    }

    world->linearAllocator.Free(contactBits, contactBitSize);
    MuliProfileZoneEnd(post_narrow_phase);
}

void ConstraintGraph::OnNewContact(Collider* colliderA, Collider* colliderB)
{
    Body* bodyA = colliderA->body;
    Body* bodyB = colliderB->body;

    MuliAssert(bodyA != bodyB);
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    if (bodyA->GetType() != Body::dynamic_body && bodyB->GetType() != Body::dynamic_body)
    {
        return;
    }

    if (EvaluateFilter(colliderA->GetFilter(), colliderB->GetFilter()) == false)
    {
        return;
    }

    for (Contact* contact : bodyB->contacts)
    {
        Body* other = contact->GetBodyA() == bodyB ? contact->GetBodyB() : contact->GetBodyA();
        if (other == bodyA)
        {
            Collider* cA = contact->colliderA;
            Collider* cB = contact->colliderB;

            if ((colliderA == cA && colliderB == cB) || (colliderA == cB && colliderB == cA))
            {
                return;
            }
        }
    }

    int32 contactId = contactPool.NewId(colliderA, colliderB);
    Contact* c = contactPool.Get(contactId);
    c->poolIndex = contactId;

    c->graphIndex = int32(contacts.size());
    contacts.push_back(c);

    c->bodyIndexA = int32(bodyA->contacts.size());
    bodyA->contacts.push_back(c);

    c->bodyIndexB = int32(bodyB->contacts.size());
    bodyB->contacts.push_back(c);

    SolverSetIndex setIndex;
    if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
    {
        setIndex = disabled_set;
    }
    else
    {
        bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
        bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
        setIndex = awakeA || awakeB ? awake_set : sleeping_set;
    }

    world->AddContactState(c, setIndex);
}

void ConstraintGraph::Destroy(Contact* c)
{
    Body* bodyA = c->GetBodyA();
    Body* bodyB = c->GetBodyB();

    int32 index = c->graphIndex;
    MuliAssert(0 <= index && index < int32(contacts.size()));
    MuliAssert(contacts[index] == c);

    Contact* moved = contacts.back();
    contacts[index] = moved;
    moved->graphIndex = index;
    contacts.pop_back();

    index = c->bodyIndexA;
    MuliAssert(0 <= index && index < int32(bodyA->contacts.size()));
    MuliAssert(bodyA->contacts[index] == c);
    moved = bodyA->contacts.back();
    bodyA->contacts[index] = moved;
    if (moved->GetBodyA() == bodyA)
    {
        moved->bodyIndexA = index;
    }
    else
    {
        moved->bodyIndexB = index;
    }
    bodyA->contacts.pop_back();

    index = c->bodyIndexB;
    MuliAssert(0 <= index && index < int32(bodyB->contacts.size()));
    MuliAssert(bodyB->contacts[index] == c);
    moved = bodyB->contacts.back();
    bodyB->contacts[index] = moved;
    if (moved->GetBodyA() == bodyB)
    {
        moved->bodyIndexA = index;
    }
    else
    {
        moved->bodyIndexB = index;
    }
    bodyB->contacts.pop_back();

    if (c->colorIndex != null_index)
    {
        RemoveContactFromGraph(c);
        c->setIndex = null_index;
    }
    else
    {
        world->RemoveContactState(c);
    }

    MuliAssert(contactPool.Get(c->poolIndex) == c);
    contactPool.Delete(c->poolIndex);
}

void ConstraintGraph::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

void ConstraintGraph::RemoveCollider(Collider* collider)
{
    broadPhase.Remove(collider);
    collider->node = AABBTree::nullNode;

    Body* body = collider->body;
    for (int32 i = 0; i < int32(body->contacts.size());)
    {
        Contact* contact = body->contacts[i];

        Collider* colliderA = contact->GetColliderA();
        Collider* colliderB = contact->GetColliderB();

        if (collider == colliderA || collider == colliderB)
        {
            bool touching = contact->IsTouching();
            bool enabled = contact->IsEnabled();
            Body* bodyA = colliderA->body;
            Body* bodyB = colliderB->body;

            Destroy(contact);

            if (touching && enabled)
            {
                bodyA->Awake();
                bodyB->Awake();
            }

            continue;
        }

        ++i;
    }
}

void ConstraintGraph::UpdateCollider(Collider* collider, const Transform& transform)
{
    AABB aabb;
    collider->GetShape()->ComputeAABB(transform, &aabb);

    broadPhase.Update(collider, aabb, Vec3::zero, true);
}

void ConstraintGraph::UpdateCollider(Collider* collider, const Transform& transform0, const Transform& transform1)
{
    AABB aabb0;
    AABB aabb1;
    collider->GetShape()->ComputeAABB(transform0, &aabb0);
    collider->GetShape()->ComputeAABB(transform1, &aabb1);

    Vec3 prediction = aabb1.GetCenter() - aabb0.GetCenter();
    aabb1.min += prediction;
    aabb1.max += prediction;

    bool rested = collider->body->GetBodyState()->resting > world->settings.sleeping_time;
    broadPhase.Update(collider, AABB::Union(aabb0, aabb1), prediction, rested);
}

ContactState* ConstraintGraph::GetContactState(Contact* contact)
{
    return &batches[contact->colorIndex].contactStates[contact->localIndex];
}

const ContactState* ConstraintGraph::GetContactState(const Contact* contact) const
{
    return &batches[contact->colorIndex].contactStates[contact->localIndex];
}

JointState* ConstraintGraph::GetJointState(Joint* joint)
{
    return &batches[joint->colorIndex].jointStates[joint->localIndex];
}

const JointState* ConstraintGraph::GetJointState(const Joint* joint) const
{
    return &batches[joint->colorIndex].jointStates[joint->localIndex];
}

int32 ConstraintGraph::AssignColor(Body* bodyA, Body* bodyB)
{
    MuliAssert(constraint_overflow_index < 32);

    constexpr uint32 colorMask = (1u << constraint_overflow_index) - 1u;

    bool nonDynamicA = bodyA->IsDynamic() == false;
    bool nonDynamicB = bodyB->IsDynamic() == false;

    uint32 usedColors = 0;
    usedColors |= nonDynamicA ? 0 : bodyA->usedColors;
    usedColors |= nonDynamicB ? 0 : bodyB->usedColors;

    uint32 freeColors = ~usedColors & colorMask;
    if (freeColors == 0)
    {
        return constraint_overflow_index;
    }

    if (nonDynamicA || nonDynamicB)
    {
        // Find the highest free color index.
        // Solving non-dynamic constraints after dynamic-only constraints improves convergence.
        return 31 - std::countl_zero(freeColors);
    }
    else
    {
        // Find the lowest free color index.
        return std::countr_zero(freeColors);
    }
}

void ConstraintGraph::AddColor(Body* bodyA, Body* bodyB, int32 colorIndex)
{
    if (colorIndex == constraint_overflow_index)
    {
        return;
    }

    uint32 colorBit = 1u << colorIndex;
    if (bodyA->IsDynamic())
    {
        bodyA->usedColors |= colorBit;
    }
    if (bodyB->IsDynamic())
    {
        bodyB->usedColors |= colorBit;
    }
}

void ConstraintGraph::RemoveColor(Body* bodyA, Body* bodyB, int32 colorIndex)
{
    if (colorIndex == constraint_overflow_index)
    {
        return;
    }

    uint32 mask = ~(1u << colorIndex);
    bodyA->usedColors &= mask;
    bodyB->usedColors &= mask;
}

ContactState* ConstraintGraph::AddContactToGraph(Contact* contact, ContactState&& source)
{
    int32 colorIndex = AssignColor(contact->GetBodyA(), contact->GetBodyB());
    AddColor(contact->GetBodyA(), contact->GetBodyB(), colorIndex);

    ConstraintBatch& batch = batches[colorIndex];
    int32 index = int32(batch.contactStates.size());

    batch.contactStates.push_back(std::move(source));
    ContactState* state = &batch.contactStates.back();
    state->contact = contact;

    contact->setIndex = awake_set;
    contact->colorIndex = colorIndex;
    contact->localIndex = index;

    return state;
}

void ConstraintGraph::RemoveContactFromGraph(Contact* contact)
{
    int32 colorIndex = contact->colorIndex;
    int32 localIndex = contact->localIndex;

    MuliAssert(0 <= colorIndex && colorIndex < constraint_color_count);

    RemoveColor(contact->GetBodyA(), contact->GetBodyB(), colorIndex);

    // Swap remove
    std::vector<ContactState>& states = batches[colorIndex].contactStates;
    int32 last = int32(states.size() - 1);
    if (localIndex != last)
    {
        states[localIndex] = std::move(states[last]);
        states[localIndex].contact->localIndex = localIndex;
    }
    states.pop_back();

    contact->colorIndex = null_index;
    contact->localIndex = null_index;
}

JointState* ConstraintGraph::AddJointToGraph(Joint* joint, const JointState& source)
{
    MuliAssert(joint->setIndex == awake_set);

    int32 colorIndex = AssignColor(joint->GetBodyA(), joint->GetBodyB());
    AddColor(joint->GetBodyA(), joint->GetBodyB(), colorIndex);

    ConstraintBatch& batch = batches[colorIndex];
    int32 index = int32(batch.jointStates.size());

    batch.jointStates.push_back(source);
    JointState* state = &batch.jointStates.back();
    state->joint = joint;

    joint->colorIndex = colorIndex;
    joint->localIndex = index;

    return state;
}

void ConstraintGraph::RemoveJointFromGraph(Joint* joint)
{
    int32 colorIndex = joint->colorIndex;
    int32 localIndex = joint->localIndex;

    MuliAssert(0 <= colorIndex && colorIndex < constraint_color_count);

    RemoveColor(joint->GetBodyA(), joint->GetBodyB(), colorIndex);

    // Swap remove
    std::vector<JointState>& states = batches[colorIndex].jointStates;
    int32 last = int32(states.size() - 1);
    if (localIndex != last)
    {
        states[localIndex] = states[last];
        states[localIndex].joint->localIndex = localIndex;
    }
    states.pop_back();

    joint->colorIndex = null_index;
    joint->localIndex = null_index;
}

} // namespace muli3
