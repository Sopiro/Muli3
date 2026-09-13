#include "muli3/bitset.h"
#include "muli3/parallel_for.h"
#include "muli3/shapes.h"
#include "muli3/world.h"

namespace muli3
{

extern void InitializeDetectionFunctionMap();

ConstraintGraph::ConstraintGraph(World* world)
    : world{ world }
    , batches{ constraint_color_count }
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

    int32 activeCount = 0;

    for (int32 i = 0; i < constraint_color_count; ++i)
    {
        activeCount += batches[i].blockContacts.Count();
        activeCount += batches[i].scalarContacts.Count();
    }

    int32 awakeContactCount = int32(awakeSet.contactStates.size());
    activeCount += awakeContactCount;

    if (activeCount == 0)
    {
        MuliProfileZoneEnd(gather_contacts);
        return;
    }

    int32 activeContactSize = activeCount * sizeof(Contact*);
    Contact** activeContacts = (Contact**)world->linearAllocator.Allocate(activeContactSize);
    int32 activeIndex = 0;

    for (int32 i = 0; i < constraint_color_count; ++i)
    {
        BlockContactArray& blockContacts = batches[i].blockContacts;
        int32 blockContactCount = blockContacts.Count();
        if (blockContactCount > 0)
        {
            memcpy(activeContacts + activeIndex, blockContacts.state.contacts.data(), blockContactCount * sizeof(Contact*));
            activeIndex += blockContactCount;
        }

        ScalarContactArray& scalarContacts = batches[i].scalarContacts;
        for (ContactState& state : scalarContacts.states)
        {
            activeContacts[activeIndex++] = state.contact;
        }
    }

    for (ContactState& state : awakeSet.contactStates)
    {
        activeContacts[activeIndex++] = state.contact;
    }
    MuliAssert(activeIndex == activeCount);

    int32 workerCount = world->settings.thread_pool ? world->settings.thread_pool->WorkerCount() : 1;
    Bitset contactBits = AllocateBitset(contactPool.GetCapacity(), workerCount, &world->linearAllocator);

    MuliProfileZoneEnd(gather_contacts);

    // 1. Parallel Stage: Update manifolds in parallel.
    // The broad-phase overlap test (AABB query) and narrow-phase collision math (manifold calculations)
    // are strictly thread-safe as they read from body transforms and write only to their own Contact instances.
    ParallelFor(
        0, activeCount, 64,
        [&](int32 begin, int32 end, int32 workerIndex) {
            MuliAssert(workerIndex < workerCount);

            MuliProfileZoneNR(narrow_phase_collision, "Collide", true);
            for (int32 i = begin; i < end; ++i)
            {
                Contact* contact = activeContacts[i];

                // Perform broad phase overlap test.
                if (broadPhase.TestOverlap(contact->colliderA, contact->colliderB) == false)
                {
                    contact->flag |= Contact::flag_disjoint;
                    SetBit(&contactBits, workerIndex, contact->poolIndex);
                    continue;
                }

                // Compute contact manifold and warm starting impulses.
                contact->Update();
                bool graphContact = contact->IsTouching() && contact->IsEnabled();
                bool inGraph = contact->colorIndex != null_index;
                if (graphContact != inGraph)
                {
                    SetBit(&contactBits, workerIndex, contact->poolIndex);
                }
            }
            MuliProfileZoneEnd(narrow_phase_collision);
        },
        world->settings.thread_pool
    );

    MuliProfileZoneNR(post_narrow_phase, "Post Narrow Phase", true);

    // 2. Serial Stage: Integrate states, execute user callbacks, and destroy disjoint contacts.
    // Sequential execution on the main thread guarantees deterministic order of events.
    for (int32 i = 0; i < activeCount; ++i)
    {
        Contact* contact = activeContacts[i];

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
            SetBit(&contactBits, 0, contact->poolIndex);
        }
    }

    // Merge worker-local contact state changes into worker 0 storage.
    MergeBitset(&contactBits);

    for (int32 word = 0; word < contactBits.wordCount; ++word)
    {
        uint64 bits = contactBits.bits[word];
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
                ContactState state = RemoveContactFromGraph(contact);

                int32 newIndex = int32(awakeSet.contactStates.size());
                awakeSet.contactStates.push_back(std::move(state));
                contact->setIndex = awake_set;
                contact->colorIndex = null_index;
                contact->localIndex = newIndex;
            }

            bits &= bits - 1;
        }
    }

    FreeBitset(&contactBits, &world->linearAllocator);
    world->linearAllocator.Free(activeContacts, activeContactSize);
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

void ConstraintGraph::AddContactToGraph(Contact* contact, ContactState&& source)
{
    int32 colorIndex = AssignColor(contact->GetBodyA(), contact->GetBodyB());
    AddColor(contact->GetBodyA(), contact->GetBodyB(), colorIndex);

    ConstraintBatch& batch = batches[colorIndex];
    int32 index;

    bool blockContact = colorIndex != constraint_overflow_index && contact->GetColliderA()->shape->IsSimpleShape();
    if (blockContact)
    {
        contact->flag |= Contact::flag_simple;
        Body* bodyA = contact->GetBodyA();
        Body* bodyB = contact->GetBodyB();
        index = batch.blockContacts.Add(
            contact, bodyA->setIndex, bodyA->localIndex, bodyB->setIndex, bodyB->localIndex, std::move(source)
        );
    }
    else
    {
        contact->flag &= ~Contact::flag_simple;
        index = batch.scalarContacts.Add(contact, std::move(source));
    }

    contact->setIndex = awake_set;
    contact->colorIndex = colorIndex;
    contact->localIndex = index;
}

ContactState ConstraintGraph::RemoveContactFromGraph(Contact* contact)
{
    int32 colorIndex = contact->colorIndex;
    int32 localIndex = contact->localIndex;

    MuliAssert(0 <= colorIndex && colorIndex < constraint_color_count);

    RemoveColor(contact->GetBodyA(), contact->GetBodyB(), colorIndex);

    ConstraintBatch& batch = batches[colorIndex];
    ContactState state;

    Contact* movedContact;
    if (contact->IsSimpleContact())
    {
        movedContact = batch.blockContacts.Remove(localIndex, &state);
    }
    else
    {
        movedContact = batch.scalarContacts.Remove(localIndex, &state);
    }

    if (movedContact)
    {
        movedContact->localIndex = localIndex;
    }

    contact->colorIndex = null_index;
    contact->localIndex = null_index;
    contact->flag &= ~Contact::flag_simple;

    return state;
}

void ConstraintGraph::AddJointToGraph(Joint* joint, JointState&& source)
{
    MuliAssert(joint->setIndex == awake_set);

    int32 colorIndex = AssignColor(joint->GetBodyA(), joint->GetBodyB());
    AddColor(joint->GetBodyA(), joint->GetBodyB(), colorIndex);

    ConstraintBatch& batch = batches[colorIndex];
    int32 index = batch.scalarJoints.Add(joint, std::move(source));

    joint->colorIndex = colorIndex;
    joint->localIndex = index;
}

JointState ConstraintGraph::RemoveJointFromGraph(Joint* joint)
{
    int32 colorIndex = joint->colorIndex;
    int32 localIndex = joint->localIndex;

    MuliAssert(0 <= colorIndex && colorIndex < constraint_color_count);

    RemoveColor(joint->GetBodyA(), joint->GetBodyB(), colorIndex);

    JointState state;
    Joint* movedJoint = batches[colorIndex].scalarJoints.Remove(localIndex, &state);
    if (movedJoint)
    {
        movedJoint->localIndex = localIndex;
    }

    joint->colorIndex = null_index;
    joint->localIndex = null_index;

    return state;
}

} // namespace muli3
