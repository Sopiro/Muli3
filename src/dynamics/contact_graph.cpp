#include "muli3/contact_graph.h"
#include "muli3/parallel_for.h"
#include "muli3/world.h"

namespace muli3
{

extern void InitializeDetectionFunctionMap();

ContactGraph::ContactGraph(World* world)
    : world{ world }
    , broadPhase{ this }
    , contactList{ nullptr }
    , contactCount{ 0 }
{
    InitializeDetectionFunctionMap();
}

ContactGraph::~ContactGraph()
{
    MuliAssert(contactList == nullptr);
}

void ContactGraph::EvaluateContacts()
{
    MuliProfileZoneNC(gather_active_contacts, "Gather Active", color::random(109817), true);

    // 1. Gather all active contacts to a flat buffer sequentially to preserve deterministic sequence.
    // Active contacts are those where at least one rigid body is awake and not static.
    LinearAllocator& allocator = world->linearAllocator;

    int32 size = contactCount * sizeof(Contact*);
    Contact** activeContacts = (Contact**)allocator.Allocate(size);
    int32 activeCount = 0;

    Contact* c = contactList;
    while (c)
    {
        RigidBody* bodyA = c->GetBodyA();
        RigidBody* bodyB = c->GetBodyB();

        bool activeA = bodyA->IsSleeping() == false && bodyA->GetType() != RigidBody::static_body;
        bool activeB = bodyB->IsSleeping() == false && bodyB->GetType() != RigidBody::static_body;

        if (activeA || activeB)
        {
            activeContacts[activeCount++] = c;
        }

        c = c->next;
    }
    MuliProfileZoneEnd(gather_active_contacts);

    // 2. Parallel Stage: Update manifolds in parallel.
    // The broad-phase overlap test (AABB query) and narrow-phase collision math (manifold calculations)
    // are strictly thread-safe as they read from body transforms and write only to their own Contact instances.
    ParallelFor(0, activeCount, [this, &activeContacts](int32 i) {
        MuliProfileZoneNC(narrow_phase_collision, "Collide", color::random(4567), true);
        Contact* contact = activeContacts[i];

        // Perform broad phase overlap test.
        if (broadPhase.TestOverlap(contact->colliderA, contact->colliderB) == false)
        {
            contact->flag |= Contact::flag_disjoint;
            MuliProfileZoneEnd(narrow_phase_collision);
            return;
        }

        // Compute contact manifold and warm starting impulses.
        contact->Update();
        MuliProfileZoneEnd(narrow_phase_collision);
    });

    MuliProfileZoneNC(post_narrow_phase, "Post Narrow Phase", color::random(945378), true);

    // 3. Serial Stage: Integrate states, execute user callbacks, and destroy disjoint contacts.
    // Sequential execution on the main thread guarantees deterministic order of events.
    for (int32 i = 0; i < activeCount; ++i)
    {
        Contact* contact = activeContacts[i];

        if ((contact->flag & Contact::flag_disjoint) != 0)
        {
            contact->flag &= ~Contact::flag_disjoint;
            Destroy(contact);
        }
        else
        {
            // Trigger contact begin/end/touching listener callbacks sequentially.
            contact->TriggerCallbacks();
        }
    }
    allocator.Free(activeContacts, size);
    MuliProfileZoneEnd(post_narrow_phase);
}

void ContactGraph::OnNewContact(Collider* colliderA, Collider* colliderB)
{
    RigidBody* bodyA = colliderA->body;
    RigidBody* bodyB = colliderB->body;

    MuliAssert(bodyA != bodyB);
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    if (bodyA->GetType() != RigidBody::dynamic_body && bodyB->GetType() != RigidBody::dynamic_body)
    {
        return;
    }

    if (EvaluateFilter(colliderA->GetFilter(), colliderB->GetFilter()) == false)
    {
        return;
    }

    ContactEdge* e = bodyB->contactList;
    while (e)
    {
        if (e->other == bodyA)
        {
            Collider* ceA = e->contact->colliderA;
            Collider* ceB = e->contact->colliderB;

            if ((colliderA == ceA && colliderB == ceB) || (colliderA == ceB && colliderB == ceA))
            {
                return;
            }
        }

        e = e->next;
    }

    void* mem = world->blockAllocator.Allocate(sizeof(Contact));
    Contact* c = new (mem) Contact(colliderA, colliderB);

    c->prev = nullptr;
    c->next = contactList;
    if (contactList != nullptr)
    {
        contactList->prev = c;
    }
    contactList = c;

    c->nodeA.contact = c;
    c->nodeA.other = bodyB;
    c->nodeA.prev = nullptr;
    c->nodeA.next = bodyA->contactList;
    if (bodyA->contactList != nullptr)
    {
        bodyA->contactList->prev = &c->nodeA;
    }
    bodyA->contactList = &c->nodeA;

    c->nodeB.contact = c;
    c->nodeB.other = bodyA;
    c->nodeB.prev = nullptr;
    c->nodeB.next = bodyB->contactList;
    if (bodyB->contactList != nullptr)
    {
        bodyB->contactList->prev = &c->nodeB;
    }
    bodyB->contactList = &c->nodeB;

    ++contactCount;
}

void ContactGraph::Destroy(Contact* c)
{
    RigidBody* bodyA = c->GetBodyA();
    RigidBody* bodyB = c->GetBodyB();

    if (c->prev) c->prev->next = c->next;
    if (c->next) c->next->prev = c->prev;
    if (c == contactList) contactList = c->next;

    if (c->nodeA.prev) c->nodeA.prev->next = c->nodeA.next;
    if (c->nodeA.next) c->nodeA.next->prev = c->nodeA.prev;
    if (&c->nodeA == bodyA->contactList) bodyA->contactList = c->nodeA.next;

    if (c->nodeB.prev) c->nodeB.prev->next = c->nodeB.next;
    if (c->nodeB.next) c->nodeB.next->prev = c->nodeB.prev;
    if (&c->nodeB == bodyB->contactList) bodyB->contactList = c->nodeB.next;

    c->~Contact();
    world->blockAllocator.Free(c, sizeof(Contact));
    --contactCount;
}

void ContactGraph::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

void ContactGraph::RemoveCollider(Collider* collider)
{
    broadPhase.Remove(collider);
    collider->node = AABBTree::nullNode;

    RigidBody* body = collider->body;
    ContactEdge* edge = body->contactList;
    while (edge)
    {
        Contact* contact = edge->contact;
        edge = edge->next;

        Collider* colliderA = contact->GetColliderA();
        Collider* colliderB = contact->GetColliderB();

        if (collider == colliderA || collider == colliderB)
        {
            Destroy(contact);
            colliderA->body->Awake();
            colliderB->body->Awake();
        }
    }
}

void ContactGraph::UpdateCollider(Collider* collider, const Transform& transform)
{
    AABB aabb;
    collider->GetShape()->ComputeAABB(transform, &aabb);
    broadPhase.Update(collider, aabb, Vec3::zero);
}

void ContactGraph::UpdateCollider(Collider* collider, const Transform& transform0, const Transform& transform1)
{
    AABB aabb0;
    AABB aabb1;
    collider->GetShape()->ComputeAABB(transform0, &aabb0);
    collider->GetShape()->ComputeAABB(transform1, &aabb1);

    Vec3 prediction = aabb1.GetCenter() - aabb0.GetCenter();
    aabb1.min += prediction;
    aabb1.max += prediction;

    broadPhase.Update(collider, AABB::Union(aabb0, aabb1), prediction);
}

} // namespace muli3
