#include "muli3/contact_graph.h"
#include "muli3/world.h"

namespace muli3
{

ContactGraph::ContactGraph(World* world)
    : world{ world }
    , broadPhase{ this }
    , contactList{ nullptr }
    , contactCount{ 0 }
{
}

ContactGraph::~ContactGraph()
{
    MuliAssert(contactList == nullptr);
}

void ContactGraph::EvaluateContacts()
{
    Contact* c = contactList;
    while (c)
    {
        RigidBody* bodyA = c->bodyA;
        RigidBody* bodyB = c->bodyB;

        bool activeA = bodyA->IsSleeping() == false && bodyA->IsStatic() == false;
        bool activeB = bodyB->IsSleeping() == false && bodyB->IsStatic() == false;

        if (activeA == false && activeB == false)
        {
            c = c->next;
            continue;
        }

        bool overlap = broadPhase.TestOverlap(bodyA, bodyB);
        if (overlap == false)
        {
            Contact* t = c;
            c = c->next;
            Destroy(t);
            continue;
        }

        c->Update();
        c = c->next;
    }
}

void ContactGraph::OnNewContact(RigidBody* bodyA, RigidBody* bodyB)
{
    MuliAssert(bodyA != bodyB);
    MuliAssert(bodyA->shape && bodyB->shape);
    MuliAssert(bodyA->shape->GetType() >= bodyB->shape->GetType());

    if (bodyA->IsStatic() && bodyB->IsStatic())
    {
        return;
    }

    ContactEdge* e = bodyB->contactList;
    while (e)
    {
        if (e->other == bodyA)
        {
            RigidBody* ceA = e->contact->bodyA;
            RigidBody* ceB = e->contact->bodyB;

            if ((bodyA == ceA && bodyB == ceB) || (bodyA == ceB && bodyB == ceA))
            {
                return;
            }
        }

        e = e->next;
    }

    void* mem = world->blockAllocator.Allocate(sizeof(Contact));
    Contact* c = new (mem) Contact(bodyA, bodyB);

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
    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

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

void ContactGraph::AddBody(RigidBody* body)
{
    if (!body->shape)
    {
        return;
    }

    AABB aabb;
    body->shape->ComputeAABB(body->transform, &aabb);
    broadPhase.Add(body, aabb);
}

void ContactGraph::RemoveBody(RigidBody* body)
{
    if (body->node != AABBTree::nullNode)
    {
        broadPhase.Remove(body);
        body->node = AABBTree::nullNode;
    }

    ContactEdge* edge = body->contactList;
    while (edge)
    {
        Contact* contact = edge->contact;
        edge = edge->next;

        Destroy(contact);
    }
}

void ContactGraph::UpdateBody(RigidBody* body, const Transform& transform)
{
    if (!body->shape || body->node == AABBTree::nullNode)
    {
        return;
    }

    AABB aabb;
    body->shape->ComputeAABB(transform, &aabb);

    broadPhase.Update(body, aabb, Vec3::zero);
}

void ContactGraph::UpdateBody(RigidBody* body, const Transform& transform0, const Transform& transform1)
{
    if (!body->shape || body->node == AABBTree::nullNode)
    {
        return;
    }

    AABB aabb0;
    AABB aabb1;
    body->shape->ComputeAABB(transform0, &aabb0);
    body->shape->ComputeAABB(transform1, &aabb1);

    Vec3 prediction = aabb1.GetCenter() - aabb0.GetCenter();
    aabb1.min += prediction;
    aabb1.max += prediction;

    broadPhase.Update(body, AABB::Union(aabb0, aabb1), prediction);
}

} // namespace muli3

