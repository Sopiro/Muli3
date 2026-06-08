#include "muli3/collider.h"
#include "muli3/aabb_tree.h"
#include "muli3/callbacks.h"
#include "muli3/contact.h"
#include "muli3/world.h"

namespace muli3
{

Collider::Collider()
    : OnDestroy{ nullptr }
    , ContactListener{ nullptr }
    , UserData{ nullptr }
    , body{ nullptr }
    , next{ nullptr }
    , shape{ nullptr }
    , density{ default_density }
    , material{ default_material }
    , filter{ default_collision_filter }
    , node{ AABBTree::nullNode }
    , enabled{ true }
{
}

Collider::~Collider()
{
    if (OnDestroy)
    {
        OnDestroy->OnColliderDestroy(this);
    }

    body = nullptr;
    next = nullptr;
}

void Collider::Create(Body* inBody, Shape* inShape, const Transform& transform, float inDensity, const Material& inMaterial)
{
    body = inBody;
    shape = body->world->CloneShape(inShape, transform);
    density = inDensity;
    material = inMaterial;
}

void Collider::Destroy(World* world)
{
    world->FreeShape(shape);
    shape = nullptr;
}

void Collider::SetEnabled(bool newEnabled)
{
    if (enabled == newEnabled)
    {
        return;
    }

    enabled = newEnabled;

    if (body->IsEnabled() == false)
    {
        return;
    }

    ConstraintGraph& graph = body->world->constraintGraph;
    if (enabled)
    {
        graph.AddCollider(this);
    }
    else
    {
        graph.RemoveCollider(this);
    }

    body->Awake();
}

void Collider::SetFilter(const CollisionFilter& newFilter)
{
    filter = newFilter;

    if (body->IsEnabled() == false || enabled == false)
    {
        return;
    }

    ConstraintGraph& graph = body->world->constraintGraph;
    ContactEdge* edge = body->contactList;
    while (edge)
    {
        Contact* contact = edge->contact;
        edge = edge->next;

        if (contact->GetColliderA() != this && contact->GetColliderB() != this)
        {
            continue;
        }

        bool touching = contact->IsTouching();
        bool contactEnabled = contact->IsEnabled();

        Body* bodyA = contact->GetBodyA();
        Body* bodyB = contact->GetBodyB();

        graph.Destroy(contact);

        if (touching && contactEnabled)
        {
            bodyA->Awake();
            bodyB->Awake();
        }
    }

    graph.broadPhase.Refresh(this);
    body->Awake();
}

} // namespace muli3
