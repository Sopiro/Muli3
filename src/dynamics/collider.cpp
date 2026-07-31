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
    , bodyIndex{ null_index }
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
    bodyIndex = null_index;
}

void Collider::Clone(Body* inBody, Shape* inShape, const Transform& transform, float inDensity, const Material& inMaterial)
{
    Create(inBody, inBody->world->CloneShape(inShape, transform), inDensity, inMaterial);
}

void Collider::Create(Body* inBody, Shape* inShape, float inDensity, const Material& inMaterial)
{
    body = inBody;
    shape = inShape;
    density = inDensity;
    material = inMaterial;
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
    for (int32 i = 0; i < int32(body->contacts.size());)
    {
        Contact* contact = body->contacts[i];

        if (contact->GetColliderA() != this && contact->GetColliderB() != this)
        {
            ++i;
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
