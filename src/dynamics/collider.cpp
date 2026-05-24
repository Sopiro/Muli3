#include "muli3/collider.h"
#include "muli3/aabb_tree.h"
#include "muli3/callbacks.h"
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

void Collider::Create(RigidBody* inBody, Shape* inShape, const Transform& transform, float inDensity, const Material& inMaterial)
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

} // namespace muli3
