#include "muli3/world.h"
#include "muli3/box.h"
#include "muli3/capsule.h"
#include "muli3/island.h"
#include "muli3/sphere.h"

namespace muli3
{

World::World(const WorldSettings& settings)
    : settings{ settings }
    , contactGraph{ this }
{
}

World::~World()
{
    Reset();
}

void World::Reset()
{
    while (bodyList)
    {
        Destroy(bodyList);
    }

    MuliAssert(bodyList == nullptr);
    MuliAssert(bodyListTail == nullptr);
    MuliAssert(bodyCount == 0);
    MuliAssert(contactGraph.contactList == nullptr);
    MuliAssert(contactGraph.contactCount == 0);

    destroyBodyBuffer.clear();
}

RigidBody* World::CreateEmptyBody(const Transform& transform, RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* b = new (mem) RigidBody(transform, type);

    b->world = this;
    b->prev = bodyListTail;
    b->next = nullptr;
    b->node = AABBTree::nullNode;
    b->contactList = nullptr;
    b->shape = nullptr;
    b->flag &= ~RigidBody::flag_island;
    b->flag |= RigidBody::flag_enabled;

    if (bodyListTail)
    {
        bodyListTail->next = b;
    }
    else
    {
        bodyList = b;
    }
    bodyListTail = b;
    ++bodyCount;

    return b;
}

RigidBody* World::CreateSphere(float radius, const Transform& transform, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateSphereShape(radius, identity, density);
    return b;
}

RigidBody* World::CreateCapsule(float height, float radius, const Transform& transform, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateCapsuleShape(height, radius, identity, density);
    return b;
}

RigidBody* World::CreateBox(
    float width, float height, float depth, const Transform& transform, RigidBody::Type type, float radius, float density
)
{
    RigidBody* b = CreateEmptyBody(transform, type);
    b->CreateBoxShape(width, height, depth, identity, radius, density);
    return b;
}

RigidBody* World::CreateBox(const Vec3& size, const Transform& transform, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size.x, size.y, size.z, transform, type, radius, density);
}

RigidBody* World::CreateBox(float size, const Transform& transform, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, size, transform, type, radius, density);
}

float World::Step(float dt)
{
    settings.step.dt = dt;
    settings.step.inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    if (settings.step.inv_dt == 0.0f)
    {
        return 0.0f;
    }

    linearAllocator.GrowMemory();

    {
        // Update broad-phase contact graph
        contactGraph.UpdateContactGraph();

        // Narrow-phase
        contactGraph.EvaluateContacts();

        Solve();
    }

    for (RigidBody* body : destroyBodyBuffer)
    {
        if (body && body->world == this)
        {
            Destroy(body);
        }
    }

    destroyBodyBuffer.clear();

    return 1.0f;
}

void World::Destroy(RigidBody* body)
{
    if (body == nullptr)
    {
        return;
    }

    MuliAssert(body->world == this);

    body->DestroyShape();
    contactGraph.RemoveBody(body);

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;
    --bodyCount;

    FreeBody(body);
}

void World::Destroy(std::span<RigidBody*> bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (destroyed.find(b) != destroyed.end())
        {
            destroyed.insert(b);
            Destroy(b);
        }
    }
}

void World::BufferDestroy(RigidBody* body)
{
    MuliAssert(body != nullptr);
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<RigidBody*> bodies)
{
    for (RigidBody* body : bodies)
    {
        BufferDestroy(body);
    }
}

void World::Solve()
{
    int32 bodyCount = GetBodyCount();
    if (bodyCount == 0)
    {
        return;
    }

    Island island{ this, bodyCount, contactGraph.contactCount };

    int32 restingBodies = 0;
    int32 islandID = 0;
    sleepingBodyCount = 0;

    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    int32 stackPointer;

    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->flag & RigidBody::flag_island)
        {
            continue;
        }

        if (b->IsSleeping())
        {
            ++sleepingBodyCount;
            continue;
        }

        if (b->IsStatic())
        {
            continue;
        }

        if (b->IsEnabled() == false)
        {
            continue;
        }

        stackPointer = 0;
        stack[stackPointer++] = b;
        b->flag |= RigidBody::flag_island;

        ++islandID;
        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            island.Add(t);
            t->islandID = islandID;

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                Contact* c = ce->contact;

                if (c->flag & Contact::flag_island)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_touching) == 0)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_enabled) == 0)
                {
                    continue;
                }

                island.Add(c);
                c->flag |= Contact::flag_island;

                RigidBody* other = ce->other;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->IsStatic())
                {
                    continue;
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            if (t->resting > settings.sleeping_time)
            {
                ++restingBodies;
            }
        }

        island.sleeping = settings.sleeping && (restingBodies == island.bodyCount);
        island.Solve();

        island.Clear();
        restingBodies = 0;
    }

    linearAllocator.Free(stack, bodyCount * sizeof(RigidBody*));
    islandCount = islandID;

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        if ((body->flag & RigidBody::flag_island) == 0)
        {
            continue;
        }

        MuliAssert(body->IsStatic() == false);

        body->flag &= ~RigidBody::flag_island;
        Transform transform0;
        body->motion.GetTransform(0.0f, &transform0);
        body->SynchronizeTransform();
        contactGraph.UpdateBody(body, transform0, body->transform);
    }

    for (Contact* contact = contactGraph.contactList; contact; contact = contact->next)
    {
        contact->flag &= ~Contact::flag_island;
    }

    MuliNotUsed(islandID);
}

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();
    blockAllocator.Free(body, sizeof(RigidBody));
}

Shape* World::CloneShape(const Shape* shape, const Transform& transform)
{
    if (shape == nullptr)
    {
        return nullptr;
    }

    switch (shape->GetType())
    {
    case Shape::sphere:
    {
        void* mem = blockAllocator.Allocate(sizeof(Sphere));
        return new (mem) Sphere(*(const Sphere*)shape, transform);
    }
    case Shape::capsule:
    {
        void* mem = blockAllocator.Allocate(sizeof(Capsule));
        return new (mem) Capsule(*(const Capsule*)shape, transform);
    }
    case Shape::box:
    {
        void* mem = blockAllocator.Allocate(sizeof(Box));
        return new (mem) Box(*(const Box*)shape, transform);
    }
    default:
        MuliAssert(false);
        break;
    }

    return nullptr;
}

void World::FreeShape(Shape* shape)
{
    switch (shape->GetType())
    {
    case Shape::sphere:
        ((Sphere*)shape)->~Sphere();
        blockAllocator.Free(shape, sizeof(Sphere));
        break;
    case Shape::capsule:
        ((Capsule*)shape)->~Capsule();
        blockAllocator.Free(shape, sizeof(Capsule));
        break;
    case Shape::box:
        ((Box*)shape)->~Box();
        blockAllocator.Free(shape, sizeof(Box));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

} // namespace muli3
