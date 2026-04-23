#include "muli3/world.h"
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

    while (shapes.empty() == false)
    {
        Destroy(shapes.back());
    }

    MuliAssert(bodyList == nullptr);
    MuliAssert(bodyListTail == nullptr);
    MuliAssert(bodies.empty());
    MuliAssert(shapes.empty());
    MuliAssert(contactGraph.contactList == nullptr);
    MuliAssert(contactGraph.contactCount == 0);

    bodies.clear();
    shapes.clear();
    destroyBodyBuffer.clear();
    destroyShapeBuffer.clear();
}

Shape* World::CreateSphereShape(float radius)
{
    void* mem = blockAllocator.Allocate(sizeof(Sphere));
    Shape* shape = new (mem) Sphere(radius);
    shapes.push_back(shape);
    return shape;
}

RigidBody* World::CreateRigidBody(const RigidBody& body)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* b = new (mem) RigidBody(body);
    bodies.push_back(b);

    b->world = this;
    b->prev = bodyListTail;
    b->next = nullptr;
    b->node = AABBTree::nullNode;
    b->contactList = nullptr;
    b->transform0 = b->transform;
    b->ownShape = false;
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

    contactGraph.AddBody(b);
    return b;
}

RigidBody* World::CreateSphere(float radius, const Transform& transform, bool isStatic, float mass)
{
    RigidBody body;
    body.shape = CreateSphereShape(radius);
    body.transform = transform;
    body.transform0 = transform;
    if (isStatic)
    {
        body.invMass = 0.0f;
    }
    else
    {
        body.SetMass(mass);
    }

    RigidBody* b = CreateRigidBody(body);
    b->ownShape = true;
    return b;
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

    contactGraph.UpdateContactGraph();
    contactGraph.EvaluateContacts();

    Solve();

    for (RigidBody* body : destroyBodyBuffer)
    {
        if (std::find(bodies.begin(), bodies.end(), body) != bodies.end())
        {
            Destroy(body);
        }
    }

    for (Shape* shape : destroyShapeBuffer)
    {
        if (std::find(shapes.begin(), shapes.end(), shape) != shapes.end())
        {
            Destroy(shape);
        }
    }

    destroyBodyBuffer.clear();
    destroyShapeBuffer.clear();

    return 1.0f;
}

void World::Destroy(RigidBody* body)
{
    if (body == nullptr)
    {
        return;
    }

    MuliAssert(body->world == this);
    if (body->world != this)
    {
        return;
    }

    contactGraph.RemoveBody(body);

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;

    auto it = std::find(bodies.begin(), bodies.end(), body);
    MuliAssert(it != bodies.end());
    if (it != bodies.end())
    {
        *it = bodies.back();
        bodies.pop_back();
    }

    Shape* shape = body->shape;
    bool destroyShape = body->ownShape;

    body->shape = nullptr;
    body->world = nullptr;
    body->prev = nullptr;
    body->next = nullptr;
    body->contactList = nullptr;
    body->node = AABBTree::nullNode;
    body->ownShape = false;

    FreeBody(body);

    if (destroyShape)
    {
        auto sit = std::find(destroyShapeBuffer.begin(), destroyShapeBuffer.end(), shape);
        if (sit != destroyShapeBuffer.end())
        {
            *sit = destroyShapeBuffer.back();
            destroyShapeBuffer.pop_back();
        }

        Destroy(shape);
    }
}

void World::Destroy(std::span<RigidBody*> inBodies)
{
    for (RigidBody* body : inBodies)
    {
        if (std::find(bodies.begin(), bodies.end(), body) != bodies.end())
        {
            Destroy(body);
        }
    }
}

void World::Destroy(Shape* shape)
{
    if (shape == nullptr)
    {
        return;
    }

    for (RigidBody* body : bodies)
    {
        MuliAssert(body->shape != shape);
        if (body->shape == shape)
        {
            return;
        }
    }

    auto it = std::find(shapes.begin(), shapes.end(), shape);
    MuliAssert(it != shapes.end());
    if (it == shapes.end())
    {
        return;
    }

    *it = shapes.back();
    shapes.pop_back();

    FreeShape(shape);
}

void World::Destroy(std::span<Shape*> inShapes)
{
    for (Shape* shape : inShapes)
    {
        if (std::find(shapes.begin(), shapes.end(), shape) != shapes.end())
        {
            Destroy(shape);
        }
    }
}

void World::BufferDestroy(RigidBody* body)
{
    if (body == nullptr)
    {
        return;
    }

    if (std::find(destroyBodyBuffer.begin(), destroyBodyBuffer.end(), body) == destroyBodyBuffer.end())
    {
        destroyBodyBuffer.push_back(body);
    }
}

void World::BufferDestroy(std::span<RigidBody*> inBodies)
{
    for (RigidBody* body : inBodies)
    {
        BufferDestroy(body);
    }
}

void World::BufferDestroy(Shape* shape)
{
    if (shape == nullptr)
    {
        return;
    }

    if (std::find(destroyShapeBuffer.begin(), destroyShapeBuffer.end(), shape) == destroyShapeBuffer.end())
    {
        destroyShapeBuffer.push_back(shape);
    }
}

void World::BufferDestroy(std::span<Shape*> inShapes)
{
    for (Shape* shape : inShapes)
    {
        BufferDestroy(shape);
    }
}

void World::Solve()
{
    int32 bodyCount = (int32)bodies.size();
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

    for (RigidBody* b : bodies)
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

    for (RigidBody* body : bodies)
    {
        if ((body->flag & RigidBody::flag_island) == 0)
        {
            continue;
        }

        MuliAssert(body->IsStatic() == false);

        body->flag &= ~RigidBody::flag_island;
        contactGraph.UpdateBody(body, body->transform0, body->transform);
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

void World::FreeShape(Shape* shape)
{
    switch (shape->GetType())
    {
    case ShapeType::sphere:
        ((Sphere*)shape)->~Sphere();
        blockAllocator.Free(shape, sizeof(Sphere));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

} // namespace muli3
