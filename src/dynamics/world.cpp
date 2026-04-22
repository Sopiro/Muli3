#include "muli3/world.h"
#include "muli3/sphere.h"

namespace muli3
{

World::World(const WorldSettings& settings)
    : settings{ settings }
{
}

void World::Reset()
{
    bodies.clear();
    shapes.clear();
    contacts.clear();
}

Shape* World::CreateSphereShape(float radius)
{
    shapes.push_back(std::make_unique<Sphere>(radius));
    return shapes.back().get();
}

RigidBody* World::CreateRigidBody(const RigidBody& body)
{
    bodies.push_back(std::make_unique<RigidBody>(body));
    return bodies.back().get();
}

RigidBody* World::CreateSphere(float radius, const Transform& transform, bool isStatic, float mass)
{
    RigidBody body;
    body.shape = CreateSphereShape(radius);
    body.transform = transform;
    if (isStatic)
    {
        body.invMass = 0.0f;
    }
    else
    {
        body.SetMass(mass);
    }

    return CreateRigidBody(body);
}

void World::Step(float dt)
{
    settings.step.dt = dt;
    settings.step.inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    contacts.clear();

    IntegrateBodies();
    FindContacts();
    SolveContacts();
}

void World::IntegrateBodies()
{
    for (std::unique_ptr<RigidBody>& body : bodies)
    {
        if (body->IsStatic())
        {
            continue;
        }

        if (settings.apply_gravity)
        {
            body->linearVelocity += settings.gravity * settings.step.dt;
        }

        body->Integrate(settings.step.dt);
    }
}

void World::FindContacts()
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            contacts.emplace_back(bodies[i].get(), bodies[j].get());
            if (contacts.back().Update() == false)
            {
                contacts.pop_back();
            }
        }
    }
}

void World::SolveContacts()
{
    for (int32 iteration = 0; iteration < settings.step.velocity_iterations; ++iteration)
    {
        for (Contact& contact : contacts)
        {
            contact.Solve(settings.step.inv_dt);
        }
    }
}

} // namespace muli3
