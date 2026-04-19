#include <muli3/world.h>

namespace muli3
{

namespace
{

std::shared_ptr<Shape> MakeSphereShape(float radius)
{
    return std::make_shared<Sphere>(radius);
}

const Sphere* AsSphere(const RigidBody& body)
{
    if (!body.shape || body.shape->GetType() != ShapeType::sphere)
    {
        return nullptr;
    }

    return (const Sphere*)body.shape.get();
}

} // namespace

World::World(const WorldSettings& settings)
    : settings{ settings }
{
}

void World::Reset()
{
    bodies.clear();
}

RigidBody* World::CreateRigidBody(const RigidBody& body)
{
    bodies.push_back(std::make_unique<RigidBody>(body));
    return bodies.back().get();
}

RigidBody* World::CreateSphere(float radius, const Transform& transform, bool isStatic, float mass)
{
    RigidBody body;
    body.shape = MakeSphereShape(radius);
    body.transform = transform;
    if (isStatic)
    {
        body.inverseMass = 0.0f;
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

    for (std::unique_ptr<RigidBody>& body : bodies)
    {
        if (body->IsStatic())
        {
            continue;
        }

        if (settings.apply_gravity)
        {
            body->linearVelocity += settings.gravity * dt;
        }
        body->Integrate(dt);
    }

    SolveContacts(dt);
}

void World::SolveContacts(float dt)
{
    for (int iteration = 0; iteration < settings.step.velocity_iterations; ++iteration)
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies.size(); ++j)
            {
                SolveSphereContact(*bodies[i], *bodies[j], dt);
            }
        }
    }
}

void World::SolveSphereContact(RigidBody& a, RigidBody& b, float dt)
{
    (void)dt;

    const Sphere* sphereA = AsSphere(a);
    const Sphere* sphereB = AsSphere(b);
    if (!sphereA || !sphereB)
    {
        return;
    }

    const Vec3 delta = b.transform.position - a.transform.position;
    const float distanceSquared = delta.LengthSquared();
    const float radiusSum = sphereA->GetRadius() + sphereB->GetRadius();
    if (distanceSquared >= radiusSum * radiusSum)
    {
        return;
    }

    float distance = SafeSqrt(distanceSquared);
    Vec3 normal = distance > epsilon ? delta / distance : Vec3{ 1.0f, 0.0f, 0.0f };
    if (distance <= epsilon)
    {
        distance = radiusSum;
    }

    const float inverseMassSum = a.inverseMass + b.inverseMass;
    if (inverseMassSum <= epsilon)
    {
        return;
    }

    const float penetration = radiusSum - distance;
    const Vec3 correction = normal * (penetration / inverseMassSum);
    if (!a.IsStatic())
    {
        a.transform.position -= correction * a.inverseMass;
    }
    if (!b.IsStatic())
    {
        b.transform.position += correction * b.inverseMass;
    }

    const Vec3 relativeVelocity = b.linearVelocity - a.linearVelocity;
    const float velocityAlongNormal = Dot(relativeVelocity, normal);
    if (velocityAlongNormal > 0.0f)
    {
        return;
    }

    const float restitution = Min(a.restitution, b.restitution);
    const float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / inverseMassSum;
    const Vec3 impulse = normal * impulseMagnitude;
    if (!a.IsStatic())
    {
        a.linearVelocity -= impulse * a.inverseMass;
    }
    if (!b.IsStatic())
    {
        b.linearVelocity += impulse * b.inverseMass;
    }

    Vec3 tangent = relativeVelocity - normal * velocityAlongNormal;
    const float tangentLength = tangent.Length();
    if (tangentLength <= epsilon)
    {
        return;
    }

    tangent /= tangentLength;
    float frictionMagnitude = -Dot(relativeVelocity, tangent) / inverseMassSum;
    const float frictionLimit = impulseMagnitude * SafeSqrt(a.friction * b.friction);
    frictionMagnitude = Clamp(frictionMagnitude, -frictionLimit, frictionLimit);
    const Vec3 frictionImpulse = tangent * frictionMagnitude;
    if (!a.IsStatic())
    {
        a.linearVelocity -= frictionImpulse * a.inverseMass;
    }
    if (!b.IsStatic())
    {
        b.linearVelocity += frictionImpulse * b.inverseMass;
    }
}

} // namespace muli3
