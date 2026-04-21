#include <muli3/world.h>

namespace muli3
{

namespace
{

const Sphere* AsSphere(const RigidBody& body)
{
    if (!body.shape || body.shape->GetType() != Shape::sphere)
    {
        return nullptr;
    }

    return (const Sphere*)body.shape;
}

} // namespace

World::World(const WorldSettings& settings)
    : settings{ settings }
{
}

void World::Reset()
{
    bodies.clear();
    shapes.clear();
    debugContacts.clear();
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
    debugContacts.clear();

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
    for (int32 iteration = 0; iteration < settings.step.velocity_iterations; ++iteration)
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies.size(); ++j)
            {
                SolveSphereContact(*bodies[i], *bodies[j], dt, iteration == 0);
            }
        }
    }
}

void World::SolveSphereContact(RigidBody& a, RigidBody& b, float dt, bool recordDebugContact)
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

    float inverseMassSum = a.inverseMass + b.inverseMass;
    if (inverseMassSum <= epsilon)
    {
        return;
    }

    float penetration = radiusSum - distance;
    Vec3 correction = normal * (penetration / inverseMassSum);
    if (!a.IsStatic())
    {
        a.transform.position -= correction * a.inverseMass;
    }
    if (!b.IsStatic())
    {
        b.transform.position += correction * b.inverseMass;
    }

    Vec3 pointA = a.transform.position + normal * sphereA->GetRadius();
    Vec3 pointB = b.transform.position - normal * sphereB->GetRadius();
    Vec3 contactPoint = Lerp(pointA, pointB, 0.5f);
    if (recordDebugContact)
    {
        debugContacts.push_back(DebugContact{ contactPoint, normal, penetration });
    }

    Vec3 centerOfMassA = a.GetWorldCenterOfMass();
    Vec3 centerOfMassB = b.GetWorldCenterOfMass();
    Vec3 ra = contactPoint - centerOfMassA;
    Vec3 rb = contactPoint - centerOfMassB;

    Vec3 velocityA = a.GetVelocityAtWorldPoint(contactPoint);
    Vec3 velocityB = b.GetVelocityAtWorldPoint(contactPoint);
    Vec3 relativeVelocity = velocityB - velocityA;
    float velocityAlongNormal = Dot(relativeVelocity, normal);
    if (velocityAlongNormal > 0.0f)
    {
        return;
    }

    float restitution = Min(a.restitution, b.restitution);
    Mat3 inverseInertiaA = a.GetInverseInertiaTensorWorld();
    Mat3 inverseInertiaB = b.GetInverseInertiaTensorWorld();

    Vec3 angularA = Cross(inverseInertiaA * Cross(ra, normal), ra);
    Vec3 angularB = Cross(inverseInertiaB * Cross(rb, normal), rb);
    float normalMass = inverseMassSum + Dot(angularA + angularB, normal);
    if (normalMass <= epsilon)
    {
        return;
    }

    float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / normalMass;
    Vec3 impulse = normal * impulseMagnitude;
    a.ApplyImpulse(contactPoint, -impulse);
    b.ApplyImpulse(contactPoint, impulse);

    Vec3 postVelocityA = a.GetVelocityAtWorldPoint(contactPoint);
    Vec3 postVelocityB = b.GetVelocityAtWorldPoint(contactPoint);
    Vec3 tangent = postVelocityB - postVelocityA - normal * Dot(postVelocityB - postVelocityA, normal);
    float tangentLength = tangent.Length();
    if (tangentLength <= epsilon)
    {
        return;
    }

    tangent /= tangentLength;
    Vec3 tangentAngularA = Cross(inverseInertiaA * Cross(ra, tangent), ra);
    Vec3 tangentAngularB = Cross(inverseInertiaB * Cross(rb, tangent), rb);
    float tangentMass = inverseMassSum + Dot(tangentAngularA + tangentAngularB, tangent);
    if (tangentMass <= epsilon)
    {
        return;
    }

    float frictionMagnitude = -Dot(postVelocityB - postVelocityA, tangent) / tangentMass;
    float frictionLimit = impulseMagnitude * SafeSqrt(a.friction * b.friction);
    frictionMagnitude = Clamp(frictionMagnitude, -frictionLimit, frictionLimit);
    Vec3 frictionImpulse = tangent * frictionMagnitude;
    a.ApplyImpulse(contactPoint, -frictionImpulse);
    b.ApplyImpulse(contactPoint, frictionImpulse);
}

} // namespace muli3
