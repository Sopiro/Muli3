#include "muli3/rigidbody.h"
#include "muli3/shape.h"
#include "muli3/sphere.h"
#include "muli3/world.h"

namespace muli3
{

void RigidBody::SetTransform(const Transform& newTransform)
{
    Transform oldTransform = transform;
    transform = newTransform;
    transform0 = transform;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetPosition(float x, float y, float z)
{
    Transform oldTransform = transform;
    transform.p = Vec3{ x, y, z };
    transform0 = transform;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetRotation(const Quat& rotation)
{
    Transform oldTransform = transform;
    transform.q = rotation;
    transform0 = transform;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetEnabled(bool enabled)
{
    if (enabled == IsEnabled())
    {
        return;
    }

    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return;
    }

    if (enabled)
    {
        flag |= flag_enabled;
        contactList = nullptr;
        world->contactGraph.AddBody(this);
    }
    else
    {
        flag &= ~flag_enabled;
        world->contactGraph.RemoveBody(this);
        islandID = 0;
        islandIndex = 0;
    }
}

void RigidBody::SetType(RigidBody::Type newType)
{
    if (type == newType)
    {
        return;
    }

    type = newType;

    force = Vec3::zero;
    torque = Vec3::zero;
    if (type != dynamic_body)
    {
        invMass = 0.0f;
    }
    else if (shape != nullptr && invMass <= epsilon)
    {
        MassData massData;
        shape->ComputeMass(default_density, &massData);
        SetMass(massData.mass);
    }

    if (type == static_body)
    {
        linearVelocity = Vec3::zero;
        angularVelocity = Vec3::zero;
        transform0 = transform;
    }

    Awake();

    if (shape != nullptr && world != nullptr)
    {
        world->contactGraph.RemoveBody(this);
        world->contactGraph.AddBody(this);
    }

    islandID = 0;
    islandIndex = 0;
}

Shape* RigidBody::CreateShape(Shape* newShape, const Transform& shapeTransform, float density)
{
    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return nullptr;
    }

    if (newShape == nullptr)
    {
        return nullptr;
    }

    DestroyShape();

    shape = world->CloneShape(newShape, shapeTransform);
    if (type == dynamic_body)
    {
        MassData massData;
        shape->ComputeMass(density, &massData);
        SetMass(massData.mass);
    }
    else
    {
        invMass = 0.0f;
    }

    if (IsEnabled())
    {
        world->contactGraph.AddBody(this);
    }

    Awake();

    return shape;
}

void RigidBody::DestroyShape()
{
    if (shape == nullptr)
    {
        return;
    }

    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return;
    }

    world->contactGraph.RemoveBody(this);

    Shape* oldShape = shape;

    shape = nullptr;
    invMass = 0.0f;

    world->FreeShape(oldShape);

    islandID = 0;
    islandIndex = 0;
    Awake();
}

Shape* RigidBody::CreateSphereShape(float radius, const Transform& shapeTransform, float density)
{
    Sphere sphere{ radius };
    return CreateShape(&sphere, shapeTransform, density);
}

Vec3 RigidBody::GetWorldCenterOfMass() const
{
    if (!shape)
    {
        return transform.p;
    }

    return transform.p + transform.q.Rotate(shape->GetCenterOfMass());
}

Mat3 RigidBody::GetInertiaTensorLocal() const
{
    if (!shape || type != dynamic_body)
    {
        return Mat3::zero;
    }

    return shape->ComputeLocalInertiaTensor(GetMass());
}

Mat3 RigidBody::GetInertiaTensorWorld() const
{
    if (!shape || type != dynamic_body)
    {
        return Mat3::zero;
    }

    const Mat3 rotation{ transform.q };
    return rotation * GetInertiaTensorLocal() * rotation.GetTranspose();
}

Mat3 RigidBody::GetInverseInertiaTensorLocal() const
{
    if (!shape || type != dynamic_body)
    {
        return Mat3::zero;
    }

    const Mat3 localInertia = shape->ComputeLocalInertiaTensor(GetMass());
    return localInertia.GetInverse();
}

Mat3 RigidBody::GetInverseInertiaTensorWorld() const
{
    if (!shape || type != dynamic_body)
    {
        return Mat3::zero;
    }

    const Mat3 rotation{ transform.q };
    return rotation * GetInverseInertiaTensorLocal() * rotation.GetTranspose();
}

void RigidBody::SetMass(float mass)
{
    invMass = mass <= epsilon ? 0.0f : 1.0f / mass;
}

void RigidBody::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    ApplyLinearImpulse(impulse);

    const Vec3 centerOfMass = GetWorldCenterOfMass();
    const Vec3 r = impulsePoint - centerOfMass;
    const Vec3 angularImpulse = Cross(r, impulse);
    ApplyAngularImpulse(angularImpulse);
}

void RigidBody::ApplyLinearImpulse(const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    linearVelocity += impulse * invMass;
}

void RigidBody::ApplyAngularImpulse(const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    angularVelocity += GetInverseInertiaTensorWorld() * impulse;
}

Vec3 RigidBody::GetVelocityAtWorldPoint(const Vec3& point) const
{
    const Vec3 centerOfMass = GetWorldCenterOfMass();
    const Vec3 r = point - centerOfMass;
    return linearVelocity + Cross(angularVelocity, r);
}

void RigidBody::Integrate(float dt)
{
    if (type == static_body)
    {
        return;
    }

    transform.p += linearVelocity * dt;

    const float angularSpeed = Length(angularVelocity);
    if (angularSpeed > epsilon)
    {
        const Vec3 axis = angularVelocity / angularSpeed;
        const Quat delta{ angularSpeed * dt, axis };
        transform.q = delta * transform.q;
        transform.q.Normalize();
    }
}

} // namespace muli3
