#include <muli3/rigidbody.h>

namespace muli3
{

Vec3 RigidBody::GetWorldCenterOfMass() const
{
    if (!shape)
    {
        return transform.position;
    }

    return transform.position + transform.rotation.Rotate(shape->GetCenterOfMass());
}

Mat3 RigidBody::GetInverseInertiaTensorLocal() const
{
    if (!shape || IsStatic())
    {
        return Mat3::Diagonal(0.0f, 0.0f, 0.0f);
    }

    const Mat3 localInertia = shape->ComputeLocalInertiaTensor(GetMass());
    return localInertia.Inversed();
}

Mat3 RigidBody::GetInverseInertiaTensorWorld() const
{
    if (!shape || IsStatic())
    {
        return Mat3::Diagonal(0.0f, 0.0f, 0.0f);
    }

    const Mat3 rotation{ transform.rotation };
    return rotation * GetInverseInertiaTensorLocal() * rotation.Transposed();
}

void RigidBody::SetMass(float mass)
{
    inverseMass = mass <= epsilon ? 0.0f : 1.0f / mass;
}

void RigidBody::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
    if (IsStatic())
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
    if (IsStatic())
    {
        return;
    }

    linearVelocity += impulse * inverseMass;
}

void RigidBody::ApplyAngularImpulse(const Vec3& impulse)
{
    if (IsStatic())
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
    if (IsStatic())
    {
        return;
    }

    transform.position += linearVelocity * dt;

    const float angularSpeed = angularVelocity.Length();
    if (angularSpeed > epsilon)
    {
        const Vec3 axis = angularVelocity / angularSpeed;
        const Quat delta{ angularSpeed * dt, axis };
        transform.rotation = Normalize(delta * transform.rotation);
    }
}

} // namespace muli3
