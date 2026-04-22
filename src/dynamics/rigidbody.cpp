#include "muli3/rigidbody.h"
#include "muli3/shape.h"

namespace muli3
{

Vec3 RigidBody::GetWorldCenterOfMass() const
{
    if (!shape)
    {
        return transform.p;
    }

    return transform.p + transform.q.Rotate(shape->GetCenterOfMass());
}

Mat3 RigidBody::GetInverseInertiaTensorLocal() const
{
    if (!shape || IsStatic())
    {
        return Mat3(0.0f);
    }

    const Mat3 localInertia = shape->ComputeLocalInertiaTensor(GetMass());
    return localInertia.GetInverse();
}

Mat3 RigidBody::GetInverseInertiaTensorWorld() const
{
    if (!shape || IsStatic())
    {
        return Mat3(0.0f);
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

    linearVelocity += impulse * invMass;
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
