#include "muli3/contact.h"
#include "muli3/settings.h"

namespace muli3
{

bool Contact::Update()
{
    if (!bodyA->shape || !bodyB->shape)
    {
        return false;
    }

    return Collide(bodyA->shape, bodyA->transform, bodyB->shape, bodyB->transform, &manifold);
}

void Contact::Solve(float invDt)
{
    float invMassSum = bodyA->invMass + bodyB->invMass;
    if (invMassSum <= epsilon)
    {
        return;
    }

    Vec3 centerOfMassA = bodyA->GetWorldCenterOfMass();
    Vec3 centerOfMassB = bodyB->GetWorldCenterOfMass();
    Mat3 inverseInertiaA = bodyA->GetInverseInertiaTensorWorld();
    Mat3 inverseInertiaB = bodyB->GetInverseInertiaTensorWorld();

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        Vec3 point = manifold.contactPoints[i].p;
        Vec3 ra = point - centerOfMassA;
        Vec3 rb = point - centerOfMassB;

        Vec3 velocityA = bodyA->GetVelocityAtWorldPoint(point);
        Vec3 velocityB = bodyB->GetVelocityAtWorldPoint(point);
        Vec3 relativeVelocity = velocityB - velocityA;
        float velocityAlongNormal = Dot(relativeVelocity, manifold.contactNormal);

        float restitution = Min(bodyA->restitution, bodyB->restitution);
        float restitutionBias = velocityAlongNormal < 0.0f ? restitution * velocityAlongNormal : 0.0f;
        float positionBias =
            -position_correction * Clamp(manifold.penetrationDepth - linear_slop, 0.0f, max_position_correction) * invDt;
        float bias = restitutionBias + positionBias;

        Vec3 angularA = Cross(inverseInertiaA * Cross(ra, manifold.contactNormal), ra);
        Vec3 angularB = Cross(inverseInertiaB * Cross(rb, manifold.contactNormal), rb);
        float normalMass = invMassSum + Dot(angularA + angularB, manifold.contactNormal);
        if (normalMass <= epsilon)
        {
            continue;
        }

        float impulseMagnitude = -(velocityAlongNormal + bias) / normalMass;
        if (impulseMagnitude <= 0.0f)
        {
            continue;
        }

        Vec3 impulse = manifold.contactNormal * impulseMagnitude;
        bodyA->ApplyImpulse(point, -impulse);
        bodyB->ApplyImpulse(point, impulse);

        Vec3 postVelocityA = bodyA->GetVelocityAtWorldPoint(point);
        Vec3 postVelocityB = bodyB->GetVelocityAtWorldPoint(point);
        Vec3 tangent =
            postVelocityB - postVelocityA - manifold.contactNormal * Dot(postVelocityB - postVelocityA, manifold.contactNormal);
        float tangentLength = Length(tangent);
        if (tangentLength <= epsilon)
        {
            continue;
        }

        tangent /= tangentLength;
        Vec3 tangentAngularA = Cross(inverseInertiaA * Cross(ra, tangent), ra);
        Vec3 tangentAngularB = Cross(inverseInertiaB * Cross(rb, tangent), rb);
        float tangentMass = invMassSum + Dot(tangentAngularA + tangentAngularB, tangent);
        if (tangentMass <= epsilon)
        {
            continue;
        }

        float frictionMagnitude = -Dot(postVelocityB - postVelocityA, tangent) / tangentMass;
        float frictionLimit = impulseMagnitude * SafeSqrt(bodyA->friction * bodyB->friction);
        frictionMagnitude = Clamp(frictionMagnitude, -frictionLimit, frictionLimit);
        Vec3 frictionImpulse = tangent * frictionMagnitude;
        bodyA->ApplyImpulse(point, -frictionImpulse);
        bodyB->ApplyImpulse(point, frictionImpulse);
    }
}

} // namespace muli3
