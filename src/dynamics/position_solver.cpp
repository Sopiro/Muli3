#include "muli3/position_solver.h"
#include "muli3/contact.h"

namespace muli3
{

void PositionSolver::Prepare(Contact* inContact, int32 index)
{
    contact = inContact;

    Vec3 comA = contact->b1->motion.c;
    Vec3 comB = contact->b2->motion.c;
    Quat qA = contact->b1->motion.q;
    Quat qB = contact->b2->motion.q;

    localPlanePoint = qA.RotateInv(contact->manifold.referencePoint.p - comA);
    localClipPoint = qB.RotateInv(contact->manifold.contactPoints[index].p - comB);
    localNormal = qA.RotateInv(contact->manifold.contactNormal);
}

bool PositionSolver::Solve()
{
    Vec3 comA = contact->b1->motion.c;
    Vec3 comB = contact->b2->motion.c;
    Quat qA = contact->b1->motion.q;
    Quat qB = contact->b2->motion.q;

    Vec3 planePoint = qA.Rotate(localPlanePoint) + comA;
    Vec3 clipPoint = qB.Rotate(localClipPoint) + comB;
    Vec3 normal = qA.Rotate(localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec3 ra = clipPoint - comA;
    Vec3 rb = clipPoint - comB;

    Mat3 iiA = contact->b1->GetWorldInverseInertiaTensor();
    Mat3 iiB = contact->b2->GetWorldInverseInertiaTensor();

    Vec3 ran = Cross(ra, normal);
    Vec3 rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k
    float k = contact->b1->invMass
            + Dot(ran, iiA * ran)
            + contact->b2->invMass
            + Dot(rbn, iiB * rbn);
    // clang-format on

    // Constraint (bias)
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec3 impulse = normal * lambda;

    // Apply position correction
    contact->b1->motion.c -= impulse * contact->b1->invMass;
    // Apply angular correction for body A
    {
        Vec3 angularImpulse = iiA * Cross(ra, impulse);
        float angle = Length(angularImpulse);
        if (angle > epsilon)
        {
            Vec3 axis = angularImpulse / angle;
            Quat delta{ angle, axis };
            contact->b1->motion.q = delta * contact->b1->motion.q;
            contact->b1->motion.q.Normalize();
        }
    }

    contact->b2->motion.c += impulse * contact->b2->invMass;
    // Apply angular correction for body B
    {
        Vec3 angularImpulse = iiB * Cross(rb, impulse);
        float angle = Length(angularImpulse);
        if (angle > epsilon)
        {
            Vec3 axis = angularImpulse / angle;
            Quat delta{ angle, axis };
            contact->b2->motion.q = delta * contact->b2->motion.q;
            contact->b2->motion.q.Normalize();
        }
    }

    contact->b1->SynchronizeTransform();
    contact->b2->SynchronizeTransform();

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

} // namespace muli3
