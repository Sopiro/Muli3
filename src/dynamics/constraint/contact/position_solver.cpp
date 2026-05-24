#include "muli3/position_solver.h"
#include "muli3/contact.h"

namespace muli3
{

void PositionSolver::Prepare(ContactState* cs, int32 index)
{
    BodyState* sA = cs->s1;
    BodyState* sB = cs->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    localPlanePoint = qA.RotateInv(cs->manifold.referencePoint.p - comA);
    localClipPoint = qB.RotateInv(cs->manifold.contactPoints[index].p - comB);
    localNormal = qA.RotateInv(cs->manifold.contactNormal);
}

bool PositionSolver::Solve(ContactState* cs)
{
    BodyState* sA = cs->s1;
    BodyState* sB = cs->s2;

    Vec3 comA = sA->motion.c;
    Vec3 comB = sB->motion.c;
    Quat qA = sA->motion.q;
    Quat qB = sB->motion.q;

    Vec3 planePoint = qA.Rotate(localPlanePoint) + comA;
    Vec3 clipPoint = qB.Rotate(localClipPoint) + comB;
    Vec3 normal = qA.Rotate(localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec3 ra = clipPoint - comA;
    Vec3 rb = clipPoint - comB;

    Vec3 ran = Cross(ra, normal);
    Vec3 rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k
    float k = sA->invMass
            + Dot(ran, cs->invIA * ran)
            + sB->invMass
            + Dot(rbn, cs->invIB * rbn);
    // clang-format on

    // Constraint (bias)
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec3 impulse = normal * lambda;

    cs->cLinearImpulseA -= impulse;
    cs->cAngularImpulseA -= Cross(ra, impulse);
    cs->cLinearImpulseB += impulse;
    cs->cAngularImpulseB += Cross(rb, impulse);

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

} // namespace muli3
