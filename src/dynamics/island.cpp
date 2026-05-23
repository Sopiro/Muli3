#include "muli3/island.h"
#include "muli3/world.h"

namespace muli3
{

// https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf
// Erin Catto's numerical method for stable gyroscopic torque integration
static Vec3 SolveGyroscopic(const Quat& q, const Mat3& inertia, const Vec3& w, float h)
{
    // Convert to body frame
    Vec3 localW = q.RotateInv(w);
    Vec3 localL = inertia * localW;

    // Residual vector
    Vec3 f = h * Cross(localW, localL);
    Mat3 gyro = Skew(localW) * inertia - Skew(localL);

    // Jacobian
    Mat3 j = inertia + Mat3{ gyro.ex * h, gyro.ey * h, gyro.ez * h };

    // Single Newton-Raphson update
    localW -= j.GetInverse() * f;

    // Back to world frame
    return q.Rotate(localW);
}

void Island::Solve(World* world)
{
    MuliProfileZoneNC(solve_island, "Island::Solve", color::solve, true);

    bool awakeIsland = false;

    const WorldSettings& settings = world->settings;
    const Timestep& step = settings.step;

    {
        MuliProfileZoneNC(integrate_velocity, "Integrate Velocities", color::integrate_velocities, true);

        // Integrate velocities, yield tentative velocities that possibly violate the constraint
        for (int32 i = 0; i < bodyCount; ++i)
        {
            RigidBody* b = bodies[i];
            b->motion.c0 = b->motion.c;
            b->motion.q0 = b->motion.q;
            b->motion.alpha0 = 0.0f;

            if (sleeping)
            {
                b->islandID = 0;
                b->islandIndex = 0;
                b->linearVelocity = Vec3::zero;
                b->angularVelocity = Vec3::zero;
                b->flag |= RigidBody::flag_sleeping;
            }
            else
            {
                b->flag &= ~RigidBody::flag_sleeping;
            }

            if (Length2(b->angularVelocity) > settings.rest_angular_tolerance ||
                Length2(b->linearVelocity) > settings.rest_linear_tolerance || Length2(b->torque) > 0.0f ||
                Length2(b->force) > 0.0f)
            {
                MuliAssert(sleeping == false);
                awakeIsland = true;
            }
            else
            {
                b->resting += step.dt;
            }

            if (b->GetType() == RigidBody::dynamic_body)
            {
                if (settings.apply_gravity)
                {
                    b->linearVelocity += settings.gravity * step.dt;
                }

                // Integrate velocites
                b->linearVelocity += b->force * b->invMass * step.dt;
                b->angularVelocity += b->GetWorldInverseInertiaTensor() * b->torque * step.dt;

                // Apply the w x (I * w) term
                if (b->GetGyroscopicTorqueEnabled())
                {
                    b->angularVelocity = SolveGyroscopic(b->motion.q, b->inertia, b->angularVelocity, step.dt);
                }

                // Apply damping
                b->linearVelocity *= 1.0f / (1.0f + b->linearDamping * step.dt);
                b->angularVelocity *= 1.0f / (1.0f + b->angularDamping * step.dt);
            }
        }

        MuliProfileZoneEnd(integrate_velocity);
    }

    {
        MuliProfileZoneNC(prepare_constraints, "Prepare Constraints", color::prepare_constraints, true);

        // Prepare constraints for solving step
        for (int32 i = 0; i < contactCount; ++i)
        {
            contacts[i]->Prepare(step);
        }
        for (int32 i = 0; i < jointCount; ++i)
        {
            joints[i]->Prepare(step);
        }

        MuliProfileZoneEnd(prepare_constraints);
    }

    {
        MuliProfileZoneNC(solve_velocity, "Solve Velocity", color::solve_velocity, true);

        // Iteratively solve the violated velocity constraints
        // Solving contacts backward converges fast
        for (int32 i = 0; i < step.velocity_iterations; ++i)
        {
            for (int32 j = contactCount; j > 0; --j)
            {
                contacts[j - 1]->SolveVelocityConstraints(step);
            }
            for (int32 j = jointCount; j > 0; --j)
            {
                joints[j - 1]->SolveVelocityConstraints(step);
            }
        }

        MuliProfileZoneEnd(solve_velocity);
    }

    {
        MuliProfileZoneNC(integrate_position, "Integrate Positions", color::integrate_positions, true);

        // Update positions using corrected velocities (Semi-implicit euler integration)
        for (int32 i = 0; i < bodyCount; ++i)
        {
            RigidBody* b = bodies[i];

            if (awakeIsland)
            {
                b->Awake();
            }

            b->force = Vec3::zero;
            b->torque = Vec3::zero;

            // Integrate position and orientation
            b->motion.c += b->linearVelocity * step.dt;

            Quat w{ b->angularVelocity, 0.0f };
            b->motion.q = b->motion.q + (w * b->motion.q) * step.dt * 0.5f;
            b->motion.q.Normalize();

            if (settings.world_bounds.TestPoint(b->transform.p) == false)
            {
                world->BufferDestroy(b);
            }
        }

        MuliProfileZoneEnd(integrate_position);
    }

    {
        MuliProfileZoneNC(solve_position, "Solve Position", color::solve_position, true);

        // Solve position constraints
        for (int32 i = 0; i < step.position_iterations; ++i)
        {
            bool contactSolved = true;
            bool jointSolved = true;

            for (int32 j = contactCount; j > 0; j--)
            {
                Contact* c = contacts[j - 1];

                bool solved = c->SolvePositionConstraints(step);
                if (solved == false)
                {
                    c->b1->Awake();
                    c->b2->Awake();
                }

                contactSolved &= solved;
            }

            for (int32 j = jointCount; j > 0; j--)
            {
                jointSolved &= joints[j - 1]->SolvePositionConstraints(step);
            }

            if (contactSolved && jointSolved)
            {
                break;
            }
        }

        MuliProfileZoneEnd(solve_position);
    }

    MuliProfileZoneEnd(solve_island);
}

} // namespace muli3
