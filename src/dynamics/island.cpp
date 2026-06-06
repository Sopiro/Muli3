#include "muli3/island.h"
#include "muli3/contact_solver.h"
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
            BodyState* s = bodies[i];
            RigidBody* b = s->body;
            s->motion.c0 = s->motion.c;
            s->motion.q0 = s->motion.q;
            s->motion.alpha0 = 0.0f;

            b->flag &= ~RigidBody::flag_sleeping;

            if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
                Length2(s->linearVelocity) > settings.rest_linear_tolerance || Length2(s->torque) > 0.0f ||
                Length2(s->force) > 0.0f)
            {
                awakeIsland = true;
            }

            if (b->GetType() == RigidBody::dynamic_body)
            {
                if (settings.apply_gravity)
                {
                    s->linearVelocity += settings.gravity * step.dt;
                }

                // Integrate velocites
                s->linearVelocity += s->force * s->invMass * step.dt;
                s->angularVelocity += b->GetWorldInverseInertiaTensor() * s->torque * step.dt;

                // Apply the w x (I * w) term
                if (b->GetGyroscopicTorqueEnabled())
                {
                    s->angularVelocity = SolveGyroscopic(s->motion.q, b->inertia, s->angularVelocity, step.dt);
                }

                // Apply damping
                s->linearVelocity *= 1.0f / (1.0f + s->linearDamping * step.dt);
                s->angularVelocity *= 1.0f / (1.0f + s->angularDamping * step.dt);
            }
        }

        MuliProfileZoneEnd(integrate_velocity);
    }

    {
        MuliProfileZoneNC(prepare_constraints, "Prepare Constraints", color::prepare_constraints, true);

        // Prepare constraints for solving step
        for (int32 i = 0; i < contactCount; ++i)
        {
            PrepareContact(contacts[i]);
        }
        for (int32 i = 0; i < jointCount; ++i)
        {
            PrepareJoint(joints[i], step);
        }

        MuliProfileZoneEnd(prepare_constraints);
    }

    {
        MuliProfileZoneNC(warm_start, "Warm Start", color::warm_start, true);

        // Prepare constraints for solving step
        for (int32 i = 0; i < contactCount; ++i)
        {
            WarmStartContact(contacts[i]);
        }
        for (int32 i = 0; i < jointCount; ++i)
        {
            WarmStartJoint(joints[i]);
        }

        MuliProfileZoneEnd(warm_start);
    }

    {
        MuliProfileZoneNC(solve_velocity, "Solve Velocity", color::solve_velocities, true);

        // Iteratively solve the violated velocity constraints
        // Solving contacts backward converges fast
        for (int32 i = 0; i < step.velocity_iterations; ++i)
        {
            for (int32 j = contactCount; j > 0; --j)
            {
                SolveContactVelocityConstraints(contacts[j - 1]);
            }
            for (int32 j = jointCount; j > 0; --j)
            {
                SolveJointVelocityConstraints(joints[j - 1], step);
            }
        }

        MuliProfileZoneEnd(solve_velocity);
    }

    {
        MuliProfileZoneNC(integrate_position, "Integrate Positions", color::integrate_positions, true);

        // Update positions using corrected velocities (Semi-implicit euler integration)
        for (int32 i = 0; i < bodyCount; ++i)
        {
            BodyState* s = bodies[i];

            s->force = Vec3::zero;
            s->torque = Vec3::zero;

            // Integrate position and orientation
            s->motion.c += s->linearVelocity * step.dt;

            Quat w{ s->angularVelocity, 0.0f };
            s->motion.q = s->motion.q + (w * s->motion.q) * step.dt * 0.5f;
            s->motion.q.Normalize();
        }

        MuliProfileZoneEnd(integrate_position);
    }

    {
        MuliProfileZoneNC(solve_position, "Solve Position", color::solve_positions, true);

        // Solve position constraints
        for (int32 i = 0; i < step.position_iterations; ++i)
        {
            bool contactSolved = true;
            bool jointSolved = true;

            for (int32 j = contactCount; j > 0; j--)
            {
                ContactState* s = contacts[j - 1];
                bool solved = SolveContactPositionConstraints(s);
                if (solved == false)
                {
                    s->s1->resting = 0.0f;
                    s->s2->resting = 0.0f;
                }

                contactSolved &= solved;
            }

            for (int32 j = jointCount; j > 0; j--)
            {
                JointState* s = joints[j - 1];
                bool solved = SolveJointPositionConstraints(s, step);
                if (solved == false)
                {
                    Joint* joint = s->joint;
                    joint->GetBodyA()->GetBodyState()->resting = 0.0f;
                    joint->GetBodyB()->GetBodyState()->resting = 0.0f;
                }

                jointSolved &= solved;
            }

            if (contactSolved && jointSolved)
            {
                break;
            }
        }

        MuliProfileZoneEnd(solve_position);
    }

    for (int32 i = 0; i < bodyCount; ++i)
    {
        BodyState* s = bodies[i];
        if (Length2(s->angularVelocity) > settings.rest_angular_tolerance ||
            Length2(s->linearVelocity) > settings.rest_linear_tolerance)
        {
            awakeIsland = true;
        }
    }

    bool sleeping = false;

    if (awakeIsland)
    {
        for (int32 i = 0; i < bodyCount; ++i)
        {
            bodies[i]->resting = 0.0f;
        }
    }
    else
    {
        sleeping = settings.sleeping;

        for (int32 i = 0; i < bodyCount; ++i)
        {
            BodyState* s = bodies[i];
            s->resting += step.dt;
            sleeping &= s->resting > settings.sleeping_time;
        }
    }

    if (sleeping == false)
    {
        for (int32 i = 0; i < bodyCount; ++i)
        {
            bodies[i]->body->flag &= ~RigidBody::flag_sleeping;
        }
    }
    else
    {
        for (int32 j = 0; j < bodyCount; ++j)
        {
            BodyState* s = bodies[j];
            RigidBody* body = s->body;

            s->force = Vec3::zero;
            s->torque = Vec3::zero;
            s->linearVelocity = Vec3::zero;
            s->angularVelocity = Vec3::zero;
            s->resting = max_float;
            body->flag |= RigidBody::flag_sleeping;
        }
    }

    MuliProfileZoneEnd(solve_island);
}

} // namespace muli3
