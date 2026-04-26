#include "muli3/island.h"

namespace muli3
{

Island::Island(World* world, int32 bodyCapacity, int32 contactCapacity)
    : world{ world }
    , bodyCapacity{ bodyCapacity }
    , contactCapacity{ contactCapacity }
    , bodyCount{ 0 }
    , contactCount{ 0 }
    , sleeping{ false }
{
    bodies = (RigidBody**)world->linearAllocator.Allocate(bodyCapacity * sizeof(RigidBody*));
    contacts = (Contact**)world->linearAllocator.Allocate(contactCapacity * sizeof(Contact*));
}

Island::~Island()
{
    world->linearAllocator.Free(contacts, contactCapacity * sizeof(Contact*));
    world->linearAllocator.Free(bodies, bodyCapacity * sizeof(RigidBody*));
}

void Island::Solve()
{
    bool awakeIsland = false;

    const WorldSettings& settings = world->settings;
    const Timestep& step = settings.step;

    // Integrate velocities, yield tentative velocities that possibly violate the constraint
    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];
        b->transform0 = b->transform;

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
            Length2(b->linearVelocity) > settings.rest_linear_tolerance || Length2(b->torque) > 0.0f || Length2(b->force) > 0.0f)
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

            b->linearVelocity += b->force * b->invMass * step.dt;
            b->angularVelocity += b->GetInverseInertiaTensorWorld() *
                                  (b->torque - Cross(b->angularVelocity, b->GetInertiaTensorWorld() * b->angularVelocity)) *
                                  step.dt;
        }
    }

    // Prepare constraints for solving step
    for (int32 i = 0; i < contactCount; ++i)
    {
        contacts[i]->Prepare(step);
    }

    // Iteratively solve the violated velocity constraints
    // Solving contacts backward converges fast
    for (int32 i = 0; i < step.velocity_iterations; ++i)
    {
        for (int32 j = contactCount; j > 0; --j)
        {
            contacts[j - 1]->SolveVelocityConstraints(step);
        }
    }

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

        b->Integrate(step.dt);

        if (settings.world_bounds.TestPoint(b->transform.p) == false)
        {
            world->BufferDestroy(b);
        }
    }

    // Solve position constraints
    for (int32 i = 0; i < step.position_iterations; ++i)
    {
        bool contactSolved = true;

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

        if (contactSolved)
        {
            break;
        }
    }
}

} // namespace muli3
