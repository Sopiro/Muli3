#pragma once

#include "collision.h"
#include "contact_solver.h"
#include "transform.h"

namespace muli3
{

class Contact;
class Joint;
class Body;
struct Timestep;

struct BodyState
{
    Body* body;

    Motion motion;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    float invMass;
    Mat3 invInertia;

    float linearDamping;
    float angularDamping;

    Vec3 force;
    Vec3 torque;

    float resting;
};

struct ContactState
{
    Contact* contact;

    BodyState* s1;
    BodyState* s2;

    ContactManifold manifold;

    SolverContact normalContact[max_contact_point_count];
    SolverContact tangentContact1[max_contact_point_count];
    SolverContact tangentContact2[max_contact_point_count];
    SolverPosition positionContact[max_contact_point_count];

    Mat3 invIA;
    Mat3 invIB;

    float friction;
    float restitution;
    float restitutionThreshold;
    Vec2 surfaceSpeed;
};

struct JointState
{
    Joint* joint;

    Mat3 invIA;
    Mat3 invIB;
};

enum SolverSetIndex
{
    // Static bodies live here.
    // Contacts and joints also move here when both connected bodies are static.
    static_set = 0,

    // Disabled bodies live here.
    // Contacts and joints move here when either connected body is disabled.
    disabled_set,

    // Awake dynamic/kinematic bodies live here.
    // Contacts and joints live here while at least one connected non-static body is awake.
    awake_set,

    // Sleeping dynamic/kinematic bodies live here.
    // Contacts and joints live here when all connected non-static bodies are sleeping.
    sleeping_set,

    solver_set_count,
};

struct SolverSet
{
    std::vector<BodyState> bodyStates;
    std::vector<ContactState> contactStates;
    std::vector<JointState> jointStates;
};

inline constexpr int32 null_index = -1;

} // namespace muli3
