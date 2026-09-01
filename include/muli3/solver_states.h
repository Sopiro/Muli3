#pragma once

#include "collision.h"
#include "transform.h"

namespace muli3
{

class Body;
class Joint;
class Contact;

// Bodies remain in a AoS layout.
// This is the stable scalar view used by collision, joints, and the public body API.
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
    float gravityScale;

    Vec3 force;
    Vec3 torque;

    float resting;
};

struct JointState
{
    Joint* joint;

    Mat3 invIA;
    Mat3 invIB;
};

// Complex and multi-manifold contacts keep their persistent collision state in AoS form.
struct ContactState
{
    Contact* contact;

    ManifoldSet manifolds;

    Float friction;
    Float restitution;
    Float restitutionThreshold;
    Vec2 surfaceSpeed;
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
    std::vector<JointState> jointStates;
    std::vector<ContactState> contactStates;
};

inline constexpr int32 null_index = -1;

} // namespace muli3
