#pragma once

#include "contact_solver.h"
#include "simd.h"
#include "transform.h"

namespace muli3
{

class Contact;
class Joint;
class Body;
struct Timestep;

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

    Vec3 force;
    Vec3 torque;

    float resting;
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

// Scalar solver data is rebuilt from ContactState before each solve.
struct ScalarContactConstraint
{
    BodyState* bodyA;
    BodyState* bodyB;

    ContactConstraintSet constraints;

    Mat3 invIA;
    Mat3 invIB;
};

// Scalar contact state and prepared constraints use separate contiguous arrays.
// This wrapper keeps both arrays in lockstep as contacts enter and leave a batch.
struct ScalarContactArray
{
    int32 Add(Contact* contact, ContactState&& state);
    Contact* Remove(int32 index, ContactState* movedContact);

    Contact* GetContact(int32 index) const
    {
        return states[index].contact;
    }

    int32 Count() const
    {
        return int32(states.size());
    }

    bool Empty() const
    {
        return states.empty();
    }

    std::vector<ContactState> states;
    std::vector<ScalarContactConstraint> constraints;
};

// Persistent solver state for simple convex contacts.
// Solver impulses remain here so narrow phase and velocity solve operate on the same canonical SIMD storage.
struct BlockContactState
{
    void Resize(int32 blockCount);

    std::vector<PtrBlock<Contact>> contacts;
    std::vector<IntBlock> bodySetA;
    std::vector<IntBlock> bodyIndexA;
    std::vector<IntBlock> bodySetB;
    std::vector<IntBlock> bodyIndexB;

    std::vector<FloatBlock> friction;
    std::vector<FloatBlock> restitution;
    std::vector<FloatBlock> restitutionThreshold;
    std::vector<Vec2Block> surfaceSpeed;

    std::vector<IntBlock> manifoldId;
    std::vector<FloatBlock> pointCount;
    std::vector<Vec3Block> normal;
    std::vector<Vec3Block> linearImpulse;
    std::vector<FloatBlock> angularImpulse;
    std::vector<IntBlock> pointId[max_contact_point_count];
    std::vector<Vec3Block> anchorA[max_contact_point_count];
    std::vector<Vec3Block> anchorB[max_contact_point_count];
    std::vector<FloatBlock> normalImpulse[max_contact_point_count];
};

// Transient SIMD solver data.
// Body pointers are rebound during Prepare and are valid until the solve finishes.
// BodyState storage is not mutated in between.
struct BlockContactConstraint
{
    void Resize(int32 blockCount);

    std::vector<PtrBlock<BodyState>> bodyA;
    std::vector<PtrBlock<BodyState>> bodyB;

    std::vector<FloatBlock> invMassA;
    std::vector<FloatBlock> invMassB;

    std::vector<Mat3Block> invIA;
    std::vector<Mat3Block> invIB;

    std::vector<Mat3Block> localInvIA;
    std::vector<Mat3Block> localInvIB;

    std::vector<Vec3Block> localNormal;
    std::vector<Vec3Block> tangent1;
    std::vector<Vec3Block> tangent2;

    std::vector<Vec3Block> frictionWA1;
    std::vector<Vec3Block> frictionWA2;
    std::vector<Vec3Block> frictionWB1;
    std::vector<Vec3Block> frictionWB2;

    std::vector<Mat2Block> linearMass;
    std::vector<Vec2Block> tangentBias;
    std::vector<Vec2Block> tangentImpulse;
    std::vector<FloatBlock> angularMass;

    std::vector<Vec3Block> normalWA[max_contact_point_count];
    std::vector<Vec3Block> normalWB[max_contact_point_count];
    std::vector<FloatBlock> normalMass[max_contact_point_count];
    std::vector<FloatBlock> normalBias[max_contact_point_count];
    std::vector<FloatBlock> leverArm[max_contact_point_count];
    std::vector<Vec3Block> localPointA[max_contact_point_count];
    std::vector<Vec3Block> localPointB[max_contact_point_count];
};

// Convex single-manifold contacts stay packed while they belong to a graph batch.
// AoS/SoA conversion only occurs when a contact enters or leaves the batch.
struct BlockContactArray
{
    int32 Add(Contact* contact, int32 setA, int32 indexA, int32 setB, int32 indexB, ContactState&& state);
    Contact* Remove(int32 index, ContactState* removed);

    Contact* GetContact(int32 index) const
    {
        return state.contacts[index / simd_width].lane[index % simd_width];
    }

    int32 Count() const
    {
        return count;
    }

    int32 BlockCount() const
    {
        return (count + simd_width - 1) / simd_width;
    }

    bool Empty() const
    {
        return count == 0;
    }

    int32 count = 0;
    int32 blockCapacity = 0;
    BlockContactState state;
    BlockContactConstraint constraint;
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
