#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace muli3
{

class World;

inline constexpr int32 constraint_color_count = 24;
inline constexpr int32 constraint_overflow_index = constraint_color_count - 1;

// A batch of constraints assigned the same graph(edge) color.
// Constraints in a batch do not share bodies and can be solved in parallel.
struct ConstraintBatch
{
    std::vector<ContactState> contactStates;
    std::vector<JointState> jointStates;
};

class ConstraintGraph
{
public:
    ConstraintGraph(World* world);
    ~ConstraintGraph();

    void UpdateContactGraph();
    void EvaluateContacts();

    int32 GetContactCount() const;

    ContactState* GetContactState(Contact* contact);
    const ContactState* GetContactState(const Contact* contact) const;
    JointState* GetJointState(Joint* joint);
    const JointState* GetJointState(const Joint* joint) const;

private:
    friend class World;
    friend class Body;
    friend class Collider;
    friend class Contact;
    friend class Joint;
    friend class BroadPhase;

    void AddCollider(Collider* collider);
    void RemoveCollider(Collider* collider);
    void UpdateCollider(Collider* collider, const Transform& transform);
    void UpdateCollider(Collider* collider, const Transform& transform0, const Transform& transform1);

    int32 AssignColor(Body* bodyA, Body* bodyB);
    void AddColor(Body* bodyA, Body* bodyB, int32 colorIndex);
    void RemoveColor(Body* bodyA, Body* bodyB, int32 colorIndex);

    ContactState* AddContactToGraph(Contact* contact, ContactState&& source);
    void RemoveContactFromGraph(Contact* contact);
    JointState* AddJointToGraph(Joint* joint, const JointState& source);
    void RemoveJointFromGraph(Joint* joint);

    World* world;
    BroadPhase broadPhase;

    ConstraintBatch batches[constraint_color_count];

    Contact* contactList;
    int32 contactCount;

    void Destroy(Contact* c);
    void OnNewContact(Collider* colliderA, Collider* colliderB);
};

inline void ConstraintGraph::UpdateContactGraph()
{
    broadPhase.FindNewContacts(this);
}

inline int32 ConstraintGraph::GetContactCount() const
{
    return contactCount;
}

} // namespace muli3
