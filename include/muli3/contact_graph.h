#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace muli3
{
class World;

class ContactGraph
{
public:
    ContactGraph(World* world);
    ~ContactGraph();

    void UpdateContactGraph();
    void EvaluateContacts();

    int32 GetContactCount() const;

protected:
    friend class RigidBody;

    void AddBody(RigidBody* body);
    void RemoveBody(RigidBody* body);
    void UpdateBody(RigidBody* body, const Transform& transform);
    void UpdateBody(RigidBody* body, const Transform& transform0, const Transform& transform1);

private:
    friend class World;
    friend class BroadPhase;

    World* world;

    BroadPhase broadPhase;

    Contact* contactList;
    int32 contactCount;

    void Destroy(Contact* c);
    void OnNewContact(RigidBody* bodyA, RigidBody* bodyB);
};

inline void ContactGraph::UpdateContactGraph()
{
    broadPhase.FindNewContacts();
}

inline int32 ContactGraph::GetContactCount() const
{
    return contactCount;
}

} // namespace muli3

