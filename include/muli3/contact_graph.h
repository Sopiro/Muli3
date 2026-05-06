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

    void AddCollider(Collider* collider);
    void RemoveCollider(Collider* collider);
    void UpdateCollider(Collider* collider, const Transform& transform);
    void UpdateCollider(Collider* collider, const Transform& transform0, const Transform& transform1);

private:
    friend class World;
    friend class BroadPhase;

    World* world;

    BroadPhase broadPhase;

    Contact* contactList;
    int32 contactCount;

    void Destroy(Contact* c);
    void OnNewContact(Collider* colliderA, Collider* colliderB);
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
