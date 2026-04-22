#pragma once

#include "contact.h"
#include "settings.h"

namespace muli3
{

class World
{
public:
    World(const WorldSettings& settings);
    ~World() = default;

    void Reset();
    void Step(float dt);

    Shape* CreateSphereShape(float radius);
    RigidBody* CreateRigidBody(const RigidBody& body = RigidBody{});
    RigidBody* CreateSphere(float radius, const Transform& transform, bool isStatic, float mass = 1.0f);

    std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies();
    const std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies() const;
    int32 GetRigidBodyCount() const;
    const std::vector<Contact>& GetContacts() const;

private:
    void IntegrateBodies();
    void FindContacts();
    void SolveContacts();

    const WorldSettings& settings;
    std::vector<std::unique_ptr<Shape>> shapes;
    std::vector<std::unique_ptr<RigidBody>> bodies;
    std::vector<Contact> contacts;
};

inline std::vector<std::unique_ptr<RigidBody>>& World::GetRigidBodies()
{
    return bodies;
}

inline const std::vector<std::unique_ptr<RigidBody>>& World::GetRigidBodies() const
{
    return bodies;
}

inline int32 World::GetRigidBodyCount() const
{
    return (int32)bodies.size();
}

inline const std::vector<Contact>& World::GetContacts() const
{
    return contacts;
}

} // namespace muli3
