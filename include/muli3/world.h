#pragma once

#include "rigid_body.h"
#include "settings.h"

namespace muli3
{

class World
{
public:
    explicit World(const WorldSettings& settings);
    ~World() = default;

    void Reset();
    void Step(float dt);

    RigidBody* CreateRigidBody(const RigidBody& body = RigidBody{});
    RigidBody* CreateSphere(float radius, const Transform& transform, bool isStatic, float mass = 1.0f);

    std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies();
    const std::vector<std::unique_ptr<RigidBody>>& GetRigidBodies() const;
    int32 GetRigidBodyCount() const;

private:
    void SolveContacts(float dt);
    void SolveSphereContact(RigidBody& a, RigidBody& b, float dt);

    const WorldSettings& settings;
    std::vector<std::unique_ptr<RigidBody>> bodies;
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

} // namespace muli3
