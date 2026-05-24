#pragma once

#include "camera.h"
#include "common.h"
#include "options.h"

namespace muli3
{

class Game;
class Renderer;

class Demo : public JointDestroyCallback, NonCopyable
{
public:
    Demo(Game& game);
    virtual ~Demo();

    virtual void UpdateInput();
    virtual void Step();
    virtual void UpdateUI() {}
    virtual void Render() {}

    virtual void OnJointDestroy(Joint* me) override;

    World& GetWorld();
    WorldSettings& GetWorldSettings();
    Camera& GetCamera();
    RigidBody* GetTargetBody();
    Collider* GetTargetCollider();

protected:
    friend class Game;

    void FindTargetBody();
    void EnableKeyboardShortcut();
    void EnableBodyCreate();
    bool EnableBodyGrab();
    void EnableCameraControl();

    Ray GetMouseRay() const;
    bool GetMouseWorldPointOnGrabPlane(Vec3* point) const;
    bool IsGrabJointActive() const;

    Game& game;
    Renderer& renderer;
    DebugOptions& options;

    Camera camera;
    WorldSettings settings;
    World* world = nullptr;
    float dt = 0.0f;
    Vec2 cursorPos{ 0.0f, 0.0f };
    Vec2 screenBounds{ 0.0f, 0.0f };
    RigidBody* targetBody = nullptr;
    Collider* targetCollider = nullptr;
    Vec3 targetPoint = Vec3::zero;

    GrabJoint* cursorJoint = nullptr;
    float grabDepth = 0.0f;
    float throwCooldown = 0.0f;
};

inline World& Demo::GetWorld()
{
    return *world;
}

inline WorldSettings& Demo::GetWorldSettings()
{
    return settings;
}

inline Camera& Demo::GetCamera()
{
    return camera;
}

inline void Demo::OnJointDestroy(Joint* me)
{
    MuliNotUsed(me);
    cursorJoint = nullptr;
}

inline RigidBody* Demo::GetTargetBody()
{
    return targetBody;
}

inline Collider* Demo::GetTargetCollider()
{
    return targetCollider;
}

typedef Demo* DemoCreateFunction(Game& game);

struct DemoFrame
{
    const char* category;
    const char* name;
    DemoCreateFunction* createFunction;
    int32 index;
};

inline std::vector<DemoFrame>& GetDemoFrames()
{
    static std::vector<DemoFrame> demoFrames;
    return demoFrames;
}

inline int32 register_demo(const char* category, const char* name, DemoCreateFunction* createFunction, int32 index = 0)
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    demoFrames.push_back(DemoFrame{ category, name, createFunction, index });
    return (int32)demoFrames.size();
}

inline void sort_demos()
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    std::sort(demoFrames.begin(), demoFrames.end(), [](const DemoFrame& lhs, const DemoFrame& rhs) {
        int categoryCompare = std::strcmp(lhs.category, rhs.category);
        if (categoryCompare != 0)
        {
            return categoryCompare < 0;
        }

        if (lhs.index != rhs.index)
        {
            return lhs.index < rhs.index;
        }

        return std::strcmp(lhs.name, rhs.name) < 0;
    });
}

} // namespace muli3
