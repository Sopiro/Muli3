#pragma once

#include "camera.h"
#include "common.h"
#include "options.h"

namespace muli3
{

class Game;
class Renderer;

class Demo : NonCopyable
{
public:
    Demo(Game& game);
    virtual ~Demo();

    virtual void Update(float dt, bool captureMouse);
    virtual void Step();
    virtual void UpdateUI() {}
    virtual void Render() {}

    World& GetWorld();
    WorldSettings& GetWorldSettings();
    Camera& GetCamera();

protected:
    friend class Game;

    Game& game;
    Renderer& renderer;
    DebugOptions& options;

    Camera camera;
    WorldSettings settings;
    World* world = nullptr;
    float dt = 0.0f;
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

typedef Demo* DemoCreateFunction(Game& game);

struct DemoFrame
{
    const char* name;
    DemoCreateFunction* createFunction;
    int32 index;
};

inline std::vector<DemoFrame>& GetDemoFrames()
{
    static std::vector<DemoFrame> demoFrames;
    return demoFrames;
}

inline int32 register_demo(const char* name, DemoCreateFunction* createFunction, int32 index = 0)
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    demoFrames.push_back(DemoFrame{ name, createFunction, index });
    return (int32)demoFrames.size();
}

inline void sort_demos()
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    std::sort(demoFrames.begin(), demoFrames.end(), [](const DemoFrame& lhs, const DemoFrame& rhs) {
        return lhs.index < rhs.index;
    });
}

} // namespace muli3
