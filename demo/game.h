#pragma once

#include "demo.h"
#include "options.h"
#include "renderer.h"

namespace muli3
{

class Game : NonCopyable
{
public:
    Game();
    ~Game();

    void Update(float dt);
    void FixedUpdate();
    void Render();

    Renderer& GetRenderer();
    DebugOptions& GetDebugOptions();
    float GetTime() const;
    float GetFixedDeltaTime() const;
    void SetFixedDeltaTime(float newFixedDeltaTime);
    void RestartDemo();
    void NextDemo();
    void PrevDemo();

private:
    static constexpr int32 profile_capacity = 256;

    void UpdateUI();
    void UpdateInput();
    void InitDemo(size_t index);

    Renderer renderer;
    Demo* demo = nullptr;

    bool restart = false;
    float fixedDeltaTime = 1.0f / 60.0f;
    float time = 0.0f;
    float dt = 0.0f;
    size_t demoCount = 0;
    size_t demoIndex = 0;
    size_t newIndex = 0;
    WorldProfile profiles[profile_capacity]{};
    uint64 profileReadIndex = 0;
    uint64 profileWriteIndex = 0;
    DebugOptions options;
};

inline Renderer& Game::GetRenderer()
{
    return renderer;
}

inline DebugOptions& Game::GetDebugOptions()
{
    return options;
}

inline float Game::GetTime() const
{
    return time;
}

inline float Game::GetFixedDeltaTime() const
{
    return fixedDeltaTime;
}

inline void Game::SetFixedDeltaTime(float newFixedDeltaTime)
{
    fixedDeltaTime = newFixedDeltaTime;

    if (demo)
    {
        demo->dt = newFixedDeltaTime;
    }
}

inline void Game::RestartDemo()
{
    restart = true;
    newIndex = demoIndex;
}

inline void Game::NextDemo()
{
    if (demoCount == 0)
    {
        return;
    }

    restart = true;
    newIndex = (demoIndex + 1) % demoCount;
}

inline void Game::PrevDemo()
{
    if (demoCount == 0)
    {
        return;
    }

    restart = true;
    newIndex = (demoIndex + demoCount - 1) % demoCount;
}

} // namespace muli3
