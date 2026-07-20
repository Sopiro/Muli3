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
    ThreadPool* GetThreadPool() const;
    DebugOptions& GetDebugOptions();
    float GetTime() const;
    float GetFixedDeltaTime() const;
    void SetFixedDeltaTime(float newFixedDeltaTime);
    void RestartDemo();
    void NextDemo();
    void PrevDemo();

private:
    static constexpr int32 profile_capacity = 256;

    struct RenderItem
    {
        const Shape* shape;
        Transform transform;
        Vec4 color;
        bool visible;
    };

    void UpdateUI();
    void UpdateInput();
    void InitDemo(size_t index);
    void ClearProfiles();
    void RecreateThreadPool();

    Renderer renderer;
    Demo* demo = nullptr;
    int32 workerCount = 1;

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
    bool profileStopped = false;
    bool profileShowOverlay = false;
    bool profileShowAverage = false;
    Vec3 skyColor = color::HexToRGB(0xC2CEDC);
    float skyIntensity = 1.0f;
    Vec3 lightDirection = { 0.45f, -1.0f, -0.35f };
    Vec3 lightColor = { 1.0f, 1.0f, 1.0f };
    float lightIntensity = 1.5f;
    std::vector<RenderItem> renderItems;
    DebugOptions options;
};

inline Renderer& Game::GetRenderer()
{
    return renderer;
}

inline ThreadPool* Game::GetThreadPool() const
{
    return ThreadPool::global_thread_pool.get();
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
    ClearProfiles();
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
