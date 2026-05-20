#if defined(_WIN32) && defined(_DEBUG)
    #include <crtdbg.h>
#endif

#include <chrono>

#include "game.h"
#include "window.h"

using namespace muli3;

static Window* window;
static Game* game;

static int32 frameRate;
static int32 updateRate;
static float targetFrameTime;
static float targetUpdateTime;

static Vec3 clearColor = { 0.72f, 0.76f, 0.82f };

int32 GetFrameRate()
{
    return frameRate;
}

void SetFrameRate(int32 newFrameRate)
{
    frameRate = Clamp(newFrameRate, 30, 300);
    targetFrameTime = 1.0f / (float)frameRate;
}

int32 GetUpdateRate()
{
    return updateRate;
}

void SetUpdateRate(int32 newUpdateRate)
{
    updateRate = Clamp(newUpdateRate, 30, 300);
    targetUpdateTime = 1.0f / (float)updateRate;

    if (game)
    {
        game->SetFixedDeltaTime(targetUpdateTime);
    }
}

static bool Init()
{
    MuliProfileSetThreadName("muli3 demo");

    window = Window::Init(1600, 900, "Muli3 Demo");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    game = new Game();

    SetFrameRate(window->GetRefreshRate());
    SetUpdateRate(window->GetRefreshRate());
    return true;
}

static void Terminate()
{
    delete game;
    game = nullptr;

    Window::Shutdown();
    window = nullptr;
}

static void MainLoop()
{
    static float frameTime = 0.0f;
    static float updateTime = 0.0f;
    static auto lastTime = std::chrono::steady_clock::now();

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<float> duration = currentTime - lastTime;
    float elapsed = Clamp(duration.count(), 0.0f, 0.1f);
    lastTime = currentTime;

    updateTime += elapsed;
    frameTime += elapsed;

    while (updateTime >= targetUpdateTime)
    {
        game->FixedUpdate();
        updateTime -= targetUpdateTime;
    }

    if (frameTime > targetFrameTime)
    {
        window->BeginFrame(clearColor);
        {
            game->Update(frameTime);
            game->Render();
        }
        window->EndFrame();
        ProfileFrameMark();

        frameTime = 0;
    }
}

int main()
{
    ProfileStartup();

#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    if (!Init())
    {
        ProfileShutdown();
        return 1;
    }

    while (!window->ShouldClose())
    {
        MainLoop();
    }

    Terminate();
    ProfileShutdown();
    return 0;
}
