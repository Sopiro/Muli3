#if defined(_WIN32) && defined(_DEBUG)
    #include <crtdbg.h>
#endif

#include "game.h"
#include "window.h"

namespace muli3
{

static Window* window;
static Game* game;

static int32 frameRate;
static int32 updateRate;
static float targetFrameTime;
static float targetUpdateTime;

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
    targetUpdateTime = 1.0f / updateRate;

    game->SetFixedDeltaTime(targetUpdateTime);
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
    SetUpdateRate(60);
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
    using clock = std::chrono::steady_clock;
    using duration = std::chrono::duration<float>;

    static float frameTime = 0;
    static float updateTime = 0;

    static auto lastTime = clock::now();
    static auto lastFrameTime = lastTime;

    auto currentTime = clock::now();

    float deltaTime = duration(currentTime - lastTime).count();
    lastTime = currentTime;

    updateTime += deltaTime;
    frameTime += deltaTime;

    if (updateTime >= targetUpdateTime)
    {
        ProfileFrameMark();
        game->FixedUpdate();

        updateTime = Min(updateTime - targetUpdateTime, targetUpdateTime);
    }

    if (frameTime >= targetFrameTime)
    {
        currentTime = clock::now();

        deltaTime = duration(currentTime - lastFrameTime).count();
        lastFrameTime = currentTime;

        frameTime = Min(frameTime - targetFrameTime, targetFrameTime);

        // MuliProfileZoneNR(render, "Render", true);
        window->BeginFrame();
        {
            game->Update(deltaTime);
            game->Render(updateTime / targetUpdateTime);
        }
        window->EndFrame();
        // MuliProfileZoneEnd(render);
    }
    else
    {
        Pause();
    }
}

} // namespace muli3

int main()
{
    using namespace muli3;

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
