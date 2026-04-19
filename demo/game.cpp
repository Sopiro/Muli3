#include "game.h"

#include "window.h"

extern muli3::int32 GetFrameRate();
extern void SetFrameRate(muli3::int32 newFrameRate);
extern muli3::int32 GetUpdateRate();
extern void SetUpdateRate(muli3::int32 newUpdateRate);

namespace muli3
{

Game::Game()
{
    MuliAssert(renderer.Initialize());

    sort_demos();
    demoCount = GetDemoFrames().size();
    MuliAssert(demoCount > 0);

    demoIndex = demoCount;

    InitDemo(0);
    Window::Get()->SetCursorHidden(true);
}

Game::~Game()
{
    delete demo;
    renderer.Shutdown();
}

void Game::Update(float deltaTime)
{
    if (restart)
    {
        InitDemo(newIndex);
        restart = false;
    }

    dt = deltaTime;
    time += dt;
    UpdateUI();
    UpdateInput();
}

void Game::FixedUpdate()
{
    if (paused)
    {
        if (step)
        {
            step = false;
            demo->Step();
        }
        return;
    }

    demo->Step();
}

void Game::Render()
{
    Window* window = Window::Get();
    Vec2 windowSize = window->GetWindowSize();
    float aspectRatio = windowSize.y > 0.0f ? windowSize.x / windowSize.y : 1.0f;
    renderer.Render(demo->GetWorld(), demo->GetCamera(), aspectRatio);
    demo->Render();
}

void Game::UpdateInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R)) RestartDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_DOWN)) PrevDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_UP)) NextDemo();

    Window* window = Window::Get();
    if (Input::IsKeyPressed(GLFW_KEY_TAB))
    {
        bool cursorHidden = window->GetCursorHidden();
        if (cursorHidden)
        {
            window->SetCursorHidden(false);
            ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        }
        else
        {
            window->SetCursorHidden(true);
            ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
        }
    }
    if (Input::IsKeyPressed(GLFW_KEY_ESCAPE))
    {
        window->SetCursorHidden(false);
    }
    if (Input::IsKeyPressed(GLFW_KEY_P))
    {
        paused = !paused;
    }

    demo->Update(dt, window->GetCursorHidden());
}

void Game::UpdateUI()
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    World& world = demo->GetWorld();
    WorldSettings& settings = demo->GetWorldSettings();

    ImGui::SetNextWindowPos({ 2.0f, 2.0f }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240.0f, 470.0f }, ImGuiCond_Once);

    static bool collapsed = false;
    if (Input::IsKeyPressed(GLFW_KEY_GRAVE_ACCENT))
    {
        collapsed = !collapsed;
    }
    ImGui::SetNextWindowCollapsed(collapsed, ImGuiCond_None);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("Muli3", NULL, flags))
    {
        if (ImGui::BeginTabBar("TabBar", ImGuiTabBarFlags_AutoSelectNewTabs))
        {
            if (ImGui::BeginTabItem("Control"))
            {
                static int32 fps = GetFrameRate();
                static int32 ups = GetUpdateRate();

                ImGui::BeginDisabled(paused);
                if (ImGui::Button("Pause")) paused = true;
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!paused);
                ImGui::PushButtonRepeat(true);
                if (ImGui::Button("Step")) step = true;
                ImGui::PopButtonRepeat();
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!paused);
                if (ImGui::Button("Start")) paused = false;
                ImGui::EndDisabled();

                ImGui::SameLine();
                if (ImGui::Button("Restart")) InitDemo(demoIndex);

                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::SliderInt("Frame rate", &fps, 30, 300))
                {
                    SetFrameRate(fps);
                }

                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::SliderInt("Update rate", &ups, 30, 300))
                {
                    SetUpdateRate(ups);
                }

                ImGui::Separator();
                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                ImGui::Separator();

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    ImGui::Checkbox("Apply gravity", &settings.apply_gravity);

                    ImGui::Text("Constraint solve iterations");
                    ImGui::SetNextItemWidth(120.0f);
                    ImGui::SliderInt("Velocity", &settings.step.velocity_iterations, 0, 50);
                }

                ImGui::Separator();
                ImGui::Text("%s", demoFrames[demoIndex].name);
                ImGui::Text("Bodies: %d", world.GetRigidBodyCount());
                ImGui::Text("Pause: P");
                ImGui::Text("Cursor: Tab / Esc");
                ImGui::Text("Demos: PageUp / PageDown");
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginListBox("##demo_list", ImVec2{ -FLT_MIN, 20.0f * ImGui::GetTextLineHeightWithSpacing() }))
                {
                    for (int32 i = 0; i < (int32)demoCount; ++i)
                    {
                        bool selected = demoIndex == (size_t)i;
                        if (ImGui::Selectable(demoFrames[i].name, selected))
                        {
                            InitDemo((size_t)i);
                        }

                        if (selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }

                    ImGui::EndListBox();
                }
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }

    if (!collapsed && ImGui::IsWindowCollapsed())
    {
        collapsed = true;
    }
    if (collapsed && !ImGui::IsWindowCollapsed())
    {
        collapsed = false;
    }

    ImGui::End();

    demo->UpdateUI();
}

void Game::InitDemo(size_t index)
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();

    if (index >= demoFrames.size())
    {
        return;
    }

    bool restoreSettings = demo && demoIndex == index;
    bool restoreCameraPosition = restoreSettings;
    Camera previousCamera;
    WorldSettings previousSettings;

    if (restoreSettings)
    {
        previousSettings = demo->GetWorldSettings();
        previousCamera = demo->GetCamera();
    }

    delete demo;
    demo = nullptr;

    time = 0.0f;
    demoIndex = index;
    demo = demoFrames[demoIndex].createFunction(*this);

    if (restoreSettings)
    {
        demo->GetWorldSettings() = previousSettings;
    }

    if (restoreCameraPosition)
    {
        demo->GetCamera() = previousCamera;
    }

    demo->dt = fixedDeltaTime;
    paused = false;
    step = false;
}

} // namespace muli3
