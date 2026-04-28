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
    bool rendererInitialized = renderer.Initialize();
    MuliAssert(rendererInitialized);
    if (!rendererInitialized)
    {
        std::exit(1);
    }

    sort_demos();
    demoCount = GetDemoFrames().size();
    MuliAssert(demoCount > 0);

    demoIndex = demoCount;

    InitDemo(0);
    Window::Get()->SetCursorHidden(false);
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
    if (options.pause)
    {
        if (options.step)
        {
            options.step = false;
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
    renderer.Render(demo->GetWorld(), demo->GetCamera(), aspectRatio, options);
    demo->Render();
}

void Game::UpdateInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R)) RestartDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_DOWN)) PrevDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_UP)) NextDemo();

    Window* window = Window::Get();
    if (!window->GetCursorHidden() && !ImGui::GetIO().WantCaptureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(true);
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
    }
    if (window->GetCursorHidden() && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(false);
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
    }
    if (Input::IsKeyPressed(GLFW_KEY_ESCAPE))
    {
        window->SetCursorHidden(false);
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
    }
    EnableKeyboardShortcut();

    demo->Update(dt, window->GetCursorHidden());
}

void Game::EnableKeyboardShortcut()
{
    if (ImGui::GetIO().WantCaptureKeyboard || Window::Get()->GetCursorHidden())
    {
        return;
    }

    if (Input::IsKeyPressed(GLFW_KEY_Y)) options.draw_body = !options.draw_body;
    if (Input::IsKeyPressed(GLFW_KEY_O)) options.draw_wireframe = !options.draw_wireframe;
    if (Input::IsKeyPressed(GLFW_KEY_B)) options.show_aabb = !options.show_aabb;
    if (Input::IsKeyPressed(GLFW_KEY_V)) options.show_bvh = !options.show_bvh;
    if (Input::IsKeyPressed(GLFW_KEY_P)) options.show_contact_point = !options.show_contact_point;
    if (Input::IsKeyPressed(GLFW_KEY_N)) options.show_contact_normal = !options.show_contact_normal;
    if (Input::IsKeyPressed(GLFW_KEY_C)) options.reset_camera = !options.reset_camera;
    if (Input::IsKeyPressed(GLFW_KEY_Q)) options.pause = !options.pause;
    if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_E)) options.step = true;

    if (Input::IsKeyPressed(GLFW_KEY_G))
    {
        demo->GetWorldSettings().apply_gravity = !demo->GetWorldSettings().apply_gravity;
    }
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

                ImGui::BeginDisabled(options.pause);
                if (ImGui::Button("Pause")) options.pause = true;
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!options.pause);
                ImGui::PushButtonRepeat(true);
                if (ImGui::Button("Step")) options.step = true;
                ImGui::PopButtonRepeat();
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!options.pause);
                if (ImGui::Button("Start")) options.pause = false;
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

                ImGui::SetNextItemOpen(false, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Debug options"))
                {
                    ImGui::Checkbox("Camera reset", &options.reset_camera);
                    ImGui::Checkbox("Draw body", &options.draw_body);
                    ImGui::Checkbox("Draw wireframe", &options.draw_wireframe);
                    ImGui::Checkbox("Show BVH", &options.show_bvh);
                    ImGui::Checkbox("Show AABB", &options.show_aabb);
                    ImGui::Checkbox("Show contact point", &options.show_contact_point);
                    ImGui::Checkbox("Show contact normal", &options.show_contact_normal);
                }

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    ImGui::Checkbox("Apply gravity", &settings.apply_gravity);

                    ImGui::Text("Constraint solve iterations");
                    ImGui::SetNextItemWidth(120.0f);
                    ImGui::SliderInt("Velocity", &settings.step.velocity_iterations, 0, 50);
                    ImGui::SetNextItemWidth(120.0f);
                    ImGui::SliderInt("Position", &settings.step.position_iterations, 0, 50);
                }

                ImGui::Separator();
                ImGui::Text("%s", demoFrames[demoIndex].name);
                ImGui::Text("Bodies: %d", world.GetBodyCount());
                ImGui::Text("Pause: Q");
                ImGui::Text("Step: E / Right");
                ImGui::Text("Camera: Hold RMB");
                ImGui::Text("Cursor: Esc");
                ImGui::Text("Demos: PageUp / PageDown");
                ImGui::Text("Draw body: Y");
                ImGui::Text("Wireframe: O");
                ImGui::Text("AABB: B");
                ImGui::Text("Contact point: P");
                ImGui::Text("Contact normal: N");
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
    bool restoreCameraPosition = demo && demoIndex == index && !options.reset_camera;
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
    options.step = false;
}

} // namespace muli3
