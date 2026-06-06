#include "muli3/parallel.h"

#include "game.h"
#include "profile_graph.h"
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

    workerCount = 8;
    RecreateThreadPool();
    InitDemo(26);
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
    demo->Step();

    if (profileStopped)
    {
        return;
    }

    if (profileWriteIndex == profile_capacity + profileReadIndex)
    {
        ++profileReadIndex;
    }

    int32 index = (int32)(profileWriteIndex & (profile_capacity - 1));
    profiles[index] = demo->GetWorld().GetProfile();
    ++profileWriteIndex;
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
    demo->UpdateInput();
}

void Game::UpdateUI()
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    World& world = demo->GetWorld();
    WorldSettings& settings = demo->GetWorldSettings();

    // ImGui::ShowDemoWindow();

    ImGui::SetNextWindowPos({ 2, 2 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 470 }, ImGuiCond_Once);

    static bool collapsed = false;
    if (Input::IsKeyPressed(GLFW_KEY_GRAVE_ACCENT))
    {
        collapsed = !collapsed;
        ImGui::SetNextWindowCollapsed(collapsed, ImGuiCond_None);
    }

    ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("Muli Engine", NULL, flags))
    {
        if (ImGui::BeginTabBar("TabBar", ImGuiTabBarFlags_AutoSelectNewTabs))
        {
            if (ImGui::BeginTabItem("Control"))
            {
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

                static int32 fps = GetFrameRate();
                static int32 ups = GetUpdateRate();

                ImGui::SetNextItemWidth(120);
                if (ImGui::SliderInt("Frame rate", &fps, 30, 300))
                {
                    SetFrameRate(fps);
                }

                ImGui::SetNextItemWidth(120);
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
                    ImGui::Checkbox("Show profiler", &options.show_profiler);
                    ImGui::Checkbox("Camera reset", &options.reset_camera);
                    ImGui::Checkbox("Colorize island", &options.colorize_island);
                    ImGui::Checkbox("Draw body", &options.draw_body);
                    ImGui::Checkbox("Draw joint", &options.draw_joint);
                    ImGui::Checkbox("Draw outlined", &options.draw_outlined);
                    ImGui::Checkbox("Show BVH", &options.show_bvh);
                    ImGui::Checkbox("Show AABB", &options.show_aabb);
                    ImGui::Checkbox("Show contact point", &options.show_contact_point);
                    ImGui::Checkbox("Show contact normal", &options.show_contact_normal);
                }

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    if (ImGui::Checkbox("Apply gravity", &settings.apply_gravity))
                    {
                        world.Awake();
                    }

                    ImGui::Text("Constraint solve iterations");
                    ImGui::SetNextItemWidth(120);
                    ImGui::SliderInt("Velocity", &settings.step.velocity_iterations, 0, 50);
                    ImGui::SetNextItemWidth(120);
                    ImGui::SliderInt("Position", &settings.step.position_iterations, 0, 50);
                    ImGui::SetNextItemWidth(120);
                    if (ImGui::SliderInt("Workers", &workerCount, 1, int32(std::max(1u, std::thread::hardware_concurrency()))))
                    {
                        RecreateThreadPool();
                    }
                    ImGui::Checkbox("Warm Starting", &settings.step.warm_starting);
                    ImGui::Checkbox("Sleeping", &settings.sleeping);
                }

                ImGui::Separator();
                ImGui::Text("%s / %s", demoFrames[demoIndex].category, demoFrames[demoIndex].name);
                ImGui::Text("Bodies: %d", world.GetBodyCount());
                ImGui::Text("Sleeping dynamic bodies: %d", world.GetSleepingBodyCount());
                ImGui::Text("Broad phase contacts: %d", world.GetContactCount());
                ImGui::Text("Steps: %lld", world.GetStepIndex());
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginChild("##demos", ImVec2{ -FLT_MIN, 24 * ImGui::GetTextLineHeightWithSpacing() }, false))
                {
                    const char* currentCategory = nullptr;
                    bool categoryOpen = false;

                    for (int32 i = 0; i < (int32)demoCount; ++i)
                    {
                        if (currentCategory == nullptr || std::strcmp(currentCategory, demoFrames[i].category) != 0)
                        {
                            currentCategory = demoFrames[i].category;

                            bool hasSelectedDemo = false;
                            for (int32 j = i; j < (int32)demoCount; ++j)
                            {
                                if (std::strcmp(currentCategory, demoFrames[j].category) != 0)
                                {
                                    break;
                                }

                                if (demoIndex == (size_t)j)
                                {
                                    hasSelectedDemo = true;
                                    break;
                                }
                            }

                            if (hasSelectedDemo)
                            {
                                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                            }

                            ImGui::PushID(currentCategory);
                            categoryOpen = ImGui::CollapsingHeader(currentCategory, ImGuiTreeNodeFlags_SpanAvailWidth);
                            ImGui::PopID();
                        }

                        if (!categoryOpen)
                        {
                            continue;
                        }

                        bool selected = demoIndex == (size_t)i;
                        ImGui::PushID(i);
                        ImGui::Bullet();
                        ImGui::SameLine();
                        if (ImGui::Selectable(demoFrames[i].name, selected))
                        {
                            InitDemo((size_t)i);
                        }
                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                }
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }

    if (!collapsed && ImGui::IsWindowCollapsed()) collapsed = true;
    if (collapsed && !ImGui::IsWindowCollapsed()) collapsed = false;

    ImGui::End();

    if (options.show_profiler)
    {
        ImGui::SetNextWindowSize({ 720.0f, 280.0f }, ImGuiCond_Once);
        if (ImGui::Begin("Profile", &options.show_profiler, ImGuiWindowFlags_AlwaysAutoResize))
        {
            int count = (int)(profileWriteIndex - profileReadIndex);
            float broadPhaseValues[profile_capacity]{};
            float narrowPhaseValues[profile_capacity]{};
            float buildIslandsValues[profile_capacity]{};
            float integrateVelocitiesValues[profile_capacity]{};
            float prepareConstraintsValues[profile_capacity]{};
            float warmStartValues[profile_capacity]{};
            float solveVelocitiesValues[profile_capacity]{};
            float integratePositionsValues[profile_capacity]{};
            float solvePositionsValues[profile_capacity]{};
            float sleepAndSyncValues[profile_capacity]{};
            float finalizeValues[profile_capacity]{};
            float postSolveValues[profile_capacity]{};

            for (int32 i = 0; i < count; ++i)
            {
                int32 index = (int32)((profileReadIndex + i) & (profile_capacity - 1));
                const WorldProfile& profile = profiles[index];
                broadPhaseValues[i] = profile.broad_phase;
                narrowPhaseValues[i] = profile.narrow_phase;
                buildIslandsValues[i] = profile.build_islands;
                integrateVelocitiesValues[i] = profile.integrate_velocities;
                prepareConstraintsValues[i] = profile.prepare_constraints;
                warmStartValues[i] = profile.warm_start;
                solveVelocitiesValues[i] = profile.solve_velocities;
                integratePositionsValues[i] = profile.integrate_positions;
                solvePositionsValues[i] = profile.solve_positions;
                sleepAndSyncValues[i] = profile.sleep_and_sync;
                finalizeValues[i] = profile.finalize;
                postSolveValues[i] = profile.post_solve;
            }

            ProfileGraphEntry entries[] = {
                { "Broad phase", color::broad_phase, broadPhaseValues },
                { "Narrow phase", color::narrow_phase, narrowPhaseValues },
                { "Build islands", color::build_islands, buildIslandsValues },
                { "Integrate velocities", color::integrate_velocities, integrateVelocitiesValues },
                { "Prepare constraints", color::prepare_constraints, prepareConstraintsValues },
                { "Warm start", color::warm_start, warmStartValues },
                { "Solve velocities", color::solve_velocities, solveVelocitiesValues },
                { "Integrate positions", color::integrate_positions, integratePositionsValues },
                { "Solve positions", color::solve_positions, solvePositionsValues },
                { "Sleep and sync", color::sleep_and_sync, sleepAndSyncValues },
                { "Finalize", color::finalize, finalizeValues },
                { "Post solve", color::post_solve, postSolveValues },
            };

            DrawProfileGraph(
                "", { -1.0f, -1.0f }, profile_capacity, count, entries, (int32)(sizeof(entries) / sizeof(entries[0])), true,
                profileShowOverlay, profileShowAverage
            );

            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 32.0f);
            ImGui::Checkbox("Stop", &profileStopped);
            ImGui::SameLine();
            ImGui::Checkbox("Overlay", &profileShowOverlay);
            ImGui::SameLine();
            if (ImGui::Button("Clear")) ClearProfiles();
            ImGui::SameLine();
            ImGui::Checkbox("Average", &profileShowAverage);
        }
        ImGui::End();
    }

    ImGui::SetNextWindowPos({ 0.0f, Window::Get()->GetWindowSize().y }, ImGuiCond_Always, { 0.0f, 1.0f });
    ImGui::Begin(
        "Body info", NULL,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
    );
    RigidBody* targetBody = demo->GetTargetBody();
    if (targetBody)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{ 12 / 255.0f, 11 / 255.0f, 14 / 255.0f, 1.0f });
        ImGui::Text("Mass: %.2f", targetBody->GetMass());
        Vec3 pos = targetBody->GetPosition();
        Vec3 rot = targetBody->GetRotation().ToEuler() * Vec3(inv_pi * 180);
        ImGui::Text("Pos: %.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
        ImGui::Text("Rot: %.2f, %.2f, %.2f", rot.x, rot.y, rot.z);
        ImGui::PopStyleColor();
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
        previousSettings.thread_pool = GetThreadPool();
        previousCamera = demo->GetCamera();
    }

    delete demo;
    demo = nullptr;
    renderer.ClearMeshCache();

    time = 0.0f;
    demoIndex = index;
    demo = demoFrames[demoIndex].createFunction(*this);
    demo->GetWorldSettings().thread_pool = GetThreadPool();
    // ClearProfiles();

    if (restoreSettings)
    {
        demo->GetWorldSettings() = previousSettings;
        demo->GetWorldSettings().thread_pool = GetThreadPool();
    }

    if (restoreCameraPosition)
    {
        demo->GetCamera() = previousCamera;
    }

    demo->dt = fixedDeltaTime;
    options.step = false;
}

void Game::ClearProfiles()
{
    std::memset(profiles, 0, profile_capacity * sizeof(WorldProfile));
    profileReadIndex = 0;
    profileWriteIndex = 0;
}

void Game::RecreateThreadPool()
{
    workerCount = std::max(workerCount, 1);
    threadPool = std::make_unique<ThreadPool>(workerCount);

    if (demo)
    {
        demo->GetWorldSettings().thread_pool = threadPool.get();
    }
}

} // namespace muli3
