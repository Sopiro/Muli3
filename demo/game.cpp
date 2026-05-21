#include "game.h"

#include "window.h"

extern muli3::int32 GetFrameRate();
extern void SetFrameRate(muli3::int32 newFrameRate);
extern muli3::int32 GetUpdateRate();
extern void SetUpdateRate(muli3::int32 newUpdateRate);

namespace muli3
{

enum ProfileValue
{
    profile_broad_phase,
    profile_narrow_phase,
    profile_deferred_destroy,
    profile_step_other,
    profile_build_islands,
    profile_integrate_velocities,
    profile_prepare_constraints,
    profile_solve_velocity,
    profile_integrate_positions,
    profile_solve_position,
    profile_update_transforms,
    profile_clear_island_flags,
    profile_solve_other,
    profile_solve_rest,
};

struct ProfileGraphEntry
{
    const char* name;
    uint32 color;
    ProfileValue value;
};

static ImU32 ToImColor(uint32 c)
{
    return IM_COL32((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff, 255);
}

static float GetProfileValue(const WorldProfile& profile, ProfileValue value)
{
    switch (value)
    {
    case profile_broad_phase:
        return profile.broad_phase;
    case profile_narrow_phase:
        return profile.narrow_phase;
    case profile_deferred_destroy:
        return profile.deferred_destroy;
    case profile_step_other:
        return (std::max)(0.0f,
                          profile.step - profile.broad_phase - profile.narrow_phase - profile.solve - profile.deferred_destroy);
    case profile_build_islands:
        return profile.build_islands;
    case profile_integrate_velocities:
        return profile.integrate_velocities;
    case profile_prepare_constraints:
        return profile.prepare_constraints;
    case profile_solve_velocity:
        return profile.solve_velocity;
    case profile_integrate_positions:
        return profile.integrate_positions;
    case profile_solve_position:
        return profile.solve_position;
    case profile_update_transforms:
        return profile.update_transforms;
    case profile_clear_island_flags:
        return profile.clear_island_flags;
    case profile_solve_other:
        return (std::max)(0.0f, profile.solve_world - profile.build_islands - profile.integrate_velocities -
                                    profile.prepare_constraints - profile.solve_velocity - profile.integrate_positions -
                                    profile.solve_position - profile.update_transforms - profile.clear_island_flags);
    case profile_solve_rest:
        return (std::max)(0.0f, profile.solve - profile.solve_world);
    default:
        return 0.0f;
    }
}

static float GetProfileTotal(const WorldProfile& profile, const ProfileGraphEntry* entries, int32 entryCount)
{
    float total = 0.0f;

    for (int32 i = 0; i < entryCount; ++i)
    {
        total += GetProfileValue(profile, entries[i].value);
    }

    return total;
}

static void DrawProfileGraph(
    const char* label,
    const WorldProfile* profiles,
    Vec2 graphSize,
    int32 profileCapacity,
    uint64 profileReadIndex,
    int32 count,
    const ProfileGraphEntry* entries,
    int32 entryCount,
    float maxRange,
    bool showOverlay
)
{
    if (count <= 0)
    {
        ImGui::TextUnformatted(label);
        return;
    }

    float maxValue = 0.1f;
    for (int32 i = 0; i < count; ++i)
    {
        int32 index = (int32)((profileReadIndex + i) & (profileCapacity - 1));
        maxValue = (std::max)(maxValue, GetProfileTotal(profiles[index], entries, entryCount));
    }

    float minValue = 0.0f;
    if (maxRange > 0)
    {
        maxValue = (std::min)(maxValue, maxRange);
    }

    ImGui::Text("%s (%.3f ms max)", label, maxValue);

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 plotSize{ graphSize.x, graphSize.y };
    ImVec2 legendSize{ 240.0f, plotSize.y };
    ImVec2 spacing{ 14.0f, 0.0f };
    ImVec2 canvasSize{ plotSize.x + spacing.x + legendSize.x, plotSize.y };
    ImVec2 canvasMin = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton(label, canvasSize);

    ImVec2 plotMin = canvasMin;
    ImVec2 plotMax{ plotMin.x + plotSize.x, plotMin.y + plotSize.y };
    ImVec2 legendMin{ plotMax.x + spacing.x, plotMin.y };
    ImU32 backgroundColor = IM_COL32(14, 29, 34, 255);
    ImU32 borderColor = IM_COL32(190, 205, 205, 255);
    ImU32 gridColor = IM_COL32(80, 105, 110, 90);

    drawList->AddRectFilled(plotMin, plotMax, backgroundColor);
    drawList->AddRect(plotMin, plotMax, borderColor);

    for (int32 i = 1; i < 4; ++i)
    {
        float y = plotMax.y - plotSize.y * (float)i / 4.0f;
        drawList->AddLine(ImVec2{ plotMin.x, y }, ImVec2{ plotMax.x, y }, gridColor);
    }

    float columnStep = plotSize.x / (float)count;
    float barWidth = (std::max)(1.0f, columnStep - 1.0f);
    float scale = plotSize.y / (maxValue - minValue);

    for (int32 i = 0; i < count; ++i)
    {
        int32 index = (int32)((profileReadIndex + i) & (profileCapacity - 1));
        const WorldProfile& profile = profiles[index];

        float x0 = plotMin.x + columnStep * (float)i;
        float x1 = (std::min)(x0 + barWidth, plotMax.x);
        float stack = 0.0f;

        for (int32 j = entryCount - 1; j >= 0; --j)
        {
            float value = GetProfileValue(profile, entries[j].value);
            if (value <= 0.0f)
            {
                continue;
            }

            float y0 = plotMax.y - ((stack + value) - minValue) * scale;
            float y1 = plotMax.y - (stack - minValue) * scale;
            y0 = Clamp(y0, plotMin.y, plotMax.y);
            y1 = Clamp(y1, plotMin.y, plotMax.y);
            drawList->AddRectFilled(ImVec2{ x0, y0 }, ImVec2{ x1, y1 }, ToImColor(entries[j].color));
            stack += value;
        }
    }

    int32 latestIndex = (int32)((profileReadIndex + count - 1) & (profileCapacity - 1));
    const WorldProfile& latestProfile = profiles[latestIndex];
    float textHeight = ImGui::GetTextLineHeight();
    float legendStep = (std::min)(textHeight + 2.0f, plotSize.y / (float)entryCount);
    float legendY = legendMin.y;

    for (int32 i = 0; i < entryCount; ++i)
    {
        float stack = 0.0f;
        for (int32 j = entryCount - 1; j > i; --j)
        {
            stack += GetProfileValue(latestProfile, entries[j].value);
        }

        float value = GetProfileValue(latestProfile, entries[i].value);
        ImU32 entryColor = ToImColor(entries[i].color);
        float lineY = legendY + textHeight * 0.5f;
        float stackY = plotMax.y - ((stack + value * 0.5f) - minValue) * scale;
        stackY = Clamp(stackY, plotMin.y, plotMax.y);

        drawList->AddLine(ImVec2{ plotMax.x, stackY }, ImVec2{ legendMin.x - 3.0f, lineY }, entryColor, 1.0f);
        drawList->AddRectFilled(
            ImVec2{ legendMin.x, legendY + 3.0f }, ImVec2{ legendMin.x + 10.0f, legendY + 13.0f }, entryColor
        );

        char text[128];
        std::snprintf(text, sizeof(text), "[%.3f ms] %s", value, entries[i].name);
        drawList->AddText(ImVec2{ legendMin.x + 14.0f, legendY }, entryColor, text);

        legendY += legendStep;
    }

    ImVec2 mousePosition = ImGui::GetIO().MousePos;
    bool plotHovered = ImGui::IsItemHovered() && mousePosition.x >= plotMin.x && mousePosition.x < plotMax.x &&
                       mousePosition.y >= plotMin.y && mousePosition.y < plotMax.y;

    if (showOverlay && plotHovered)
    {
        int32 hoverOffset = (int32)((mousePosition.x - plotMin.x) / columnStep);
        hoverOffset = Clamp(hoverOffset, 0, count - 1);
        int32 hoverIndex = (int32)((profileReadIndex + hoverOffset) & (profileCapacity - 1));
        const WorldProfile& hoverProfile = profiles[hoverIndex];

        float lineX = plotMin.x + columnStep * ((float)hoverOffset + 0.5f);
        drawList->AddLine(ImVec2{ lineX, plotMin.y }, ImVec2{ lineX, plotMax.y }, IM_COL32(255, 255, 255, 180), 1.0f);

        ImGui::BeginTooltip();
        ImGui::Text("-%d frames", count - hoverOffset - 1);
        ImGui::Separator();
        ImGui::Text("Total: %.3f ms", GetProfileTotal(hoverProfile, entries, entryCount));

        for (int32 i = 0; i < entryCount; ++i)
        {
            float value = GetProfileValue(hoverProfile, entries[i].value);
            ImGui::TextColored(
                ImVec4{
                    (float)((entries[i].color >> 16) & 0xff) / 255.0f,
                    (float)((entries[i].color >> 8) & 0xff) / 255.0f,
                    (float)(entries[i].color & 0xff) / 255.0f,
                    1.0f,
                },
                "%.3f ms %s", value, entries[i].name
            );
        }

        ImGui::EndTooltip();
    }
}

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
                    ImGui::Checkbox("Warm starting", &settings.step.warm_starting);
                    ImGui::Checkbox("Sleeping", &settings.sleeping);
                }

                ImGui::Separator();
                ImGui::Text("%s / %s", demoFrames[demoIndex].category, demoFrames[demoIndex].name);
                ImGui::Text("Bodies: %d", world.GetBodyCount());
                ImGui::Text("Sleeping dynamic bodies: %d", world.GetSleepingBodyCount());
                ImGui::Text("Broad phase contacts: %d", world.GetContactCount());
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
        if (ImGui::Begin("Profile", &options.show_profiler, ImGuiWindowFlags_AlwaysAutoResize))
        {
            int count = (int)(profileWriteIndex - profileReadIndex);

            static constexpr ProfileGraphEntry worldEntries[] = {
                { "Broad phase", color::broad_phase, profile_broad_phase },
                { "Narrow phase", color::narrow_phase, profile_narrow_phase },
                { "Build islands", color::build_islands, profile_build_islands },
                { "Integrate velocities", color::integrate_velocities, profile_integrate_velocities },
                { "Prepare constraints", color::prepare_constraints, profile_prepare_constraints },
                { "Solve velocity", color::solve_velocity, profile_solve_velocity },
                { "Integrate positions", color::integrate_positions, profile_integrate_positions },
                { "Solve position", color::solve_position, profile_solve_position },
                { "Update transforms", color::update_transforms, profile_update_transforms },
                { "Clear island flags", color::clear_island_flags, profile_clear_island_flags },
                { "Solve other", color::solve, profile_solve_other },
                { "Solve rest", color::solve, profile_solve_rest },
                { "Deferred destroy", color::deferred_destroy, profile_deferred_destroy },
                { "Other", color::step, profile_step_other },
            };

            DrawProfileGraph(
                "World Profile", profiles, { 420, 160 }, profile_capacity, profileReadIndex, count, worldEntries,
                (int32)(sizeof(worldEntries) / sizeof(worldEntries[0])), profileMaxRange, profileShowOverlay
            );

            ImGui::Checkbox("Stop", &profileStopped);
            ImGui::SameLine();
            ImGui::Checkbox("Overlay", &profileShowOverlay);
            ImGui::SameLine();
            if (ImGui::Button("Clear"))
            {
                profileReadIndex = 0;
                profileWriteIndex = 0;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            ImGui::SameLine();
            ImGui::SliderFloat("Max range", &profileMaxRange, 0.0f, 10.0f, "%.2f ms");
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
        previousCamera = demo->GetCamera();
    }

    delete demo;
    demo = nullptr;
    renderer.ClearMeshCache();

    time = 0.0f;
    demoIndex = index;
    demo = demoFrames[demoIndex].createFunction(*this);
    // profileReadIndex = 0;
    // profileWriteIndex = 0;
    // profileStopped = false;

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
