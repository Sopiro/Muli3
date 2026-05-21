#pragma once

#include "color.h"

#ifdef MULI3_PROFILE
    // Tracy profiler (https://github.com/wolfpld/tracy)
    #include <tracy/TracyC.h>

    #define MuliProfileZoneC(ctx, color, active) TracyCZoneC(ctx, color, active)
    #define MuliProfileZoneNC(ctx, name, color, active) TracyCZoneNC(ctx, name, color, active)
    #define MuliProfileZoneEnd(ctx) TracyCZoneEnd(ctx)
    #define MuliProfileSetThreadName(name) TracyCSetThreadName(name)
    #define MuliProfileStartup() ___tracy_startup_profiler()
    #define MuliProfileShutdown() ___tracy_shutdown_profiler()
    #define MuliProfileFrameMark TracyCFrameMark
#else
    #define MuliProfileZoneC(ctx, color, active)
    #define MuliProfileZoneNC(ctx, name, color, active)
    #define MuliProfileZoneEnd(ctx)
    #define MuliProfileSetThreadName(name)
    #define MuliProfileStartup()
    #define MuliProfileShutdown()
    #define MuliProfileFrameMark
#endif

namespace muli3
{

inline void ProfileStartup()
{
    MuliProfileStartup();
}

inline void ProfileShutdown()
{
    MuliProfileShutdown();
}

inline void ProfileFrameMark()
{
    MuliProfileFrameMark;
}

struct WorldProfile
{
    float step = 0.0f;
    float broad_phase = 0.0f;
    float narrow_phase = 0.0f;
    float solve = 0.0f;
    float solve_world = 0.0f;
    float build_islands = 0.0f;
    float solve_islands = 0.0f;
    float integrate_velocities = 0.0f;
    float prepare_constraints = 0.0f;
    float solve_velocity = 0.0f;
    float integrate_positions = 0.0f;
    float solve_position = 0.0f;
    float update_transforms = 0.0f;
    float clear_island_flags = 0.0f;
    float deferred_destroy = 0.0f;
};

class ProfileScope
{
public:
    explicit ProfileScope(float* milliseconds)
        : milliseconds{ milliseconds }
        , start{ std::chrono::steady_clock::now() }
    {
    }

    ~ProfileScope()
    {
        if (milliseconds)
        {
            auto end = std::chrono::steady_clock::now();
            *milliseconds += std::chrono::duration<float, std::milli>(end - start).count();
        }
    }

private:
    float* milliseconds;
    std::chrono::steady_clock::time_point start;
};

namespace color
{

constexpr float offset = -0.1f;
constexpr float shuffle = 1.0f;
constexpr float saturation = 1.0f;
constexpr float lightness = 0.6f;

constexpr float Wrap01(float v)
{
    while (v >= 1.0f)
        v -= 1.0f;
    while (v < 0.0f)
        v += 1.0f;
    return v;
}

#define WORLD_PROFILE_COLOR(member)                                                                                              \
    RGBToHex(HSLToRGB(                                                                                                           \
        { Wrap01(shuffle * (offset + offsetof(WorldProfile, member) / float(sizeof(WorldProfile)))), saturation, lightness }     \
    ))

inline constexpr uint32 step = WORLD_PROFILE_COLOR(step);
inline constexpr uint32 broad_phase = WORLD_PROFILE_COLOR(broad_phase);
inline constexpr uint32 narrow_phase = WORLD_PROFILE_COLOR(narrow_phase);
inline constexpr uint32 solve = WORLD_PROFILE_COLOR(solve);
inline constexpr uint32 deferred_destroy = WORLD_PROFILE_COLOR(deferred_destroy);
inline constexpr uint32 build_islands = WORLD_PROFILE_COLOR(build_islands);
inline constexpr uint32 integrate_velocities = WORLD_PROFILE_COLOR(integrate_velocities);
inline constexpr uint32 prepare_constraints = WORLD_PROFILE_COLOR(prepare_constraints);
inline constexpr uint32 solve_velocity = WORLD_PROFILE_COLOR(solve_velocity);
inline constexpr uint32 integrate_positions = WORLD_PROFILE_COLOR(integrate_positions);
inline constexpr uint32 solve_position = WORLD_PROFILE_COLOR(solve_position);
inline constexpr uint32 update_transforms = WORLD_PROFILE_COLOR(update_transforms);
inline constexpr uint32 clear_island_flags = WORLD_PROFILE_COLOR(clear_island_flags);

#undef WORLD_PROFILE_COLOR

} // namespace color

} // namespace muli3
