#pragma once

#include "color.h"

#ifdef MULI3_PROFILE
    // Tracy profiler (https://github.com/wolfpld/tracy)
    #include <tracy/TracyC.h>

    #define MuliProfileZoneNR(ctx, name, active) TracyCZoneNC(ctx, name, muli3::color::Random(name), active)

    #define MuliProfileZoneN(ctx, name, active) TracyCZoneN(ctx, name, active)
    #define MuliProfileZoneC(ctx, color, active) TracyCZoneC(ctx, color, active)
    #define MuliProfileZoneNC(ctx, name, color, active) TracyCZoneNC(ctx, name, color, active)
    #define MuliProfileZoneEnd(ctx) TracyCZoneEnd(ctx)
    #define MuliProfileSetThreadName(name) TracyCSetThreadName(name)
    #define MuliProfileStartup() ___tracy_startup_profiler()
    #define MuliProfileShutdown() ___tracy_shutdown_profiler()
    #define MuliProfileFrameMark TracyCFrameMark
#else
    #define MuliProfileZoneNR(ctx, name, active)
    #define MuliProfileZoneN(ctx, name, active)
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
    // Step profiles
    float broad_phase;
    float narrow_phase;
    float solve;

    // Solver profiles
    float build_islands;
    float integrate_velocities;
    float prepare_constraints;
    float warm_start;
    float solve_velocities;
    float integrate_positions;
    float solve_positions;
    float sleep_and_sync;
    float finalize;

    float post_solve;
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
        Stop();
    }

    void Stop()
    {
        if (milliseconds)
        {
            auto end = std::chrono::steady_clock::now();
            *milliseconds += std::chrono::duration<float, std::milli>(end - start).count();

            milliseconds = nullptr;
        }
    }

private:
    float* milliseconds;
    std::chrono::steady_clock::time_point start;
};

namespace color
{

constexpr float offset = 0.0f;
constexpr float cap = 1.0f;
constexpr float saturation = 1.0f;
constexpr float lightness = 0.62f;

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
        { cap * Wrap01((offset + (offsetof(WorldProfile, member)) / float(sizeof(WorldProfile)))), saturation, lightness }       \
    ))

inline constexpr uint32 broad_phase = WORLD_PROFILE_COLOR(broad_phase);
inline constexpr uint32 narrow_phase = WORLD_PROFILE_COLOR(narrow_phase);
inline constexpr uint32 solve = WORLD_PROFILE_COLOR(solve);
inline constexpr uint32 post_solve = WORLD_PROFILE_COLOR(post_solve);

inline constexpr uint32 build_islands = WORLD_PROFILE_COLOR(build_islands);
inline constexpr uint32 integrate_velocities = WORLD_PROFILE_COLOR(integrate_velocities);
inline constexpr uint32 prepare_constraints = WORLD_PROFILE_COLOR(prepare_constraints);
inline constexpr uint32 warm_start = WORLD_PROFILE_COLOR(warm_start);
inline constexpr uint32 solve_velocities = WORLD_PROFILE_COLOR(solve_velocities);
inline constexpr uint32 integrate_positions = WORLD_PROFILE_COLOR(integrate_positions);
inline constexpr uint32 solve_positions = WORLD_PROFILE_COLOR(solve_positions);
inline constexpr uint32 sleep_and_sync = WORLD_PROFILE_COLOR(sleep_and_sync);
inline constexpr uint32 finalize = WORLD_PROFILE_COLOR(finalize);

#undef WORLD_PROFILE_COLOR

} // namespace color

} // namespace muli3
