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
    float step;
    float broad_phase;
    float narrow_phase;
    float solve;
    float build_islands;
    float solve_islands;
    float integrate_velocities;
    float prepare_constraints;
    float solve_velocity;
    float integrate_positions;
    float solve_position;
    float sync_transforms;
    float finalize;
    float deferred_destroy;
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
inline constexpr uint32 solve_islands = WORLD_PROFILE_COLOR(solve_islands);
inline constexpr uint32 integrate_velocities = WORLD_PROFILE_COLOR(integrate_velocities);
inline constexpr uint32 prepare_constraints = WORLD_PROFILE_COLOR(prepare_constraints);
inline constexpr uint32 solve_velocity = WORLD_PROFILE_COLOR(solve_velocity);
inline constexpr uint32 integrate_positions = WORLD_PROFILE_COLOR(integrate_positions);
inline constexpr uint32 solve_position = WORLD_PROFILE_COLOR(solve_position);
inline constexpr uint32 sync_transforms = WORLD_PROFILE_COLOR(sync_transforms);
inline constexpr uint32 finalize = WORLD_PROFILE_COLOR(finalize);

#undef WORLD_PROFILE_COLOR

inline constexpr uint32 random(uint32 seed)
{
    uint32 h = seed + 0x9E3779B9u;
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    h *= 0x846CA68Bu;
    h ^= h >> 16;

    uint32 r = 80u + (h & 0x7Fu);
    uint32 g = 80u + ((h >> 8) & 0x7Fu);
    uint32 b = 80u + ((h >> 16) & 0x7Fu);

    return (r << 16) | (g << 8) | b;
}

} // namespace color

} // namespace muli3
