#pragma once

#include "common.h"

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

namespace color
{

inline constexpr uint32 step = 0x8CC924;
inline constexpr uint32 broad_phase = 0x7B68EE;
inline constexpr uint32 narrow_phase = 0x1E90FF;
inline constexpr uint32 solve = 0x4B0082;
inline constexpr uint32 deferred_destroy = 0xCD5C5C;
inline constexpr uint32 build_islands = 0xB0C4DE;
inline constexpr uint32 integrate_velocities = 0xFF1493;
inline constexpr uint32 prepare_constraints = 0xFDF5E6;
inline constexpr uint32 solve_velocity = 0xFFFACD;
inline constexpr uint32 integrate_positions = 0x8FBC8F;
inline constexpr uint32 solve_position = 0x00BFFF;
inline constexpr uint32 update_transforms = 0x3CB371;
inline constexpr uint32 clear_island_flags = 0x778899;

} // namespace color

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

} // namespace muli3
