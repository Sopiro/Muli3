#pragma once

#include "floats.h"

// Select SIMD instruction set for solver backend.
// The scalar backend preserves the 4-lane layout for determinism.

#if defined(MULI3_DISABLE_SIMD)
    #define MULI3_SIMD_NONE
#elif defined(__AVX2__)
    #define MULI3_SIMD_AVX2
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define MULI3_SIMD_NEON
#elif defined(__x86_64__) || defined(_M_X64) || defined(__SSE2__) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    #define MULI3_SIMD_SSE2
#else
    #define MULI3_SIMD_NONE
#endif

#if defined(MULI3_SIMD_AVX2)
    #define MULI3_SIMD_WIDTH 8
    #define MULI3_SIMD_ALIGNMENT 32
#else
    #define MULI3_SIMD_WIDTH 4
    #define MULI3_SIMD_ALIGNMENT 16
#endif

namespace muli3
{

inline constexpr int32 simd_width = MULI3_SIMD_WIDTH;
inline constexpr int32 simd_alignment = MULI3_SIMD_ALIGNMENT;

template <typename T>
struct PtrBlock
{
    T* lane[simd_width];
};

struct alignas(simd_alignment) FloatBlock
{
    Float lane[simd_width];
};

struct alignas(simd_alignment) IntBlock
{
    int32 lane[simd_width];
};

struct Vec2Block
{
    FloatBlock x, y;
};

struct Vec3Block
{
    FloatBlock x, y, z;
};

struct QuatBlock
{
    FloatBlock x, y, z, w;
};

struct Mat2Block
{
    Vec2Block ex, ey;
};

struct Mat3Block
{
    Vec3Block ex, ey, ez;
};

} // namespace muli3
