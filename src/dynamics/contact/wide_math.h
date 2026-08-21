#pragma once

#include "muli3/simd.h"

#if defined(MULI3_SIMD_AVX2)
    #include <immintrin.h>
#elif defined(MULI3_SIMD_NEON)
    #include <arm_neon.h>
#elif defined(MULI3_SIMD_SSE2)
    #include <emmintrin.h>
#endif

namespace muli3
{

#if defined(MULI3_SIMD_AVX2)

using FloatW = __m256;

inline FloatW LoadW(const Float* values)
{
    return _mm256_loadu_ps(values);
}
inline void StoreW(Float* values, FloatW a)
{
    _mm256_storeu_ps(values, a);
}
inline FloatW ZeroW()
{
    return _mm256_setzero_ps();
}
inline FloatW SplatW(Float value)
{
    return _mm256_set1_ps(value);
}
inline FloatW NegW(FloatW a)
{
    return _mm256_xor_ps(a, _mm256_set1_ps(-0.0f));
}
inline FloatW AddW(FloatW a, FloatW b)
{
    return _mm256_add_ps(a, b);
}
inline FloatW SubW(FloatW a, FloatW b)
{
    return _mm256_sub_ps(a, b);
}
inline FloatW MulW(FloatW a, FloatW b)
{
    return _mm256_mul_ps(a, b);
}
inline FloatW DivW(FloatW a, FloatW b)
{
    return _mm256_div_ps(a, b);
}
inline FloatW SqrtW(FloatW a)
{
    return _mm256_sqrt_ps(a);
}
inline FloatW MinW(FloatW a, FloatW b)
{
    return _mm256_min_ps(a, b);
}
inline FloatW MaxW(FloatW a, FloatW b)
{
    return _mm256_max_ps(a, b);
}
inline FloatW AndW(FloatW a, FloatW b)
{
    return _mm256_and_ps(a, b);
}
inline FloatW OrW(FloatW a, FloatW b)
{
    return _mm256_or_ps(a, b);
}
inline FloatW GreaterThanW(FloatW a, FloatW b)
{
    return _mm256_cmp_ps(a, b, _CMP_GT_OQ);
}
inline FloatW LessThanW(FloatW a, FloatW b)
{
    return _mm256_cmp_ps(a, b, _CMP_LT_OQ);
}
inline FloatW GreaterEqualW(FloatW a, FloatW b)
{
    return _mm256_cmp_ps(a, b, _CMP_GE_OQ);
}
inline FloatW LessEqualW(FloatW a, FloatW b)
{
    return _mm256_cmp_ps(a, b, _CMP_LE_OQ);
}
inline FloatW EqualsW(FloatW a, FloatW b)
{
    return _mm256_cmp_ps(a, b, _CMP_EQ_OQ);
}
inline FloatW BlendW(FloatW a, FloatW b, FloatW mask)
{
    return _mm256_or_ps(_mm256_and_ps(mask, b), _mm256_andnot_ps(mask, a));
}
inline int32 MoveMaskW(FloatW a)
{
    return _mm256_movemask_ps(a);
}

#elif defined(MULI3_SIMD_SSE2)

using FloatW = __m128;

inline FloatW LoadW(const Float* values)
{
    return _mm_loadu_ps(values);
}
inline void StoreW(Float* values, FloatW a)
{
    _mm_storeu_ps(values, a);
}
inline FloatW ZeroW()
{
    return _mm_setzero_ps();
}
inline FloatW SplatW(Float value)
{
    return _mm_set1_ps(value);
}
inline FloatW NegW(FloatW a)
{
    return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
}
inline FloatW AddW(FloatW a, FloatW b)
{
    return _mm_add_ps(a, b);
}
inline FloatW SubW(FloatW a, FloatW b)
{
    return _mm_sub_ps(a, b);
}
inline FloatW MulW(FloatW a, FloatW b)
{
    return _mm_mul_ps(a, b);
}
inline FloatW DivW(FloatW a, FloatW b)
{
    return _mm_div_ps(a, b);
}
inline FloatW SqrtW(FloatW a)
{
    return _mm_sqrt_ps(a);
}
inline FloatW MinW(FloatW a, FloatW b)
{
    return _mm_min_ps(a, b);
}
inline FloatW MaxW(FloatW a, FloatW b)
{
    return _mm_max_ps(a, b);
}
inline FloatW AndW(FloatW a, FloatW b)
{
    return _mm_and_ps(a, b);
}
inline FloatW OrW(FloatW a, FloatW b)
{
    return _mm_or_ps(a, b);
}
inline FloatW GreaterThanW(FloatW a, FloatW b)
{
    return _mm_cmpgt_ps(a, b);
}
inline FloatW LessThanW(FloatW a, FloatW b)
{
    return _mm_cmplt_ps(a, b);
}
inline FloatW GreaterEqualW(FloatW a, FloatW b)
{
    return _mm_cmpge_ps(a, b);
}
inline FloatW LessEqualW(FloatW a, FloatW b)
{
    return _mm_cmple_ps(a, b);
}
inline FloatW EqualsW(FloatW a, FloatW b)
{
    return _mm_cmpeq_ps(a, b);
}
inline FloatW BlendW(FloatW a, FloatW b, FloatW mask)
{
    return _mm_or_ps(_mm_and_ps(mask, b), _mm_andnot_ps(mask, a));
}
inline int32 MoveMaskW(FloatW a)
{
    return _mm_movemask_ps(a);
}

#elif defined(MULI3_SIMD_NEON)

using FloatW = float32x4_t;

inline FloatW LoadW(const Float* values)
{
    return vld1q_f32(values);
}
inline void StoreW(Float* values, FloatW a)
{
    vst1q_f32(values, a);
}
inline FloatW ZeroW()
{
    return vdupq_n_f32(0.0f);
}
inline FloatW SplatW(Float value)
{
    return vdupq_n_f32(value);
}
inline FloatW NegW(FloatW a)
{
    return vnegq_f32(a);
}
inline FloatW AddW(FloatW a, FloatW b)
{
    return vaddq_f32(a, b);
}
inline FloatW SubW(FloatW a, FloatW b)
{
    return vsubq_f32(a, b);
}
inline FloatW MulW(FloatW a, FloatW b)
{
    return vmulq_f32(a, b);
}
inline FloatW DivW(FloatW a, FloatW b)
{
    return vdivq_f32(a, b);
}
inline FloatW SqrtW(FloatW a)
{
    return vsqrtq_f32(a);
}
inline FloatW MinW(FloatW a, FloatW b)
{
    return vminq_f32(a, b);
}
inline FloatW MaxW(FloatW a, FloatW b)
{
    return vmaxq_f32(a, b);
}
inline FloatW AndW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)));
}
inline FloatW OrW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)));
}
inline FloatW GreaterThanW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vcgtq_f32(a, b));
}
inline FloatW LessThanW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vcltq_f32(a, b));
}
inline FloatW GreaterEqualW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vcgeq_f32(a, b));
}
inline FloatW LessEqualW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vcleq_f32(a, b));
}
inline FloatW EqualsW(FloatW a, FloatW b)
{
    return vreinterpretq_f32_u32(vceqq_f32(a, b));
}
inline FloatW BlendW(FloatW a, FloatW b, FloatW mask)
{
    return vbslq_f32(vreinterpretq_u32_f32(mask), b, a);
}
inline int32 MoveMaskW(FloatW a)
{
    uint32x4_t sign = vshrq_n_u32(vreinterpretq_u32_f32(a), 31);
    return int32(
        vgetq_lane_u32(sign, 0) | (vgetq_lane_u32(sign, 1) << 1) | (vgetq_lane_u32(sign, 2) << 2) | (vgetq_lane_u32(sign, 3) << 3)
    );
}

#else

// Scalar fallback

static constexpr Float true_value = std::bit_cast<Float>(~uint32(0));

struct alignas(simd_alignment) FloatW
{
    Float lane[simd_width];
};

inline FloatW ZeroW()
{
    return {};
}
inline FloatW SplatW(Float value)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = value;
    }
    return result;
}
inline FloatW LoadW(const Float* values)
{
    FloatW result;
    memcpy(result.lane, values, sizeof(result.lane));
    return result;
}
inline void StoreW(Float* values, FloatW a)
{
    memcpy(values, a.lane, sizeof(a.lane));
}
inline FloatW NegW(FloatW a)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = -a.lane[i];
    }
    return result;
}
inline FloatW AddW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] + b.lane[i];
    }
    return result;
}
inline FloatW SubW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] - b.lane[i];
    }
    return result;
}
inline FloatW MulW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] * b.lane[i];
    }
    return result;
}
inline FloatW DivW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] / b.lane[i];
    }
    return result;
}
inline FloatW SqrtW(FloatW a)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = std::sqrt(a.lane[i]);
    }
    return result;
}
inline FloatW MinW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] <= b.lane[i] ? a.lane[i] : b.lane[i];
    }
    return result;
}
inline FloatW MaxW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] >= b.lane[i] ? a.lane[i] : b.lane[i];
    }
    return result;
}
inline FloatW AndW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = std::bit_cast<Float>(std::bit_cast<uint32>(a.lane[i]) & std::bit_cast<uint32>(b.lane[i]));
    }
    return result;
}
inline FloatW OrW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = std::bit_cast<Float>(std::bit_cast<uint32>(a.lane[i]) | std::bit_cast<uint32>(b.lane[i]));
    }
    return result;
}
inline FloatW GreaterThanW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] > b.lane[i] ? true_value : 0.0f;
    }
    return result;
}
inline FloatW LessThanW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] < b.lane[i] ? true_value : 0.0f;
    }
    return result;
}
inline FloatW GreaterEqualW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] >= b.lane[i] ? true_value : 0.0f;
    }
    return result;
}
inline FloatW LessEqualW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] <= b.lane[i] ? true_value : 0.0f;
    }
    return result;
}
inline FloatW EqualsW(FloatW a, FloatW b)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        result.lane[i] = a.lane[i] == b.lane[i] ? true_value : 0.0f;
    }
    return result;
}
inline FloatW BlendW(FloatW a, FloatW b, FloatW mask)
{
    FloatW result;
    for (int32 i = 0; i < simd_width; ++i)
    {
        uint32 maskBits = std::bit_cast<uint32>(mask.lane[i]);
        result.lane[i] =
            std::bit_cast<Float>((maskBits & std::bit_cast<uint32>(b.lane[i])) | (~maskBits & std::bit_cast<uint32>(a.lane[i])));
    }
    return result;
}
inline int32 MoveMaskW(FloatW a)
{
    int32 mask = 0;
    for (int32 i = 0; i < simd_width; ++i)
    {
        mask |= int32(std::bit_cast<uint32>(a.lane[i]) >> 31) << i;
    }
    return mask;
}

#endif

inline FloatW LoadW(const FloatBlock& block)
{
    return LoadW(block.lane);
}
inline void StoreW(FloatBlock* block, FloatW value)
{
    StoreW(block->lane, value);
}

inline FloatW ClampW(FloatW a, FloatW left, FloatW right)
{
    return MinW(MaxW(a, left), right);
}

struct Vec2W
{
    FloatW x, y;
};

struct Vec3W
{
    FloatW x, y, z;
};

struct Mat2W
{
    Vec2W ex, ey;
};

struct Mat3W
{
    Vec3W ex, ey, ez;
};

struct QuatW
{
    FloatW x, y, z, w;
};

inline Vec2W LoadW(const Vec2Block& block)
{
    return { LoadW(block.x), LoadW(block.y) };
}

inline Vec3W LoadW(const Vec3Block& block)
{
    return { LoadW(block.x), LoadW(block.y), LoadW(block.z) };
}

inline QuatW LoadW(const QuatBlock& block)
{
    return { LoadW(block.x), LoadW(block.y), LoadW(block.z), LoadW(block.w) };
}

inline Mat2W LoadW(const Mat2Block& block)
{
    return { LoadW(block.ex), LoadW(block.ey) };
}

inline Mat3W LoadW(const Mat3Block& block)
{
    return { LoadW(block.ex), LoadW(block.ey), LoadW(block.ez) };
}

inline void StoreW(Vec2Block* block, Vec2W value)
{
    StoreW(&block->x, value.x);
    StoreW(&block->y, value.y);
}

inline void StoreW(Vec3Block* block, Vec3W value)
{
    StoreW(&block->x, value.x);
    StoreW(&block->y, value.y);
    StoreW(&block->z, value.z);
}

inline void StoreW(QuatBlock* block, QuatW value)
{
    StoreW(&block->x, value.x);
    StoreW(&block->y, value.y);
    StoreW(&block->z, value.z);
    StoreW(&block->w, value.w);
}

inline void StoreW(Mat2Block* block, Mat2W value)
{
    StoreW(&block->ex, value.ex);
    StoreW(&block->ey, value.ey);
}

inline void StoreW(Mat3Block* block, Mat3W value)
{
    StoreW(&block->ex, value.ex);
    StoreW(&block->ey, value.ey);
    StoreW(&block->ez, value.ez);
}

inline Vec2W operator-(Vec2W v)
{
    return { NegW(v.x), NegW(v.y) };
}

inline Vec2W operator+(Vec2W a, Vec2W b)
{
    return { AddW(a.x, b.x), AddW(a.y, b.y) };
}

inline Vec2W operator-(Vec2W a, Vec2W b)
{
    return { SubW(a.x, b.x), SubW(a.y, b.y) };
}

inline Vec2W operator*(FloatW s, Vec2W v)
{
    return { MulW(s, v.x), MulW(s, v.y) };
}

inline Vec2W operator*(Vec2W v, FloatW s)
{
    return s * v;
}

inline Vec2W operator/(Vec2W v, FloatW s)
{
    return { DivW(v.x, s), DivW(v.y, s) };
}

inline Vec2W MulAdd(Vec2W a, FloatW s, Vec2W b)
{
    return { AddW(a.x, MulW(s, b.x)), AddW(a.y, MulW(s, b.y)) };
}

inline Vec2W MulSub(Vec2W a, FloatW s, Vec2W b)
{
    return { SubW(a.x, MulW(s, b.x)), SubW(a.y, MulW(s, b.y)) };
}

inline FloatW Dot(Vec2W a, Vec2W b)
{
    return AddW(MulW(a.x, b.x), MulW(a.y, b.y));
}

inline Vec3W operator-(Vec3W v)
{
    return { NegW(v.x), NegW(v.y), NegW(v.z) };
}

inline Vec3W operator+(Vec3W a, Vec3W b)
{
    return { AddW(a.x, b.x), AddW(a.y, b.y), AddW(a.z, b.z) };
}

inline Vec3W operator-(Vec3W a, Vec3W b)
{
    return { SubW(a.x, b.x), SubW(a.y, b.y), SubW(a.z, b.z) };
}

inline Vec3W operator*(FloatW s, Vec3W v)
{
    return { MulW(s, v.x), MulW(s, v.y), MulW(s, v.z) };
}

inline Vec3W operator*(Vec3W v, FloatW s)
{
    return s * v;
}

inline Vec3W operator/(Vec3W v, FloatW s)
{
    return { DivW(v.x, s), DivW(v.y, s), DivW(v.z, s) };
}

inline Vec3W MulAdd(Vec3W a, FloatW s, Vec3W b)
{
    return { AddW(a.x, MulW(s, b.x)), AddW(a.y, MulW(s, b.y)), AddW(a.z, MulW(s, b.z)) };
}

inline Vec3W MulSub(Vec3W a, FloatW s, Vec3W b)
{
    return { SubW(a.x, MulW(s, b.x)), SubW(a.y, MulW(s, b.y)), SubW(a.z, MulW(s, b.z)) };
}

inline FloatW Dot(Vec3W a, Vec3W b)
{
    return AddW(AddW(MulW(a.x, b.x), MulW(a.y, b.y)), MulW(a.z, b.z));
}

inline Vec3W Cross(Vec3W a, Vec3W b)
{
    return {
        SubW(MulW(a.y, b.z), MulW(a.z, b.y)),
        SubW(MulW(a.z, b.x), MulW(a.x, b.z)),
        SubW(MulW(a.x, b.y), MulW(a.y, b.x)),
    };
}

inline Vec2W Mul(Mat2W m, Vec2W v)
{
    return {
        AddW(MulW(m.ex.x, v.x), MulW(m.ey.x, v.y)),
        AddW(MulW(m.ex.y, v.x), MulW(m.ey.y, v.y)),
    };
}

inline Vec3W Mul(Mat3W m, Vec3W v)
{
    return {
        AddW(AddW(MulW(m.ex.x, v.x), MulW(m.ey.x, v.y)), MulW(m.ez.x, v.z)),
        AddW(AddW(MulW(m.ex.y, v.x), MulW(m.ey.y, v.y)), MulW(m.ez.y, v.z)),
        AddW(AddW(MulW(m.ex.z, v.x), MulW(m.ey.z, v.y)), MulW(m.ez.z, v.z)),
    };
}

inline Vec3W MulAdd(Vec3W a, Mat3W m, Vec3W b)
{
    return a + Mul(m, b);
}

inline Vec3W MulSub(Vec3W a, Mat3W m, Vec3W b)
{
    return a - Mul(m, b);
}

inline QuatW operator-(QuatW q)
{
    return { NegW(q.x), NegW(q.y), NegW(q.z), NegW(q.w) };
}

inline QuatW operator+(QuatW a, QuatW b)
{
    return { AddW(a.x, b.x), AddW(a.y, b.y), AddW(a.z, b.z), AddW(a.w, b.w) };
}

inline QuatW operator-(QuatW a, QuatW b)
{
    return { SubW(a.x, b.x), SubW(a.y, b.y), SubW(a.z, b.z), SubW(a.w, b.w) };
}

inline QuatW operator*(FloatW s, QuatW q)
{
    return { MulW(s, q.x), MulW(s, q.y), MulW(s, q.z), MulW(s, q.w) };
}

inline QuatW operator*(QuatW q, FloatW s)
{
    return s * q;
}

inline QuatW operator/(QuatW q, FloatW s)
{
    return { DivW(q.x, s), DivW(q.y, s), DivW(q.z, s), DivW(q.w, s) };
}

inline FloatW Dot(QuatW a, QuatW b)
{
    return AddW(AddW(AddW(MulW(a.x, b.x), MulW(a.y, b.y)), MulW(a.z, b.z)), MulW(a.w, b.w));
}

inline QuatW operator*(QuatW a, QuatW b)
{
    return {
        SubW(AddW(AddW(MulW(a.w, b.x), MulW(b.w, a.x)), MulW(a.y, b.z)), MulW(b.y, a.z)),
        SubW(AddW(AddW(MulW(a.w, b.y), MulW(b.w, a.y)), MulW(a.z, b.x)), MulW(b.z, a.x)),
        SubW(AddW(AddW(MulW(a.w, b.z), MulW(b.w, a.z)), MulW(a.x, b.y)), MulW(b.x, a.y)),
        SubW(SubW(SubW(MulW(a.w, b.w), MulW(a.x, b.x)), MulW(a.y, b.y)), MulW(a.z, b.z)),
    };
}

inline QuatW Normalize(QuatW q)
{
    return q / SqrtW(Dot(q, q));
}

inline Vec3W Rotate(QuatW q, Vec3W v)
{
    FloatW two = SplatW(2.0f);
    FloatW half = SplatW(0.5f);

    FloatW vx = MulW(two, v.x);
    FloatW vy = MulW(two, v.y);
    FloatW vz = MulW(two, v.z);
    FloatW w2 = SubW(MulW(q.w, q.w), half);
    FloatW dot2 = AddW(AddW(MulW(q.x, vx), MulW(q.y, vy)), MulW(q.z, vz));

    return {
        AddW(AddW(MulW(vx, w2), MulW(SubW(MulW(q.y, vz), MulW(q.z, vy)), q.w)), MulW(q.x, dot2)),
        AddW(AddW(MulW(vy, w2), MulW(SubW(MulW(q.z, vx), MulW(q.x, vz)), q.w)), MulW(q.y, dot2)),
        AddW(AddW(MulW(vz, w2), MulW(SubW(MulW(q.x, vy), MulW(q.y, vx)), q.w)), MulW(q.z, dot2)),
    };
}

inline Vec3W RotateInv(QuatW q, Vec3W v)
{
    FloatW two = SplatW(2.0f);
    FloatW half = SplatW(0.5f);

    FloatW vx = MulW(two, v.x);
    FloatW vy = MulW(two, v.y);
    FloatW vz = MulW(two, v.z);
    FloatW w2 = SubW(MulW(q.w, q.w), half);
    FloatW dot2 = AddW(AddW(MulW(q.x, vx), MulW(q.y, vy)), MulW(q.z, vz));

    return {
        AddW(SubW(MulW(vx, w2), MulW(SubW(MulW(q.y, vz), MulW(q.z, vy)), q.w)), MulW(q.x, dot2)),
        AddW(SubW(MulW(vy, w2), MulW(SubW(MulW(q.z, vx), MulW(q.x, vz)), q.w)), MulW(q.y, dot2)),
        AddW(SubW(MulW(vz, w2), MulW(SubW(MulW(q.x, vy), MulW(q.y, vx)), q.w)), MulW(q.z, dot2)),
    };
}

inline QuatW Blend(QuatW a, QuatW b, FloatW mask)
{
    return { BlendW(a.x, b.x, mask), BlendW(a.y, b.y, mask), BlendW(a.z, b.z, mask), BlendW(a.w, b.w, mask) };
}

inline Mat3W ComputeWorldInvInertia(QuatW rotation, Mat3W localInvInertia)
{
    Vec3W basisX{ SplatW(1.0f), ZeroW(), ZeroW() };
    Vec3W basisY{ ZeroW(), SplatW(1.0f), ZeroW() };
    Vec3W basisZ{ ZeroW(), ZeroW(), SplatW(1.0f) };
    return {
        Rotate(rotation, Mul(localInvInertia, RotateInv(rotation, basisX))),
        Rotate(rotation, Mul(localInvInertia, RotateInv(rotation, basisY))),
        Rotate(rotation, Mul(localInvInertia, RotateInv(rotation, basisZ))),
    };
}

inline void CoordinateSystemW(Vec3W normal, Vec3W* tangent1, Vec3W* tangent2)
{
    FloatW zero = ZeroW();
    FloatW one = SplatW(1.0f);

    FloatW sign = BlendW(one, NegW(one), LessThanW(normal.z, zero));
    FloatW a = DivW(NegW(one), AddW(sign, normal.z));
    FloatW b = MulW(MulW(normal.x, normal.y), a);
    *tangent1 = {
        AddW(one, MulW(MulW(sign, MulW(normal.x, normal.x)), a)),
        MulW(sign, b),
        NegW(MulW(sign, normal.x)),
    };
    *tangent2 = {
        b,
        AddW(sign, MulW(MulW(normal.y, normal.y), a)),
        NegW(normal.y),
    };
}

inline Mat2W InverseW(FloatW k11, FloatW k12, FloatW k22)
{
    FloatW zero = ZeroW();
    FloatW one = SplatW(1.0f);

    FloatW determinant = SubW(MulW(k11, k22), MulW(k12, k12));
    FloatW valid = GreaterThanW(determinant, zero);
    FloatW safeDeterminant = BlendW(one, determinant, valid);
    FloatW invDeterminant = BlendW(zero, DivW(one, safeDeterminant), valid);

    return {
        { MulW(k22, invDeterminant), NegW(MulW(k12, invDeterminant)) },
        { NegW(MulW(k12, invDeterminant)), MulW(k11, invDeterminant) },
    };
}

} // namespace muli3
