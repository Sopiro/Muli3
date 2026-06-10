#pragma once

#include "math.h"

namespace muli3::color
{

constexpr inline float HueToRGB(float p, float q, float t)
{
    if (t < 0.0f)
    {
        t += 1.0f;
    }
    else if (t > 1.0f)
    {
        t -= 1.0f;
    }

    if (t < 1.0f / 6.0f)
    {
        return p + (q - p) * 6.0f * t;
    }
    else if (t < 1.0f / 2.0f)
    {
        return q;
    }
    else if (t < 2.0f / 3.0f)
    {
        return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
    }

    return p;
}

constexpr inline Vec3 HSLToRGB(const Vec3& hsl)
{
    Vec3 result;

    if (hsl.y == 0.0f)
    {
        result.x = result.y = result.z = hsl.z;
    }
    else
    {
        float q = hsl.z < 0.5f ? hsl.z * (1.0f + hsl.y) : hsl.z + hsl.y - hsl.z * hsl.y;
        float p = 2.0f * hsl.z - q;
        result.x = HueToRGB(p, q, hsl.x + 1.0f / 3.0f);
        result.y = HueToRGB(p, q, hsl.x);
        result.z = HueToRGB(p, q, hsl.x - 1.0f / 3.0f);
    }

    return result;
}

constexpr inline uint32 RGBToHex(const Vec3& rgb)
{
    uint32 r = std::min<uint32>(uint32(rgb.x * 256), 255);
    uint32 g = std::min<uint32>(uint32(rgb.y * 256), 255);
    uint32 b = std::min<uint32>(uint32(rgb.z * 256), 255);

    return (uint32(r) << 16) | (uint32(g) << 8) | uint32(b);
}

constexpr inline uint32 RGBToHex(const Vec4& rgb)
{
    uint32 r = std::min<uint32>(uint32(rgb.x * 256), 255);
    uint32 g = std::min<uint32>(uint32(rgb.y * 256), 255);
    uint32 b = std::min<uint32>(uint32(rgb.z * 256), 255);

    return (uint32(r) << 16) | (uint32(g) << 8) | uint32(b);
}

inline constexpr uint32 Random(uint32 seed)
{
    uint32 h = seed + 0x9E3779B9u;
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    h *= 0x846CA68Bu;
    h ^= h >> 16;

    float r = h / float(UINT32_MAX);

    return RGBToHex(HSLToRGB({ r, 0.9f, 0.6f }));
}

inline constexpr uint32 Random(const char* seed)
{
    uint32 h = 2166136261u;
    while (*seed)
    {
        h ^= uint32(*seed++);
        h *= 16777619u;
    }

    return Random(h);
}

} // namespace muli3::color
