#include "muli3/noise.h"

namespace muli3
{

static uint32 Hash(uint32 x, uint32 z, uint32 seed)
{
    // Stable pseudo-random value for an integer lattice point.
    uint32 h = seed + 0x9e3779b9u;
    h ^= x + 0x85ebca6bu + (h << 6) + (h >> 2);
    h ^= z + 0xc2b2ae35u + (h << 6) + (h >> 2);
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

static Float Grad(uint32 hash, Float x, Float z)
{
    constexpr float d = Float(0.7071067811865475); // 1 / sqrt(2)

    // Project the local offset onto one of 8 gradient directions.
    switch (hash & 7)
    {
    case 0:
        return x;
    case 1:
        return -x;
    case 2:
        return z;
    case 3:
        return -z;
    case 4:
        return (x + z) * d;
    case 5:
        return (-x + z) * d;
    case 6:
        return (x - z) * d;
    default:
        return (-x - z) * d;
    }
}

Float Noise2(Float x, Float z, uint32 seed)
{
    // Sample the four lattice corners around the point and blend them.
    int32 x0 = int32(std::floor(x));
    int32 z0 = int32(std::floor(z));
    int32 x1 = x0 + 1;
    int32 z1 = z0 + 1;

    Float tx = x - x0;
    Float tz = z - z0;
    Float u = SmootherStep01(tx);
    Float v = SmootherStep01(tz);

    Float n00 = Grad(Hash((uint32)x0, (uint32)z0, seed), tx, tz);
    Float n10 = Grad(Hash((uint32)x1, (uint32)z0, seed), tx - 1, tz);
    Float n01 = Grad(Hash((uint32)x0, (uint32)z1, seed), tx, tz - 1);
    Float n11 = Grad(Hash((uint32)x1, (uint32)z1, seed), tx - 1, tz - 1);

    Float nx0 = Lerp(n00, n10, u);
    Float nx1 = Lerp(n01, n11, u);
    return Clamp(Lerp(nx0, nx1, v) * Float(1.4142135623730951), -1, 1);
}

Float FractalNoise2(Float x, Float z, int32 octaves, Float lacunarity, Float gain, uint32 seed)
{
    // Layer higher frequencies with lower amplitudes, then normalize.
    Float sum = 0;
    Float amplitude = 1;
    Float amplitudeSum = 0;
    Float frequency = 1;

    for (int32 i = 0; i < octaves; ++i)
    {
        sum += Noise2(x * frequency, z * frequency, seed + (uint32)i) * amplitude;
        amplitudeSum += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }

    return amplitudeSum > 0 ? sum / amplitudeSum : 0;
}

Float TurbulenceNoise2(Float x, Float z, int32 octaves, Float lacunarity, Float gain, uint32 seed)
{
    // Fold negative values upward to make sharper rolling terrain.
    Float sum = 0;
    Float amplitude = 1;
    Float amplitudeSum = 0;
    Float frequency = 1;

    for (int32 i = 0; i < octaves; ++i)
    {
        sum += Abs(Noise2(x * frequency, z * frequency, seed + (uint32)i)) * amplitude;
        amplitudeSum += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }

    return amplitudeSum > 0 ? sum / amplitudeSum : 0;
}

Float RidgedNoise2(Float x, Float z, int32 octaves, Float lacunarity, Float gain, Float offset, uint32 seed)
{
    // Invert folded noise so high values form ridge lines.
    Float sum = 0;
    Float amplitude = 0.5f;
    Float frequency = 1;
    Float previous = 1;

    for (int32 i = 0; i < octaves; ++i)
    {
        Float n = offset - Abs(Noise2(x * frequency, z * frequency, seed + (uint32)i));
        n *= n;
        sum += n * amplitude * previous;
        previous = n;
        amplitude *= gain;
        frequency *= lacunarity;
    }

    return Clamp(sum, 0, 1);
}

Float HeightNoise2(Float x, Float z, const NoiseParams& params)
{
    // Convenience wrapper for height fields.
    return params.amplitude *
           FractalNoise2(x * params.frequency, z * params.frequency, params.octaves, params.lacunarity, params.gain, params.seed);
}

} // namespace muli3
