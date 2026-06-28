#pragma once

#include "math.h"

namespace muli3
{

struct NoiseParams
{
    Float frequency = 1.0f;
    Float amplitude = 1.0f;
    int32 octaves = 4;
    Float lacunarity = 2.0f;
    Float gain = 0.5f;
    uint32 seed = 0;
};

// Coherent 2D gradient noise in roughly [-1, 1].
Float Noise2(Float x, Float z, uint32 seed = 0);

// Sum multiple octaves of Noise2 and normalize the result to roughly [-1, 1].
Float FractalNoise2(Float x, Float z, int32 octaves = 4, Float lacunarity = 2.0f, Float gain = 0.5f, uint32 seed = 0);

// Absolute-value fractal noise useful for rough terrain details.
Float TurbulenceNoise2(Float x, Float z, int32 octaves = 4, Float lacunarity = 2.0f, Float gain = 0.5f, uint32 seed = 0);

// Inverted turbulence that forms ridge-like terrain features.
Float RidgedNoise2(
    Float x, Float z, int32 octaves = 4, Float lacunarity = 2.0f, Float gain = 0.5f, Float offset = 1.0f, uint32 seed = 0
);

// Convenience helper for terrain: frequency scales the coordinates and amplitude scales the result.
Float HeightNoise2(Float x, Float z, const NoiseParams& params);

} // namespace muli3
