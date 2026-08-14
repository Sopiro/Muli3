#include "renderer.h"
#include "muli3/color.h"
#include "muli3/frame.h"

namespace muli3
{

static constexpr size_t g_max_shape_batch_count = 4096;
static constexpr int32 g_max_vertex_count = 1024 * 4;

static constexpr int32 g_fill_pass = 0;
static constexpr int32 g_outline_pass = 1;

static constexpr int32 g_sample_count = 4;

static constexpr float g_metallic = 0.0f;
static constexpr float g_roughness = 0.55f;

static constexpr int g_shadow_map_size = 4096;
static constexpr float g_shadow_view_distance = 55.0f;
static constexpr float g_shadow_bounds_padding = 2.0f;
static constexpr float g_shadow_depth_padding = 32.0f;

static constexpr int32 g_color_count = 10;

Vec4 g_colors[g_color_count];
Vec4 g_colors2[constraint_color_count];
bool g_colorsInitialized = false;

uint32 ComputeHilbertIndex(uint32 x, uint32 y)
{
    constexpr uint32 width = 64;
    uint32 index = 0;
    for (uint32 level = width / 2; level > 0; level /= 2)
    {
        uint32 regionX = (x & level) != 0 ? 1 : 0;
        uint32 regionY = (y & level) != 0 ? 1 : 0;
        index += level * level * ((3 * regionX) ^ regionY);
        if (regionY == 0)
        {
            if (regionX == 1)
            {
                x = width - 1 - x;
                y = width - 1 - y;
            }
            std::swap(x, y);
        }
    }
    return index;
}

float SrgbToLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
}

void AppendStaticBuffer(GLenum target, GLuint buffer, const void* data, size_t oldSize, size_t newSize, size_t* capacity)
{
    glBindBuffer(target, buffer);
    if (newSize > *capacity)
    {
        *capacity = Max<size_t>(newSize, Max<size_t>(*capacity * 2, 4096));
        glBufferData(target, (GLsizeiptr)*capacity, nullptr, GL_STATIC_DRAW);
        glBufferSubData(target, 0, (GLsizeiptr)newSize, data);
    }
    else if (newSize > oldSize)
    {
        const uint8* bytes = (const uint8*)data;
        glBufferSubData(target, (GLintptr)oldSize, (GLsizeiptr)(newSize - oldSize), bytes + oldSize);
    }
}

constexpr const char* g_shapeVertexShader = R"(
#version 330 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

layout (location = 3) in vec4 iModel0;
layout (location = 4) in vec4 iModel1;
layout (location = 5) in vec4 iModel2;
layout (location = 6) in vec4 iModel3;

layout (location = 7) in vec4 iColor;

layout (location = 8) in vec3 iNormal0;
layout (location = 9) in vec3 iNormal1;
layout (location = 10) in vec3 iNormal2;

uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uLightViewProjection;

out vec3 vWorldPosition;
out vec3 vWorldNormal;
out vec2 vTexCoord;
out vec4 vShadowPosition;
out vec3 vBaseColor;

void main()
{
    mat4 model = mat4(iModel0, iModel1, iModel2, iModel3);

    vec4 worldPosition = model * vec4(aPosition, 1.0);
    vec3 worldNormal = normalize(mat3(iNormal0, iNormal1, iNormal2) * aNormal);

    vWorldPosition = worldPosition.xyz;
    vWorldNormal = worldNormal;
    vTexCoord = aTexCoord;

    vShadowPosition = uLightViewProjection * worldPosition;
    vBaseColor = iColor.rgb;

    gl_Position = uProjection * uView * worldPosition;
}
)";

constexpr const char* g_shapeFragmentShaderHeader = R"(
#version 330 core

in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vShadowPosition;
in vec3 vBaseColor;

uniform vec3 uLightDirection;
uniform vec3 uCameraPosition;
uniform sampler2DShadow uShadowMap;

out vec4 FragColor;

vec3 SrgbToLinear(vec3 color)
{
    bvec3 cutoff = lessThanEqual(color, vec3(0.04045));
    vec3 lower = color / 12.92;
    vec3 higher = pow((color + 0.055) / 1.055, vec3(2.4));

    return mix(higher, lower, cutoff);
}

float CheckerMask(vec2 uv)
{
    vec2 cell = fract(uv);

    float checker = mod(floor(uv.x) + floor(uv.y), 2.0);
    float edgeDistance = min(min(cell.x, 1.0 - cell.x), min(cell.y, 1.0 - cell.y));
    float filterWidth = max(max(fwidth(uv.x), fwidth(uv.y)) * 0.35, 0.0001);

    return mix(0.5, checker, smoothstep(0.0, filterWidth, edgeDistance));
}
)";

constexpr const char* g_pbrFragmentShader = R"(
uniform vec3 uLightColor;
uniform float uLightIntensity;
uniform vec3 uSkyColor;
uniform float uMetallic;
uniform float uRoughness;

const float pi = 3.14159265358979323846;

float D_TrowbridgeReitz(float NoH, float alpha2)
{
    float d = NoH * NoH * (alpha2 - 1.0) + 1.0;
    return alpha2 / max(pi * d * d, 0.0000001);
}

float G_SmithCorrelated(float NoV, float NoL, float alpha2)
{
    float k = 1.0 - alpha2;
    float lambdaV = sqrt(alpha2 + k * NoV * NoV);
    float lambdaL = sqrt(alpha2 + k * NoL * NoL);
    return 2.0 * NoV * NoL / (NoL * lambdaV + NoV * lambdaL);
}

vec3 F_Schlick(float cosTheta, vec3 f0)
{
    return f0 + (1.0 - f0) * pow(1.0 - cosTheta, 5.0);
}

vec3 F_SchlickRoughness(float cosTheta, vec3 f0, float roughness)
{
    return f0 + (max(vec3(1.0 - roughness), f0) - f0) * pow(1.0 - cosTheta, 5.0);
}

vec2 EnvironmentBRDF(float NoV, float roughness)
{
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
    return vec2(-1.04, 1.04) * a004 + r.zw;
}

vec3 Shade(vec3 albedo, vec3 N, vec3 V, vec3 L, float shadow, float ao)
{
    vec3 H = normalize(L + V);
    float NoL = max(dot(N, L), 0.0);
    float NoV = max(dot(N, V), 0.0001);
    float NoH = max(dot(N, H), 0.0);
    float HoV = max(dot(H, V), 0.0);

    float metallic = clamp(uMetallic, 0.0, 1.0);
    float roughness = clamp(uRoughness, 0.0, 1.0);

    // Keep a finite rasterized lobe for a perfectly smooth surface.
    float alpha = max(roughness * roughness, 0.0025);
    float alpha2 = alpha * alpha;

    vec3 f0 = mix(vec3(0.04), albedo, metallic);
    vec3 F = F_Schlick(HoV, f0);
    float D = D_TrowbridgeReitz(NoH, alpha2);
    float G = G_SmithCorrelated(NoV, NoL, alpha2);

    vec3 specular = D * G * F / max(4.0 * NoV * NoL, 0.0001);
    vec3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / pi;
    vec3 direct = (diffuse + specular) * uLightColor * uLightIntensity * NoL * shadow;

    vec3 environmentF = F_SchlickRoughness(NoV, f0, roughness);
    vec3 environmentDiffuse = (1.0 - environmentF) * (1.0 - metallic) * albedo;
    vec2 environmentBRDF = EnvironmentBRDF(NoV, roughness);
    vec3 environmentSpecular = f0 * environmentBRDF.x + environmentBRDF.y;

    // AO attenuates environment lighting.
    // Direct light is handled separately by the shadow map, while rough surfaces receive stronger specular AO.
    float specularOcclusion = mix(1.0, ao, roughness);
    vec3 ambient = uSkyColor * (environmentDiffuse * ao + environmentSpecular * specularOcclusion);

    return ambient + direct;
}
)";

constexpr const char* g_shapeFragmentShaderBody = R"(

float ComputeShadow(vec3 N, vec3 L)
{
    vec3 projCoords = vShadowPosition.xyz / vShadowPosition.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (any(lessThan(projCoords, vec3(0.0))) || any(greaterThan(projCoords, vec3(1.0))))
    {
        return 1.0;
    }

    float NoL = clamp(dot(N, L), 0.001, 1.0);
    float tanAngle = sqrt(1.0 - NoL * NoL) / NoL;
    float bias = clamp(0.0005 * tanAngle, 0.0002, 0.005);
    vec2 texelSize = 1.0 / vec2(textureSize(uShadowMap, 0));

    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            visibility += texture(uShadowMap, vec3(projCoords.xy + vec2(x, y) * texelSize, projCoords.z - bias));
        }
    }

    visibility /= 9.0;

    // Fade the PCF result at the shadow-map boundary to avoid a visible coverage edge.
    vec2 edge = min(projCoords.xy, 1.0 - projCoords.xy);
    float edgeFade = clamp(min(edge.x, edge.y) * 40.0, 0.0, 1.0);
    return mix(1.0, visibility, edgeFade);
}

void main()
{
    vec2 tiledUv = vTexCoord * vec2(8.0, 6.0);
    float checker = CheckerMask(tiledUv);

    vec3 colorA = vec3(0.96, 0.96, 0.96);
    vec3 colorB = vec3(0.76, 0.76, 0.76);

    vec3 albedo = SrgbToLinear(mix(colorA, colorB, checker)) * SrgbToLinear(vBaseColor);

    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-uLightDirection);
    float shadow = dot(N, L) > 0.0 ? ComputeShadow(N, L) : 1.0;
    vec3 V = normalize(uCameraPosition - vWorldPosition);

    FragColor = vec4(Shade(albedo, N, V, L, shadow, 1.0), 1.0);
}
)";

constexpr const char* g_shadowVertexShader = R"(
#version 330 core

layout (location = 0) in vec3 aPosition;
layout (location = 3) in vec4 iModel0;
layout (location = 4) in vec4 iModel1;
layout (location = 5) in vec4 iModel2;
layout (location = 6) in vec4 iModel3;

uniform mat4 uLightViewProjection;

void main()
{
    mat4 model = mat4(iModel0, iModel1, iModel2, iModel3);
    vec4 worldPosition = model * vec4(aPosition, 1.0);

    gl_Position = uLightViewProjection * worldPosition;
}
)";

constexpr const char* g_shadowFragmentShader = R"(
#version 330 core

void main()
{
}
)";

constexpr const char* g_geometryVertexShader = R"(
#version 450 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec4 iModel0;
layout (location = 4) in vec4 iModel1;
layout (location = 5) in vec4 iModel2;
layout (location = 6) in vec4 iModel3;
layout (location = 7) in vec4 iColor;
layout (location = 8) in vec3 iNormal0;
layout (location = 9) in vec3 iNormal1;
layout (location = 10) in vec3 iNormal2;

uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vViewNormal;
out vec2 vTexCoord;
out vec3 vBaseColor;

void main()
{
    mat4 model = mat4(iModel0, iModel1, iModel2, iModel3);
    vec4 worldPosition = model * vec4(aPosition, 1.0);

    vViewNormal = mat3(uView) * normalize(mat3(iNormal0, iNormal1, iNormal2) * aNormal);
    vTexCoord = aTexCoord;
    vBaseColor = iColor.rgb;

    gl_Position = uProjection * uView * worldPosition;
}
)";

constexpr const char* g_geometryFragmentShader = R"(
#version 450 core

in vec3 vViewNormal;
in vec2 vTexCoord;
in vec3 vBaseColor;

layout (location = 0) out vec4 Normal;
layout (location = 1) out vec4 Albedo;

vec3 SrgbToLinear(vec3 color)
{
    bvec3 cutoff = lessThanEqual(color, vec3(0.04045));

    vec3 lower = color / 12.92;
    vec3 higher = pow((color + 0.055) / 1.055, vec3(2.4));

    return mix(higher, lower, cutoff);
}

float CheckerMask(vec2 uv)
{
    vec2 cell = fract(uv);
    float checker = mod(floor(uv.x) + floor(uv.y), 2.0);

    float edgeDistance = min(min(cell.x, 1.0 - cell.x), min(cell.y, 1.0 - cell.y));
    float filterWidth = max(max(fwidth(uv.x), fwidth(uv.y)) * 0.35, 0.0001);

    return mix(0.5, checker, smoothstep(0.0, filterWidth, edgeDistance));
}

void main()
{
    float checker = CheckerMask(vTexCoord * vec2(8.0, 6.0));

    vec3 colorA = vec3(0.96, 0.96, 0.96);
    vec3 colorB = vec3(0.76, 0.76, 0.76);

    vec3 albedo = SrgbToLinear(mix(colorA, colorB, checker)) * SrgbToLinear(vBaseColor);

    Normal = vec4(normalize(vViewNormal) * 0.5 + 0.5, 1.0);
    Albedo = vec4(albedo, 1.0);
}
)";

constexpr const char* g_fullscreenVertexShader = R"(
#version 450 core

out vec2 vTexCoord;

void main()
{
    vec2 position = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vTexCoord = position;

    gl_Position = vec4(position * 2.0 - 1.0, 0.0, 1.0);
}
)";

constexpr const char* g_aoResolveFragmentShader = R"(
#version 450 core

in vec2 vTexCoord;

uniform sampler2DMS uDepth;
uniform sampler2DMS uNormal;

out vec4 Normal;

void main()
{
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    int sampleCount = textureSamples(uDepth);

    // Keep single MSAA sample instead of averaging across geometry edges.
    float avgDepth = 0.0;
    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        avgDepth += texelFetch(uDepth, pixel, sampleIndex).r;
    }
    avgDepth /= float(sampleCount);

    int bestSample = 0;
    float bestDepth = texelFetch(uDepth, pixel, 0).r;
    float bestDistance = abs(bestDepth - avgDepth);

    for (int sampleIndex = 1; sampleIndex < sampleCount; ++sampleIndex)
    {
        float depth = texelFetch(uDepth, pixel, sampleIndex).r;
        float distance = abs(depth - avgDepth);

        if (distance < bestDistance || (distance == bestDistance && depth < bestDepth))
        {
            bestSample = sampleIndex;
            bestDepth = depth;
            bestDistance = distance;
        }
    }

    Normal = texelFetch(uNormal, pixel, bestSample);
    gl_FragDepth = bestDepth;
}
)";

constexpr const char* g_aoFragmentShader = R"(
#version 450 core

in vec2 vTexCoord;

uniform sampler2D uDepth;
uniform sampler2D uNormal;
uniform sampler2D uSpatialNoise;
uniform mat4 uProjection;

// It's actually a visibility.
out float Occlusion;

const float pi = 3.14159265358979323846;
const float halfPi = 1.57079632679489661923;

vec2 SpatialNoise()
{
    ivec2 pixel = ivec2(gl_FragCoord.xy) & 63;
    return texelFetch(uSpatialNoise, pixel, 0).rg;
}

float ViewDepth(float depth)
{
    // The hardware depth is non-linear. Recover the view-space z value first.
    float ndcDepth = depth * 2.0 - 1.0;
    return -uProjection[3][2] / (ndcDepth + uProjection[2][2]);
}

vec3 ReconstructViewPosition(vec2 uv, float depth)
{
    // Rebuild the view-space position needed to measure real sample distances.
    float z = ViewDepth(depth);
    vec2 ndc = uv * 2.0 - 1.0 + vec2(uProjection[2][0], uProjection[2][1]);
    vec2 xy = -z * ndc / vec2(uProjection[0][0], uProjection[1][1]);
    return vec3(xy, z);
}

void main()
{
    float depth = texture(uDepth, vTexCoord).r;
    if (depth >= 1.0)
    {
        Occlusion = 1.0;
        return;
    }

    vec3 position = ReconstructViewPosition(vTexCoord, depth);
    vec3 N = normalize(texture(uNormal, vTexCoord).xyz * 2.0 - 1.0);
    vec3 viewDirection = normalize(-position);
    vec2 noise = SpatialNoise();
    vec2 depthSize = vec2(textureSize(uDepth, 0));
    vec2 texelSize = 1.0 / depthSize;

    // The trace radius is in view/world units and is converted to pixels below.
    const float effectRadius = 0.75;
    const float falloffRange = effectRadius * 0.5;
    const float falloffFrom = effectRadius - falloffRange;
    const float falloffMul = -1.0 / falloffRange;
    const float falloffAdd = falloffFrom / falloffRange + 1.0;

    const int sliceCount = 8;
    const int stepsPerSlice = 4;

    // Keep the radius view-independent: the same world-space radius covers
    // fewer pixels on distant surfaces and more pixels on nearby surfaces.
    float pixelViewSize = 2.0 * -position.z / (uProjection[0][0] * depthSize.x);
    float screenRadius = clamp(effectRadius / max(pixelViewSize, 0.0001), 1.0, 128.0);
    float minStep = 1.3 / screenRadius;
    float visibility = 0.0;

    for (int slice = 0; slice < sliceCount; ++slice)
    {
        // A slice covers one screen direction and its opposite. Using [0, pi)
        // here therefore covers the full circle without duplicating planes.
        float sliceK = (float(slice) + noise.x) / float(sliceCount);
        float phi = sliceK * pi;

        vec2 screenDirection = vec2(cos(phi), sin(phi));
        vec3 direction = vec3(screenDirection, 0.0);

        // The slice plane contains viewDirection and the screen direction projected perpendicular to it.
        // The surface normal is not generally in this plane, so it is projected into the plane below.
        vec3 orthoDirection = direction - dot(direction, viewDirection) * viewDirection;
        vec3 axis = normalize(cross(orthoDirection, viewDirection));
        vec3 projectedNormal = N - axis * dot(N, axis);

        float projectedNormalLength = max(length(projectedNormal), 0.0001);
        float signNormal = sign(dot(orthoDirection, projectedNormal));
        float cosNormal = clamp(dot(projectedNormal, viewDirection) / projectedNormalLength, 0.0, 1.0);
        float normalAngle = signNormal * acos(cosNormal);

        // Horizons are kept as cosines while sampling.
        // This avoids an acos per sample; only the two final bounds need to be converted to angles.
        float lowHorizon0 = cos(normalAngle + halfPi);
        float lowHorizon1 = cos(normalAngle - halfPi);
        float horizon0 = lowHorizon0;
        float horizon1 = lowHorizon1;

        for (int stepIndex = 0; stepIndex < stepsPerSlice; ++stepIndex)
        {
            float sequence = (float(slice) + float(stepIndex * stepsPerSlice)) * 0.61803398875;
            float stepNoise = fract(noise.y + sequence);

            // Squaring concentrates samples near the center.
            // minStep keeps the first sample away from the current pixel to reduce self-occlusion.
            float stepScale = (float(stepIndex) + stepNoise) / float(stepsPerSlice);
            stepScale = stepScale * stepScale + minStep;

            vec2 sampleOffset = round(screenDirection * stepScale * screenRadius) * texelSize;
            vec2 sampleUv0 = vTexCoord + sampleOffset;
            vec2 sampleUv1 = vTexCoord - sampleOffset;

            if (all(greaterThanEqual(sampleUv0, vec2(0.0))) && all(lessThanEqual(sampleUv0, vec2(1.0))))
            {
                vec3 samplePosition = ReconstructViewPosition(sampleUv0, texture(uDepth, sampleUv0).r);
                vec3 sampleDelta = samplePosition - position;
                float sampleDistance = length(sampleDelta);

                // Fade the horizon contribution to the unoccluded baseline at the outer part of the effect radius.
                float weight = clamp(sampleDistance * falloffMul + falloffAdd, 0.0, 1.0);
                float horizon = dot(sampleDelta / max(sampleDistance, 0.0001), viewDirection);

                // Keep the highest horizon on this side of the slice.
                horizon0 = max(horizon0, mix(lowHorizon0, horizon, weight));
            }

            if (all(greaterThanEqual(sampleUv1, vec2(0.0))) && all(lessThanEqual(sampleUv1, vec2(1.0))))
            {
                vec3 samplePosition = ReconstructViewPosition(sampleUv1, texture(uDepth, sampleUv1).r);
                vec3 sampleDelta = samplePosition - position;

                float sampleDistance = length(sampleDelta);
                float weight = clamp(sampleDistance * falloffMul + falloffAdd, 0.0, 1.0);
                float horizon = dot(sampleDelta / max(sampleDistance, 0.0001), viewDirection);
        
                horizon1 = max(horizon1, mix(lowHorizon1, horizon, weight));
            }
        }

        // Correct the projected normal length slightly to avoid unstable dark
        // slices when the normal is nearly perpendicular to the slice plane.
        projectedNormalLength = mix(projectedNormalLength, 1.0, 0.05);

        float horizonAngle0 = -acos(clamp(horizon1, -1.0, 1.0));
        float horizonAngle1 = acos(clamp(horizon0, -1.0, 1.0));

        // Analytically integrate the cosine-weighted visible arc between the two horizon angles.
        // Samples only determine these arc boundaries.
        float arc0 = (cosNormal + 2.0 * horizonAngle0 * sin(normalAngle) - cos(2.0 * horizonAngle0 - normalAngle)) * 0.25;
        float arc1 = (cosNormal + 2.0 * horizonAngle1 * sin(normalAngle) - cos(2.0 * horizonAngle1 - normalAngle)) * 0.25;

        visibility += projectedNormalLength * max(arc0 + arc1, 0.0);
    }

    // This texture stores visibility: 1 means unoccluded. The power and floor
    // are artistic stabilization to preserve readable contact shading.
    visibility = pow(clamp(visibility / float(sliceCount), 0.0, 1.0), 1.45);
    Occlusion = max(visibility, 0.08);
}
)";

constexpr const char* g_aoBlurFragmentShader = R"(
#version 450 core
in vec2 vTexCoord;

uniform sampler2D uAO;
uniform sampler2D uDepth;
uniform sampler2D uNormal;
uniform mat4 uProjection;
uniform vec3 uInvAoSize;

out float Occlusion;

float ViewDepth(vec2 uv)
{
    float depth = texture(uDepth, uv).r;
    float ndcDepth = depth * 2.0 - 1.0;

    return -uProjection[3][2] / (ndcDepth + uProjection[2][2]);
}

void main()
{
    float centerRawDepth = texture(uDepth, vTexCoord).r;
    if (centerRawDepth >= 1.0)
    {
        Occlusion = 1.0;
        return;
    }

    float centerDepth = ViewDepth(vTexCoord);
    vec3 centerNormal = normalize(texture(uNormal, vTexCoord).xyz * 2.0 - 1.0);
    float sum = 0.0;
    float weightSum = 0.0;

    // A small cross-bilateral filter removes deterministic trace noise without
    // blurring across depth or normal discontinuities.
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            vec2 uv = vTexCoord + vec2(x, y) * uInvAoSize.xy;

            float sampleDepth = ViewDepth(uv);
            vec3 sampleNormal = normalize(texture(uNormal, uv).xyz * 2.0 - 1.0);

            float spatialWeight = exp(-0.5 * dot(vec2(x, y), vec2(x, y)));
            float depthWeight = exp(-abs(centerDepth - sampleDepth) * 8.0);
            float normalWeight = pow(max(dot(centerNormal, sampleNormal), 0.0), 8);

            float weight = spatialWeight * depthWeight * normalWeight;

            sum += texture(uAO, uv).r * weight;
            weightSum += weight;
        }
    }

    Occlusion = sum / max(weightSum, 0.0001);
}
)";

constexpr const char* g_deferredFragmentShaderHeader = R"(
#version 450 core
in vec2 vTexCoord;

uniform sampler2DMS uDepth;
uniform sampler2DMS uNormal;
uniform sampler2DMS uAlbedo;
uniform sampler2D uAO;
uniform sampler2DShadow uShadowMap;
uniform mat4 uView;
uniform mat4 uInvView;
uniform mat4 uProjection;
uniform mat4 uLightViewProjection;
uniform vec3 uLightDirection;
uniform vec3 uClearColor;
uniform int uShadeGeometry;

out vec4 FragColor;

float ViewDepth(float depth)
{
    float ndcDepth = depth * 2.0 - 1.0;
    return -uProjection[3][2] / (ndcDepth + uProjection[2][2]);
}

vec3 ReconstructViewPosition(vec2 uv, float depth)
{
    float z = ViewDepth(depth);
    vec2 ndc = uv * 2.0 - 1.0 + vec2(uProjection[2][0], uProjection[2][1]);
    vec2 xy = -z * ndc / vec2(uProjection[0][0], uProjection[1][1]);
    return vec3(xy, z);
}
)";

constexpr const char* g_deferredFragmentShaderBody = R"(

float ComputeShadow(vec3 worldPosition, vec3 N, vec3 L)
{
    vec4 shadowPosition = uLightViewProjection * vec4(worldPosition, 1.0);
    vec3 projCoords = shadowPosition.xyz / shadowPosition.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (any(lessThan(projCoords, vec3(0.0))) || any(greaterThan(projCoords, vec3(1.0))))
    {
        return 1.0;
    }

    float NoL = clamp(dot(N, L), 0.001, 1.0);
    float tanAngle = sqrt(1.0 - NoL * NoL) / NoL;
    float bias = clamp(0.0005 * tanAngle, 0.0002, 0.005);
    vec2 texelSize = 1.0 / vec2(textureSize(uShadowMap, 0));
    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            visibility += texture(uShadowMap, vec3(projCoords.xy + vec2(x, y) * texelSize, projCoords.z - bias));
        }
    }
    visibility /= 9.0;

    const float fadeSpeed = 40.0;

    vec2 edge = min(projCoords.xy, 1.0 - projCoords.xy);
    float edgeFade = clamp(min(edge.x, edge.y) * fadeSpeed, 0.0, 1.0);

    return mix(1.0, visibility, edgeFade);
}

vec3 ShadeSample(vec2 uv, float depth, vec3 encodedNormal, vec3 albedo, float ao)
{
    if (depth >= 1.0)
    {
        return uClearColor;
    }

    vec3 viewPosition = ReconstructViewPosition(uv, depth);
    vec3 worldPosition = (uInvView * vec4(viewPosition, 1.0)).xyz;
    vec3 N = normalize(mat3(uInvView) * (encodedNormal * 2.0 - 1.0));
    vec3 V = normalize(-viewPosition);
    V = normalize(mat3(uInvView) * V);
    vec3 L = normalize(-uLightDirection);

    float shadow = dot(N, L) > 0.0 ? ComputeShadow(worldPosition, N, L) : 1.0;

    return Shade(albedo, N, V, L, shadow, ao);
}

void main()
{
    if (uShadeGeometry == 0)
    {
        FragColor = vec4(uClearColor, 1.0);
        return;
    }

    ivec2 pixel = ivec2(gl_FragCoord.xy);
    vec2 uv = (vec2(pixel) + gl_SamplePosition) / vec2(textureSize(uDepth));
    float ao = texture(uAO, uv).r;
    float depth = texelFetch(uDepth, pixel, gl_SampleID).r;
    vec3 normal = texelFetch(uNormal, pixel, gl_SampleID).xyz;
    vec3 albedo = texelFetch(uAlbedo, pixel, gl_SampleID).rgb;
    FragColor = vec4(ShadeSample(uv, depth, normal, albedo, ao), 1.0);
}
)";

constexpr const char* g_presentFragmentShader = R"(
#version 450 core

in vec2 vTexCoord;

uniform sampler2D uSceneColor;
uniform sampler2DMS uDebugColor;

out vec4 FragColor;

vec3 LinearToSrgb(vec3 color)
{
    bvec3 cutoff = lessThanEqual(color, vec3(0.0031308));

    vec3 lower = color * 12.92;
    vec3 higher = 1.055 * pow(max(color, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;

    return mix(higher, lower, cutoff);
}

vec3 ToneMap_PBRNeutral(vec3 color)
{
    const float startCompression = 0.8 - 0.04;
    const float desaturation = 0.15;

    float x = min(color.r, min(color.g, color.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    color -= offset;

    float peak = max(color.r, max(color.g, color.b));
    if (peak < startCompression)
    {
        return color;
    }

    const float d = 1.0 - startCompression;
    float newPeak = 1.0 - d * d / (peak + d - startCompression);
    color *= newPeak / peak;

    float g = 1.0 - 1.0 / (desaturation * (peak - newPeak) + 1.0);
    return mix(color, vec3(newPeak), g);
}

void main()
{
    vec3 linearColor = texture(uSceneColor, vTexCoord).rgb;
    vec3 toneMappedColor = ToneMap_PBRNeutral(linearColor);
    vec3 sceneColor = LinearToSrgb(toneMappedColor);

    ivec2 debugSize = textureSize(uDebugColor);
    ivec2 pixel = min(ivec2(vTexCoord * vec2(debugSize)), debugSize - 1);

    int sampleCount = textureSamples(uDebugColor);

    vec4 debugColor = vec4(0.0);
    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        debugColor += texelFetch(uDebugColor, pixel, sampleIndex);
    }
    debugColor /= float(sampleCount);

    FragColor = vec4(debugColor.rgb + sceneColor * (1.0 - debugColor.a), 1.0);
}
)";

constexpr const char* g_primitiveVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPoint;
layout (location = 1) in vec4 aColor;

uniform mat4 uView;
uniform mat4 uProjection;
uniform float uPointSize;

out vec4 vColor;

void main()
{
    gl_Position = uProjection * uView * vec4(aPoint, 1.0);
    gl_PointSize = uPointSize;
    vColor = aColor;
}
)";

constexpr const char* g_primitiveFragmentShader = R"(
#version 330 core
in vec4 vColor;

out vec4 FragColor;

void main()
{
    FragColor = vColor;
}
)";

void ComputeCameraFrustumCorners(const Camera& camera, float aspectRatio, Vec3* corners)
{
    float zNear = 0.1f;
    float zFar = g_shadow_view_distance;
    float tanHalfFov = std::tan(DegToRad(camera.fovDegrees) * 0.5f);

    float nearY = zNear * tanHalfFov;
    float nearX = nearY * aspectRatio;
    float farY = zFar * tanHalfFov;
    float farX = farY * aspectRatio;

    Vec3 position = camera.GetPosition();
    Vec3 forward = camera.GetForward();
    Vec3 right = camera.GetRight();
    Vec3 up = camera.GetUp();

    Vec3 nearCenter = position + forward * zNear;
    Vec3 farCenter = position + forward * zFar;

    corners[0] = nearCenter - right * nearX - up * nearY;
    corners[1] = nearCenter + right * nearX - up * nearY;
    corners[2] = nearCenter - right * nearX + up * nearY;
    corners[3] = nearCenter + right * nearX + up * nearY;
    corners[4] = farCenter - right * farX - up * farY;
    corners[5] = farCenter + right * farX - up * farY;
    corners[6] = farCenter - right * farX + up * farY;
    corners[7] = farCenter + right * farX + up * farY;
}

Mat4 ComputeLightViewProjection(const Camera& camera, float aspectRatio, const Vec3& lightDirection)
{
    Vec3 frustumCorners[8];
    ComputeCameraFrustumCorners(camera, aspectRatio, frustumCorners);

    Vec3 frustumCenter = Vec3::zero;
    for (int32 i = 0; i < 8; ++i)
    {
        frustumCenter += frustumCorners[i];
    }
    frustumCenter /= 8.0f;

    float frustumRadius = 0.0f;
    for (int32 i = 0; i < 8; ++i)
    {
        frustumRadius = Max(frustumRadius, Length(frustumCorners[i] - frustumCenter));
    }

    Vec3 lightPosition = frustumCenter - lightDirection * (frustumRadius + g_shadow_depth_padding);
    Vec3 lightUp = AbsDot(lightDirection, y_axis) < 0.99f ? y_axis : z_axis;
    Mat4 lightView = Mat4::LookAt(lightPosition, frustumCenter, lightUp);

    Vec3 lightMin{ max_float };
    Vec3 lightMax{ -max_float };
    for (int32 i = 0; i < 8; ++i)
    {
        Vec4 p = lightView * Vec4{ frustumCorners[i], 1.0f };
        lightMin = Min(lightMin, Vec3{ p.x, p.y, p.z });
        lightMax = Max(lightMax, Vec3{ p.x, p.y, p.z });
    }

    float halfWidth = Max((lightMax.x - lightMin.x) * 0.5f + g_shadow_bounds_padding, 8.0f);
    float halfHeight = Max((lightMax.y - lightMin.y) * 0.5f + g_shadow_bounds_padding, 8.0f);

    Vec2 center{ (lightMin.x + lightMax.x) * 0.5f, (lightMin.y + lightMax.y) * 0.5f };

    float zNear = Max(-lightMax.z - g_shadow_depth_padding, 0.1f);
    float zFar = Max(-lightMin.z + g_shadow_bounds_padding, zNear + 1.0f);
    Mat4 lightProjection =
        Mat4::Orth(center.x - halfWidth, center.x + halfWidth, center.y - halfHeight, center.y + halfHeight, zNear, zFar);

    // Snap the final shadow matrix to texel increments so camera movement does not shimmer the map.
    Mat4 lightViewProjection = lightProjection * lightView;
    Vec4 shadowOrigin = lightViewProjection * Vec4{ Vec3::zero, 1.0f };
    shadowOrigin *= g_shadow_map_size * 0.5f;

    Vec4 roundedOrigin{
        std::floor(shadowOrigin.x + 0.5f),
        std::floor(shadowOrigin.y + 0.5f),
        shadowOrigin.z,
        shadowOrigin.w,
    };
    Vec4 roundOffset = (roundedOrigin - shadowOrigin) * (2.0f / g_shadow_map_size);
    lightProjection.ew.x += roundOffset.x;
    lightProjection.ew.y += roundOffset.y;

    return lightProjection * lightView;
}

void SetInstanceAttribute(GLuint index, GLint size, GLsizei stride, size_t offset)
{
    glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offset));
    glEnableVertexAttribArray(index);
    glVertexAttribDivisor(index, 1);
}

void InitializeColors()
{
    if (g_colorsInitialized)
    {
        return;
    }

    constexpr float stride = 360.0f / g_color_count;
    for (int32 i = 0; i < g_color_count; ++i)
    {
        Vec3 rgb = color::HSLToRGB({ i * stride / 360.0f, 1.0f, 0.8f });
        g_colors[i] = Vec4{ rgb.x, rgb.y, rgb.z, 0.85f };
    }

    for (int32 i = 0; i < constraint_color_count; ++i)
    {
        float hue = ((i * 7) % constraint_color_count) / float(constraint_color_count);
        Vec3 rgb = color::HSLToRGB({ hue, 0.9f, 0.62f });
        g_colors2[i] = { rgb.x, rgb.y, rgb.z, 0.95f };
    }

    g_colorsInitialized = true;
}

Vec4 GetModeColor(const Renderer::DrawMode& mode)
{
    if (mode.colorIndex < 0)
    {
        return Renderer::default_white;
    }

    return g_colors[mode.colorIndex % g_color_count];
}

Renderer::~Renderer()
{
    Shutdown();
}

bool Renderer::CreateShadowResources()
{
    glGenFramebuffers(1, &shadowFramebuffer);
    glGenTextures(1, &shadowDepthTexture);

    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, g_shadow_map_size, g_shadow_map_size, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    const float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowDepthTexture, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        DestroyShadowResources();
        return false;
    }
    return true;
}

bool Renderer::CreateFrameResources(int32 width, int32 height)
{
    if (geometryFramebuffer != 0 && width == frameWidth && height == frameHeight)
    {
        return true;
    }

    DestroyFrameResources();
    frameWidth = width;
    frameHeight = height;

    glGenFramebuffers(1, &geometryFramebuffer);
    glGenTextures(1, &geometryNormalTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, geometryNormalTexture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, g_sample_count, GL_RGB10_A2, width, height, GL_TRUE);

    glGenTextures(1, &geometryAlbedoTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, geometryAlbedoTexture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, g_sample_count, GL_RGBA8, width, height, GL_TRUE);

    glGenTextures(1, &geometryDepthTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, geometryDepthTexture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, g_sample_count, GL_DEPTH_COMPONENT32F, width, height, GL_TRUE);

    glBindFramebuffer(GL_FRAMEBUFFER, geometryFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, geometryNormalTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D_MULTISAMPLE, geometryAlbedoTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, geometryDepthTexture, 0);
    GLenum geometryDrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
    glDrawBuffers(2, geometryDrawBuffers);
    bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenFramebuffers(1, &aoInputFramebuffer);
    glGenTextures(1, &aoNormalTexture);
    glBindTexture(GL_TEXTURE_2D, aoNormalTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB10_A2, width, height, 0, GL_RGBA, GL_UNSIGNED_INT_2_10_10_10_REV, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &aoDepthTexture);
    glBindTexture(GL_TEXTURE_2D, aoDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindFramebuffer(GL_FRAMEBUFFER, aoInputFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, aoNormalTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, aoDepthTexture, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenFramebuffers(1, &aoFramebuffer);
    glGenTextures(1, &aoTexture);
    glBindTexture(GL_TEXTURE_2D, aoTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, aoFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, aoTexture, 0);
    complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenFramebuffers(1, &aoBlurFramebuffer);
    glGenTextures(1, &aoBlurTexture);
    glBindTexture(GL_TEXTURE_2D, aoBlurTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, aoBlurFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, aoBlurTexture, 0);
    complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenTextures(1, &sceneColorTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, sceneColorTexture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, g_sample_count, GL_RGBA16F, width, height, GL_TRUE);

    glGenTextures(1, &debugColorTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, debugColorTexture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, g_sample_count, GL_RGBA8, width, height, GL_TRUE);

    glGenFramebuffers(1, &sceneFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, sceneColorTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D_MULTISAMPLE, debugColorTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, geometryDepthTexture, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenTextures(1, &presentColorTexture);
    glBindTexture(GL_TEXTURE_2D, presentColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenFramebuffers(1, &presentFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, presentFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, presentColorTexture, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    std::array<uint8, 64 * 64 * 2> spatialNoise;
    // A tiled low-discrepancy pattern rotates slices and jitters radial steps.
    for (uint32 y = 0; y < 64; ++y)
    {
        for (uint32 x = 0; x < 64; ++x)
        {
            float index = (float)ComputeHilbertIndex(x, y);
            float noiseX = 0.5f + index * 0.75487766625f;
            float noiseY = 0.5f + index * 0.56984029100f;
            noiseX -= std::floor(noiseX);
            noiseY -= std::floor(noiseY);
            size_t offset = (y * 64 + x) * 2;
            spatialNoise[offset] = (uint8)(noiseX * 255.0f + 0.5f);
            spatialNoise[offset + 1] = (uint8)(noiseY * 255.0f + 0.5f);
        }
    }

    glGenTextures(1, &aoNoiseTexture);
    glBindTexture(GL_TEXTURE_2D, aoNoiseTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, 64, 64, 0, GL_RG, GL_UNSIGNED_BYTE, spatialNoise.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    if (fullscreenVAO == 0)
    {
        glGenVertexArrays(1, &fullscreenVAO);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (!complete)
    {
        DestroyFrameResources();
        return false;
    }
    return true;
}

bool Renderer::CreatePrimitiveResources()
{
    if (!primitiveShader.Create(g_primitiveVertexShader, g_primitiveFragmentShader))
    {
        return false;
    }

    glGenVertexArrays(1, &primVAO);
    glGenBuffers(1, &primVBO);

    glBindVertexArray(primVAO);
    glBindBuffer(GL_ARRAY_BUFFER, primVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * g_max_vertex_count, nullptr, GL_STREAM_DRAW);
    primitiveCapacity = g_max_vertex_count;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, point)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return primVAO != 0 && primVBO != 0;
}

void Renderer::SetShapeInstanceAttributes()
{
    SetInstanceAttribute(3, 4, sizeof(ShapeInstance), offsetof(ShapeInstance, model.ex));
    SetInstanceAttribute(4, 4, sizeof(ShapeInstance), offsetof(ShapeInstance, model.ey));
    SetInstanceAttribute(5, 4, sizeof(ShapeInstance), offsetof(ShapeInstance, model.ez));
    SetInstanceAttribute(6, 4, sizeof(ShapeInstance), offsetof(ShapeInstance, model.ew));
    SetInstanceAttribute(7, 4, sizeof(ShapeInstance), offsetof(ShapeInstance, color));
    SetInstanceAttribute(8, 3, sizeof(ShapeInstance), offsetof(ShapeInstance, normal.ex));
    SetInstanceAttribute(9, 3, sizeof(ShapeInstance), offsetof(ShapeInstance, normal.ey));
    SetInstanceAttribute(10, 3, sizeof(ShapeInstance), offsetof(ShapeInstance, normal.ez));
}

GLuint Renderer::UploadShapeInstances(const ShapeInstance* instances, size_t count)
{
    MuliAssert(count <= g_max_shape_batch_count);

    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    if (shapeInstanceOffset + count > shapeInstanceCapacity)
    {
        shapeInstanceCapacity = Max(shapeInstanceCapacity * 2, count);
        shapeInstanceOffset = 0;
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(shapeInstanceCapacity * sizeof(ShapeInstance)), nullptr, GL_STREAM_DRAW);
    }

    GLuint baseInstance = (GLuint)shapeInstanceOffset;
    glBufferSubData(
        GL_ARRAY_BUFFER, (GLintptr)(shapeInstanceOffset * sizeof(ShapeInstance)), (GLsizeiptr)(count * sizeof(ShapeInstance)),
        instances
    );
    shapeInstanceOffset += count;
    return baseInstance;
}

bool Renderer::CreateShapeResources()
{
    glGenBuffers(1, &shapeInstanceVBO);
    glGenVertexArrays(1, &shapeMeshVAO);
    glGenVertexArrays(1, &shapeMeshOutlineVAO);
    glGenBuffers(1, &shapeMeshVBO);
    glGenBuffers(1, &shapeMeshEBO);
    glGenBuffers(1, &shapeMeshOutlineEBO);
    glGenBuffers(1, &shapeMeshIndirectVBO);

    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(shapeInstanceCapacity * sizeof(ShapeInstance)), nullptr, GL_STREAM_DRAW);

    GLuint meshVaos[] = {
        sphereBatch.mesh.GetVAO(),        sphereBatch.mesh.GetOutlineVAO(),
        capsuleTopBatch.mesh.GetVAO(),    capsuleTopBatch.mesh.GetOutlineVAO(),
        capsuleBottomBatch.mesh.GetVAO(), capsuleBottomBatch.mesh.GetOutlineVAO(),
        capsuleMidBatch.mesh.GetVAO(),    capsuleMidBatch.mesh.GetOutlineVAO(),
        boxBatch.mesh.GetVAO(),           boxBatch.mesh.GetOutlineVAO(),
        triangleBatch.mesh.GetVAO(),      triangleBatch.mesh.GetOutlineVAO(),
    };
    for (GLuint vao : meshVaos)
    {
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
        SetShapeInstanceAttributes();
    }

    glBindVertexArray(shapeMeshVAO);
    glBindBuffer(GL_ARRAY_BUFFER, shapeMeshVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shapeMeshEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(shapeMeshOutlineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, shapeMeshVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shapeMeshOutlineEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return shapeInstanceVBO != 0 && shapeMeshVAO != 0 && shapeMeshOutlineVAO != 0 && shapeMeshVBO != 0 && shapeMeshEBO != 0 &&
           shapeMeshOutlineEBO != 0 && shapeMeshIndirectVBO != 0;
}

void Renderer::DestroyShadowResources()
{
    if (shadowDepthTexture != 0)
    {
        glDeleteTextures(1, &shadowDepthTexture);
        shadowDepthTexture = 0;
    }
    if (shadowFramebuffer != 0)
    {
        glDeleteFramebuffers(1, &shadowFramebuffer);
        shadowFramebuffer = 0;
    }
}

void Renderer::DestroyFrameResources()
{
    GLuint sceneTextures[] = { sceneColorTexture, presentColorTexture, debugColorTexture };
    glDeleteTextures(3, sceneTextures);
    sceneColorTexture = 0;
    presentColorTexture = 0;
    debugColorTexture = 0;

    GLuint sceneFramebuffers[] = { sceneFramebuffer, presentFramebuffer };
    glDeleteFramebuffers(2, sceneFramebuffers);
    sceneFramebuffer = 0;
    presentFramebuffer = 0;

    GLuint aoTextures[] = {
        geometryNormalTexture, geometryAlbedoTexture, geometryDepthTexture, aoNormalTexture, aoDepthTexture, aoTexture,
        aoBlurTexture,         aoNoiseTexture,
    };
    glDeleteTextures(8, aoTextures);
    geometryNormalTexture = 0;
    geometryAlbedoTexture = 0;
    geometryDepthTexture = 0;
    aoNormalTexture = 0;
    aoDepthTexture = 0;
    aoTexture = 0;
    aoBlurTexture = 0;
    aoNoiseTexture = 0;

    GLuint aoFramebuffers[] = {
        geometryFramebuffer,
        aoInputFramebuffer,
        aoFramebuffer,
        aoBlurFramebuffer,
    };
    glDeleteFramebuffers(4, aoFramebuffers);
    geometryFramebuffer = 0;
    aoInputFramebuffer = 0;
    aoFramebuffer = 0;
    aoBlurFramebuffer = 0;

    if (fullscreenVAO != 0)
    {
        glDeleteVertexArrays(1, &fullscreenVAO);
        fullscreenVAO = 0;
    }
    frameWidth = 0;
    frameHeight = 0;
}

void Renderer::DestroyPrimitiveResources()
{
    if (primVBO != 0)
    {
        glDeleteBuffers(1, &primVBO);
        primVBO = 0;
    }
    primitiveCapacity = 0;
    if (primVAO != 0)
    {
        glDeleteVertexArrays(1, &primVAO);
        primVAO = 0;
    }
}

void Renderer::DestroyShapeResources()
{
    if (shapeMeshIndirectVBO != 0)
    {
        glDeleteBuffers(1, &shapeMeshIndirectVBO);
        shapeMeshIndirectVBO = 0;
    }
    if (shapeMeshEBO != 0)
    {
        glDeleteBuffers(1, &shapeMeshEBO);
        shapeMeshEBO = 0;
    }
    if (shapeMeshOutlineEBO != 0)
    {
        glDeleteBuffers(1, &shapeMeshOutlineEBO);
        shapeMeshOutlineEBO = 0;
    }
    if (shapeMeshVBO != 0)
    {
        glDeleteBuffers(1, &shapeMeshVBO);
        shapeMeshVBO = 0;
    }
    if (shapeMeshVAO != 0)
    {
        glDeleteVertexArrays(1, &shapeMeshVAO);
        shapeMeshVAO = 0;
    }
    if (shapeMeshOutlineVAO != 0)
    {
        glDeleteVertexArrays(1, &shapeMeshOutlineVAO);
        shapeMeshOutlineVAO = 0;
    }
    if (shapeInstanceVBO != 0)
    {
        glDeleteBuffers(1, &shapeInstanceVBO);
        shapeInstanceVBO = 0;
    }
}

bool Renderer::Initialize()
{
    Shutdown();

    std::string shapeFragmentShader = g_shapeFragmentShaderHeader;
    shapeFragmentShader += g_pbrFragmentShader;
    shapeFragmentShader += g_shapeFragmentShaderBody;

    if (!shapeShader.Create(g_shapeVertexShader, shapeFragmentShader.c_str()) ||
        !shadowShader.Create(g_shadowVertexShader, g_shadowFragmentShader) ||
        !geometryShader.Create(g_geometryVertexShader, g_geometryFragmentShader) ||
        !aoResolveShader.Create(g_fullscreenVertexShader, g_aoResolveFragmentShader) ||
        !aoShader.Create(g_fullscreenVertexShader, g_aoFragmentShader) ||
        !aoBlurShader.Create(g_fullscreenVertexShader, g_aoBlurFragmentShader))
    {
        Shutdown();
        return false;
    }

    std::string deferredFragmentShader = g_deferredFragmentShaderHeader;
    deferredFragmentShader += g_pbrFragmentShader;
    deferredFragmentShader += g_deferredFragmentShaderBody;

    if (!deferredShader.Create(g_fullscreenVertexShader, deferredFragmentShader.c_str()) ||
        !presentShader.Create(g_fullscreenVertexShader, g_presentFragmentShader) || !CreateShadowResources() ||
        !CreatePrimitiveResources())
    {
        Shutdown();
        return false;
    }

    InitializeColors();

    shapeShader.Use();
    shapeShader.SetInt("uShadowMap", 0);
    shapeShader.SetFloat("uMetallic", g_metallic);
    shapeShader.SetFloat("uRoughness", g_roughness);

    aoResolveShader.Use();
    aoResolveShader.SetInt("uDepth", 0);
    aoResolveShader.SetInt("uNormal", 1);

    aoShader.Use();
    aoShader.SetInt("uDepth", 0);
    aoShader.SetInt("uNormal", 1);
    aoShader.SetInt("uSpatialNoise", 2);

    aoBlurShader.Use();
    aoBlurShader.SetInt("uAO", 0);
    aoBlurShader.SetInt("uDepth", 1);
    aoBlurShader.SetInt("uNormal", 2);

    deferredShader.Use();
    deferredShader.SetInt("uDepth", 0);
    deferredShader.SetInt("uNormal", 1);
    deferredShader.SetInt("uAlbedo", 2);
    deferredShader.SetInt("uAO", 3);
    deferredShader.SetInt("uShadowMap", 4);
    deferredShader.SetFloat("uMetallic", g_metallic);
    deferredShader.SetFloat("uRoughness", g_roughness);

    presentShader.Use();
    presentShader.SetInt("uSceneColor", 0);
    presentShader.SetInt("uDebugColor", 1);

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    std::vector<uint32> outlineIndices;

    BuildSphereMesh(&vertices, &indices, &outlineIndices, 24, 12);
    sphereBatch.mesh.Upload(vertices, indices, outlineIndices);

    BuildCapsuleTopMesh(&vertices, &indices, &outlineIndices, 24, 12);
    capsuleTopBatch.mesh.Upload(vertices, indices, outlineIndices);

    BuildCapsuleBottomMesh(&vertices, &indices, &outlineIndices, 24, 12);
    capsuleBottomBatch.mesh.Upload(vertices, indices, outlineIndices);

    BuildCapsuleMidMesh(&vertices, &indices, &outlineIndices, 24);
    capsuleMidBatch.mesh.Upload(vertices, indices, outlineIndices);

    BuildBoxMesh(&vertices, &indices, &outlineIndices);
    boxBatch.mesh.Upload(vertices, indices, outlineIndices);

    BuildTriangleMesh(&vertices, &indices, &outlineIndices);
    triangleBatch.mesh.Upload(vertices, indices, outlineIndices);

    if (!CreateShapeResources())
    {
        Shutdown();
        return false;
    }

    InstancedMeshBatch* batches[] = {
        &sphereBatch, &capsuleTopBatch, &capsuleBottomBatch, &capsuleMidBatch, &boxBatch, &triangleBatch,
    };
    for (InstancedMeshBatch* batch : batches)
    {
        batch->instances[g_fill_pass].reserve(g_max_shape_batch_count);
        batch->instances[g_outline_pass].reserve(g_max_shape_batch_count);
    }
    points.resize(g_max_vertex_count);
    lines.resize(g_max_vertex_count);
    return true;
}

void Renderer::Shutdown()
{
    DestroyShapeResources();
    ClearMeshCache();
    sphereBatch.mesh.Destroy();
    capsuleTopBatch.mesh.Destroy();
    capsuleBottomBatch.mesh.Destroy();
    capsuleMidBatch.mesh.Destroy();
    boxBatch.mesh.Destroy();
    triangleBatch.mesh.Destroy();
    shapeShader.Destroy();
    shadowShader.Destroy();
    geometryShader.Destroy();
    aoResolveShader.Destroy();
    aoShader.Destroy();
    aoBlurShader.Destroy();
    deferredShader.Destroy();
    presentShader.Destroy();
    DestroyShadowResources();
    DestroyFrameResources();
    primitiveShader.Destroy();
    DestroyPrimitiveResources();
    pointCount = 0;
    lineCount = 0;
    currentShapeShader = nullptr;
}

void Renderer::ClearMeshCache()
{
    InstancedMeshBatch* batches[] = {
        &sphereBatch, &capsuleTopBatch, &capsuleBottomBatch, &capsuleMidBatch, &boxBatch, &triangleBatch,
    };
    for (InstancedMeshBatch* batch : batches)
    {
        batch->instances[g_fill_pass].clear();
        batch->instances[g_outline_pass].clear();
    }

    for (auto& [hash, mesh] : heightFieldMeshes)
    {
        MuliNotUsed(hash);
        mesh.Destroy();
    }
    for (auto& [hash, mesh] : meshShapeMeshes)
    {
        MuliNotUsed(hash);
        mesh.Destroy();
    }

    shapeMeshes.clear();
    shapeMeshInstances[g_fill_pass].clear();
    shapeMeshInstances[g_outline_pass].clear();
    activeShapeMeshKeys[g_fill_pass].clear();
    activeShapeMeshKeys[g_outline_pass].clear();
    queuedShapeCount[g_fill_pass] = 0;
    queuedShapeCount[g_outline_pass] = 0;
    shapeMeshInstanceCount[g_fill_pass] = 0;
    shapeMeshInstanceCount[g_outline_pass] = 0;
    shapeMeshVertices.clear();
    shapeMeshIndices.clear();
    shapeMeshOutlineIndices.clear();
    shapeMeshVertexCapacity = 0;
    shapeMeshIndexCapacity = 0;
    shapeMeshOutlineIndexCapacity = 0;
    shapeMeshInstanceBuffer.clear();
    shapeMeshCommands.clear();
    heightFieldMeshes.clear();
    meshShapeMeshes.clear();

    if (shapeMeshVBO != 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, shapeMeshVBO);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    if (shapeMeshEBO != 0)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shapeMeshEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
    if (shapeMeshOutlineEBO != 0)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shapeMeshOutlineEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}

void Renderer::DrawShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe)
{
    Shader* shader = currentShapeShader ? currentShapeShader : &shapeShader;
    QueueShape(shape, transform, color, wireframe, *shader);
}

void Renderer::DrawShape(const Shape* shape, const Transform& transform, const DrawMode& mode)
{
    if (mode.fill == false && mode.outline == false)
    {
        return;
    }

    Vec4 color = GetModeColor(mode);

    if (mode.fill)
    {
        QueueShape(shape, transform, color, false, shapeShader);
    }

    if (mode.outline)
    {
        QueueShape(shape, transform, default_black, true, shapeShader);
    }
}

void Renderer::FlushShapes()
{
    Shader* shader = currentShapeShader ? currentShapeShader : &shapeShader;
    FlushQueuedShapes(*shader, false);
    FlushQueuedShapes(*shader, true);
}

void Renderer::QueueShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader)
{
    if (!shape)
    {
        return;
    }

    int32 pass = wireframe ? g_outline_pass : g_fill_pass;
    if (shape->GetType() == Shape::sphere)
    {
        const SphereShape* sphere = (const SphereShape*)shape;
        Mat3 rotation{ transform.q };
        float radius = sphere->GetRadius();
        Mat4 model{
            Vec4{ rotation.ex * radius, 0.0f },
            Vec4{ rotation.ey * radius, 0.0f },
            Vec4{ rotation.ez * radius, 0.0f },
            Vec4{ Mul(transform, sphere->GetCenter()), 1.0f },
        };
        sphereBatch.instances[pass].emplace_back(model, color);
    }
    else if (shape->GetType() == Shape::capsule)
    {
        const CapsuleShape* capsule = (const CapsuleShape*)shape;
        Vec3 a = Mul(transform, capsule->GetVertexA());
        Vec3 b = Mul(transform, capsule->GetVertexB());
        Vec3 axis = b - a;
        float height = axis.Normalize();
        float radius = capsule->GetRadius();
        Vec3 center = (a + b) * 0.5f;

        if (height == 0.0f)
        {
            axis = transform.q.Rotate(y_axis);
        }

        Vec3 localAxis = capsule->GetVertexB() - capsule->GetVertexA();
        if (localAxis.Normalize() == 0.0f)
        {
            localAxis = y_axis;
        }

        Vec3 localX;
        CoordinateSystem(localAxis, &localX);

        // Build the radial frame in local space so its orientation remains continuous as the body rotates.
        Vec3 x = transform.q.Rotate(localX);
        x -= Dot(x, axis) * axis;
        x.Normalize();
        Vec3 z = Cross(x, axis);
        float halfHeight = height * 0.5f;

        Mat4 model{
            Vec4{ x * radius, 0.0f },
            Vec4{ axis * radius, 0.0f },
            Vec4{ z * radius, 0.0f },
            Vec4{ center, 1.0f },
        };
        Mat4 topModel = model;
        topModel.ew = Vec4{ center + axis * halfHeight, 1.0f };

        Mat4 bottomModel = model;
        bottomModel.ew = Vec4{ center - axis * halfHeight, 1.0f };

        Mat4 midModel{
            Vec4{ x * radius, 0.0f },
            Vec4{ axis * halfHeight, 0.0f },
            Vec4{ z * radius, 0.0f },
            Vec4{ center, 1.0f },
        };

        capsuleTopBatch.instances[pass].emplace_back(topModel, color);
        capsuleBottomBatch.instances[pass].emplace_back(bottomModel, color);
        capsuleMidBatch.instances[pass].emplace_back(midModel, color);
    }
    else if (shape->GetType() == Shape::box)
    {
        const BoxShape* box = (const BoxShape*)shape;
        Mat3 rotation{ transform.q * box->GetRotation() };
        Vec3 halfExtents = box->GetHalfExtents();
        Mat4 model{
            Vec4{ rotation.ex * halfExtents.x, 0.0f },
            Vec4{ rotation.ey * halfExtents.y, 0.0f },
            Vec4{ rotation.ez * halfExtents.z, 0.0f },
            Vec4{ Mul(transform, box->GetCenter()), 1.0f },
        };
        boxBatch.instances[pass].emplace_back(model, color);
    }
    else if (shape->GetType() == Shape::convex)
    {
        const ConvexShape* convex = (const ConvexShape*)shape;
        uint64 key = convex->GetHash();
        GetConvexMesh(convex, key);

        std::vector<ShapeInstance>& instances = shapeMeshInstances[pass][key];
        if (instances.empty())
        {
            activeShapeMeshKeys[pass].push_back(key);
        }
        instances.emplace_back(Mat4(transform), color);
        ++shapeMeshInstanceCount[pass];
    }
    else if (shape->GetType() == Shape::triangle)
    {
        const TriangleShape* triangle = (const TriangleShape*)shape;
        Vec3 a = Mul(transform, triangle->GetVertex(0));
        Vec3 b = Mul(transform, triangle->GetVertex(1));
        Vec3 c = Mul(transform, triangle->GetVertex(2));
        Mat4 model{
            Vec4{ b - a, 0.0f },
            Vec4{ c - a, 0.0f },
            Vec4{ transform.q.Rotate(triangle->GetNormal()), 0.0f },
            Vec4{ a, 1.0f },
        };

        triangleBatch.instances[pass].emplace_back(model, color);
    }
    else if (shape->GetType() == Shape::polygon)
    {
        const PolygonShape* polygon = (const PolygonShape*)shape;
        uint64 key = polygon->GetHash();
        GetPolygonMesh(polygon, key);

        std::vector<ShapeInstance>& instances = shapeMeshInstances[pass][key];
        if (instances.empty())
        {
            activeShapeMeshKeys[pass].push_back(key);
        }
        instances.emplace_back(Mat4(transform), color);
        ++shapeMeshInstanceCount[pass];
    }
    else if (shape->GetType() == Shape::height_field)
    {
        DrawHeightField((const HeightFieldShape*)shape, transform, color, wireframe, shader);
        return;
    }
    else if (shape->GetType() == Shape::mesh)
    {
        DrawMeshShape((const MeshShape*)shape, transform, color, wireframe, shader);
        return;
    }

    ++queuedShapeCount[pass];
    if (queuedShapeCount[pass] == g_max_shape_batch_count)
    {
        FlushQueuedShapes(shader, wireframe);
    }
}

const Renderer::ShapeMeshRange& Renderer::GetConvexMesh(const ConvexShape* shape, uint64 hash)
{
    auto it = shapeMeshes.find(hash);
    if (it != shapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    std::vector<uint32> outlineIndices;
    BuildConvexMesh(&vertices, &indices, &outlineIndices, *shape);
    return StoreShapeMesh(hash, vertices, indices, outlineIndices);
}

const Renderer::ShapeMeshRange& Renderer::GetPolygonMesh(const PolygonShape* shape, uint64 hash)
{
    auto it = shapeMeshes.find(hash);
    if (it != shapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    std::vector<uint32> outlineIndices;
    BuildPolygonMesh(&vertices, &indices, &outlineIndices, *shape);
    return StoreShapeMesh(hash, vertices, indices, outlineIndices);
}

const Renderer::ShapeMeshRange& Renderer::StoreShapeMesh(
    uint64 hash, std::span<const MeshVertex> vertices, std::span<const uint32> indices, std::span<const uint32> outlineIndices
)
{
    ShapeMeshRange range;
    range.firstIndex = (GLuint)shapeMeshIndices.size();
    range.indexCount = (GLuint)indices.size();
    range.firstOutlineIndex = (GLuint)shapeMeshOutlineIndices.size();
    range.outlineIndexCount = (GLuint)outlineIndices.size();
    range.baseVertex = (GLint)shapeMeshVertices.size();

    size_t oldVertexSize = shapeMeshVertices.size() * sizeof(MeshVertex);
    size_t oldIndexSize = shapeMeshIndices.size() * sizeof(uint32);
    size_t oldOutlineIndexSize = shapeMeshOutlineIndices.size() * sizeof(uint32);

    shapeMeshVertices.insert(shapeMeshVertices.end(), vertices.begin(), vertices.end());
    shapeMeshIndices.insert(shapeMeshIndices.end(), indices.begin(), indices.end());
    shapeMeshOutlineIndices.insert(shapeMeshOutlineIndices.end(), outlineIndices.begin(), outlineIndices.end());

    glBindVertexArray(shapeMeshVAO);
    AppendStaticBuffer(
        GL_ARRAY_BUFFER, shapeMeshVBO, shapeMeshVertices.data(), oldVertexSize, shapeMeshVertices.size() * sizeof(MeshVertex),
        &shapeMeshVertexCapacity
    );
    AppendStaticBuffer(
        GL_ELEMENT_ARRAY_BUFFER, shapeMeshEBO, shapeMeshIndices.data(), oldIndexSize, shapeMeshIndices.size() * sizeof(uint32),
        &shapeMeshIndexCapacity
    );

    glBindVertexArray(shapeMeshOutlineVAO);
    AppendStaticBuffer(
        GL_ELEMENT_ARRAY_BUFFER, shapeMeshOutlineEBO, shapeMeshOutlineIndices.data(), oldOutlineIndexSize,
        shapeMeshOutlineIndices.size() * sizeof(uint32), &shapeMeshOutlineIndexCapacity
    );
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    auto [inserted, _] = shapeMeshes.emplace(hash, range);
    return inserted->second;
}

Mesh& Renderer::GetHeightFieldMesh(const HeightFieldShape* shape)
{
    uint64 hash = shape->GetHash();
    auto it = heightFieldMeshes.find(hash);
    if (it != heightFieldMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    BuildHeightFieldMesh(&vertices, &indices, *shape);

    Mesh& mesh = heightFieldMeshes[hash];
    mesh.Upload(vertices, indices, GL_TRIANGLES);

    glBindVertexArray(mesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return mesh;
}

Mesh& Renderer::GetMeshShapeMesh(const MeshShape* shape)
{
    uint64 hash = shape->GetHash();
    auto it = meshShapeMeshes.find(hash);
    if (it != meshShapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    BuildMeshShapeMesh(&vertices, &indices, *shape);

    Mesh& mesh = meshShapeMeshes[hash];
    mesh.Upload(vertices, indices, GL_TRIANGLES);
    glBindVertexArray(mesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return mesh;
}

void Renderer::DrawHeightField(
    const HeightFieldShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
)
{
    ShapeInstance instance{ Mat4(transform), color };
    Mesh& mesh = GetHeightFieldMesh(shape);

    if (wireframe)
    {
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
    }

    shader.Use();
    GLuint baseInstance = UploadShapeInstances(&instance, 1);
    mesh.DrawInstanced(1, false, baseInstance);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (wireframe)
    {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(GL_LESS);
    }
}

void Renderer::DrawMeshShape(
    const MeshShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
)
{
    ShapeInstance instance{ Mat4(transform), color };
    Mesh& mesh = GetMeshShapeMesh(shape);

    if (wireframe)
    {
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
    }

    shader.Use();
    GLuint baseInstance = UploadShapeInstances(&instance, 1);
    mesh.DrawInstanced(1, false, baseInstance);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (wireframe)
    {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(GL_LESS);
    }
}

void Renderer::DrawAABB(const AABB& aabb, const Vec4& color)
{
    const Vec3 v000{ aabb.min.x, aabb.min.y, aabb.min.z };
    const Vec3 v001{ aabb.min.x, aabb.min.y, aabb.max.z };
    const Vec3 v010{ aabb.min.x, aabb.max.y, aabb.min.z };
    const Vec3 v011{ aabb.min.x, aabb.max.y, aabb.max.z };
    const Vec3 v100{ aabb.max.x, aabb.min.y, aabb.min.z };
    const Vec3 v101{ aabb.max.x, aabb.min.y, aabb.max.z };
    const Vec3 v110{ aabb.max.x, aabb.max.y, aabb.min.z };
    const Vec3 v111{ aabb.max.x, aabb.max.y, aabb.max.z };

    DrawLine(v000, v001, color);
    DrawLine(v001, v011, color);
    DrawLine(v011, v010, color);
    DrawLine(v010, v000, color);
    DrawLine(v100, v101, color);
    DrawLine(v101, v111, color);
    DrawLine(v111, v110, color);
    DrawLine(v110, v100, color);
    DrawLine(v000, v100, color);
    DrawLine(v001, v101, color);
    DrawLine(v010, v110, color);
    DrawLine(v011, v111, color);
}

void Renderer::FlushQueuedShapes(const Shader& shader, bool wireframe)
{
    int32 pass = wireframe ? g_outline_pass : g_fill_pass;
    if (queuedShapeCount[pass] == 0)
    {
        return;
    }

    if (wireframe)
    {
        glDepthFunc(GL_LEQUAL);
    }

    shader.Use();
    FlushInstancedMesh(sphereBatch, pass, wireframe);
    FlushInstancedMesh(capsuleTopBatch, pass, wireframe);
    FlushInstancedMesh(capsuleBottomBatch, pass, wireframe);
    FlushInstancedMesh(capsuleMidBatch, pass, wireframe);
    FlushInstancedMesh(boxBatch, pass, wireframe);
    FlushInstancedMesh(triangleBatch, pass, wireframe);
    FlushShapeMeshes(pass, wireframe);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (wireframe)
    {
        glDepthFunc(GL_LESS);
    }

    queuedShapeCount[pass] = 0;
}

void Renderer::FlushInstancedMesh(InstancedMeshBatch& batch, int32 pass, bool wireframe)
{
    std::vector<ShapeInstance>& instances = batch.instances[pass];
    if (instances.empty())
    {
        return;
    }

    GLuint baseInstance = UploadShapeInstances(instances.data(), instances.size());
    batch.mesh.DrawInstanced((GLsizei)instances.size(), wireframe, baseInstance);
    instances.clear();
}

void Renderer::FlushShapeMeshes(int32 pass, bool wireframe)
{
    std::unordered_map<uint64, std::vector<ShapeInstance>>& batches = shapeMeshInstances[pass];
    if (shapeMeshInstanceCount[pass] == 0)
    {
        return;
    }
    shapeMeshInstanceBuffer.clear();
    shapeMeshCommands.clear();
    shapeMeshInstanceBuffer.reserve(shapeMeshInstanceCount[pass]);
    shapeMeshCommands.reserve(activeShapeMeshKeys[pass].size());

    for (uint64 key : activeShapeMeshKeys[pass])
    {
        std::vector<ShapeInstance>& instances = batches.at(key);

        auto rangeIt = shapeMeshes.find(key);
        MuliAssert(rangeIt != shapeMeshes.end());
        const ShapeMeshRange& range = rangeIt->second;

        DrawElementsIndirectCommand command;
        command.count = wireframe ? range.outlineIndexCount : range.indexCount;
        command.instanceCount = (GLuint)instances.size();
        command.firstIndex = wireframe ? range.firstOutlineIndex : range.firstIndex;
        command.baseVertex = range.baseVertex;
        command.baseInstance = (GLuint)shapeMeshInstanceBuffer.size();
        shapeMeshCommands.push_back(command);

        shapeMeshInstanceBuffer.insert(shapeMeshInstanceBuffer.end(), instances.begin(), instances.end());
        instances.clear();
    }

    if (!shapeMeshCommands.empty())
    {
        glBindVertexArray(wireframe ? shapeMeshOutlineVAO : shapeMeshVAO);
        GLuint baseInstance = UploadShapeInstances(shapeMeshInstanceBuffer.data(), shapeMeshInstanceBuffer.size());
        for (DrawElementsIndirectCommand& command : shapeMeshCommands)
        {
            command.baseInstance += baseInstance;
        }
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, shapeMeshIndirectVBO);
        glBufferData(
            GL_DRAW_INDIRECT_BUFFER, (GLsizeiptr)(shapeMeshCommands.size() * sizeof(DrawElementsIndirectCommand)),
            shapeMeshCommands.data(), GL_STREAM_DRAW
        );
        glMultiDrawElementsIndirect(
            wireframe ? GL_LINES : GL_TRIANGLES, GL_UNSIGNED_INT, nullptr, (GLsizei)shapeMeshCommands.size(), 0
        );
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    activeShapeMeshKeys[pass].clear();
    shapeMeshInstanceCount[pass] = 0;
}

void Renderer::FlushPoints(bool overlay)
{
    FlushPrimitive(GL_POINTS, points, pointCount, overlay);
    pointCount = 0;
}

void Renderer::FlushLines(bool overlay)
{
    FlushPrimitive(GL_LINES, lines, lineCount, overlay);
    lineCount = 0;
}

void Renderer::FlushPrimitive(GLenum primitive, const std::vector<Vertex>& vertices, int32 vertexCount, bool overlay)
{
    if (vertexCount == 0)
    {
        return;
    }

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, sceneFramebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT1);
    glViewport(0, 0, frameWidth, frameHeight);

    if (overlay)
    {
        glDisable(GL_DEPTH_TEST);
    }
    else
    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
    }
    glDepthMask(GL_FALSE);

    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_SHADING);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(lineWidth);

    primitiveShader.Use();
    primitiveShader.SetMat4("uView", viewMatrix);
    primitiveShader.SetMat4("uProjection", projectionMatrix);
    primitiveShader.SetFloat("uPointSize", pointSize);

    glBindVertexArray(primVAO);
    glBindBuffer(GL_ARRAY_BUFFER, primVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * primitiveCapacity, nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(vertexCount * sizeof(Vertex)), vertices.data());
    glDrawArrays(primitive, 0, vertexCount);
    glBindVertexArray(0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Renderer::EnsurePrimitiveCapacity(std::vector<Vertex>& vertices, int32 requiredCount)
{
    if (requiredCount > (int32)vertices.size())
    {
        int32 newCapacity = Max<int32>((int32)vertices.size() * 2, requiredCount);
        vertices.resize(newCapacity);
    }

    if (requiredCount <= primitiveCapacity)
    {
        return;
    }

    primitiveCapacity = Max<int32>(primitiveCapacity * 2, requiredCount);

    glBindBuffer(GL_ARRAY_BUFFER, primVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * primitiveCapacity, nullptr, GL_STREAM_DRAW);
}

Vec4 Renderer::GetColor(int32 colorIndex) const
{
    return g_colors[colorIndex % g_color_count];
}

void Renderer::BeginFrame(
    const Camera& camera,
    float aspectRatio,
    const Vec3& newSkyColor,
    float skyIntensity,
    const Vec3& newLightDirection,
    const Vec3& newLightColor,
    float newLightIntensity
)
{
    glGetIntegerv(GL_VIEWPORT, viewport);
    bool frameResourcesCreated = CreateFrameResources(Max(viewport[2], 1), Max(viewport[3], 1));
    MuliAssert(frameResourcesCreated);
    MuliNotUsed(frameResourcesCreated);
    shapeInstanceOffset = 0;
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(shapeInstanceCapacity * sizeof(ShapeInstance)), nullptr, GL_STREAM_DRAW);
    if (Length2(newLightDirection) > epsilon)
    {
        lightDirection = Normalize(newLightDirection);
    }
    lightColor = Vec3{ SrgbToLinear(newLightColor.x), SrgbToLinear(newLightColor.y), SrgbToLinear(newLightColor.z) };
    lightIntensity = newLightIntensity;
    cameraPosition = camera.GetPosition();
    skyColor = Vec3{ SrgbToLinear(newSkyColor.x), SrgbToLinear(newSkyColor.y), SrgbToLinear(newSkyColor.z) };
    environmentColor = skyColor * skyIntensity;
    lightViewProjectionMatrix = ComputeLightViewProjection(camera, aspectRatio, lightDirection);
    viewMatrix = camera.GetViewMatrix();
    projectionMatrix = camera.GetProjectionMatrix(aspectRatio);
}

void Renderer::BeginShadowPass()
{
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glViewport(0, 0, g_shadow_map_size, g_shadow_map_size);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glClear(GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_BACK);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_SHADING);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.5f, 1.0f);

    shadowShader.Use();
    shadowShader.SetMat4("uLightViewProjection", lightViewProjectionMatrix);
    currentShapeShader = &shadowShader;
}

void Renderer::EndShadowPass()
{
    FlushQueuedShapes(shadowShader, false);
    FlushQueuedShapes(shadowShader, true);

    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    currentShapeShader = nullptr;
}

void Renderer::BeginAoPass()
{
    glBindFramebuffer(GL_FRAMEBUFFER, geometryFramebuffer);
    GLenum geometryDrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
    glDrawBuffers(2, geometryDrawBuffers);
    glViewport(0, 0, frameWidth, frameHeight);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_SHADING);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    const float clearNormal[] = { 0.5f, 0.5f, 1.0f, 0.0f };
    const float clearAlbedo[] = { 0.0f, 0.0f, 0.0f, 0.0f };
    glClearBufferfv(GL_COLOR, 0, clearNormal);
    glClearBufferfv(GL_COLOR, 1, clearAlbedo);
    glClear(GL_DEPTH_BUFFER_BIT);

    geometryShader.Use();
    geometryShader.SetMat4("uView", viewMatrix);
    geometryShader.SetMat4("uProjection", projectionMatrix);
    currentShapeShader = &geometryShader;
}

void Renderer::EndAoPass()
{
    FlushQueuedShapes(geometryShader, false);

    int32 aoWidth = frameWidth;
    int32 aoHeight = frameHeight;
    glBindVertexArray(fullscreenVAO);

    // Resolve depth and normal from one representative MSAA sample.
    glBindFramebuffer(GL_FRAMEBUFFER, aoInputFramebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0, 0, aoWidth, aoHeight);
    glBindTextureUnit(0, geometryDepthTexture);
    glBindTextureUnit(1, geometryNormalTexture);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_SHADING);
    aoResolveShader.Use();
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glDepthFunc(GL_LESS);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glBindFramebuffer(GL_FRAMEBUFFER, aoFramebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0, 0, aoWidth, aoHeight);
    glBindTextureUnit(0, aoDepthTexture);
    glBindTextureUnit(1, aoNormalTexture);
    glBindTextureUnit(2, aoNoiseTexture);
    aoShader.Use();
    aoShader.SetMat4("uProjection", projectionMatrix);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glBindFramebuffer(GL_FRAMEBUFFER, aoBlurFramebuffer);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glBindTextureUnit(0, aoTexture);
    glBindTextureUnit(1, aoDepthTexture);
    glBindTextureUnit(2, aoNormalTexture);
    aoBlurShader.Use();
    aoBlurShader.SetMat4("uProjection", projectionMatrix);
    aoBlurShader.SetVec3("uInvAoSize", Vec3{ 1.0f / aoWidth, 1.0f / aoHeight, 0.0f });
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    currentShapeShader = nullptr;
}

void Renderer::BeginShapePass(bool shadeGeometry)
{
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFramebuffer);
    glViewport(0, 0, frameWidth, frameHeight);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glDisable(GL_LINE_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawBuffer(GL_COLOR_ATTACHMENT1);
    const float clearDebug[] = { 0.0f, 0.0f, 0.0f, 0.0f };
    glClearBufferfv(GL_COLOR, 0, clearDebug);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glBindTextureUnit(0, geometryDepthTexture);
    glBindTextureUnit(1, geometryNormalTexture);
    glBindTextureUnit(2, geometryAlbedoTexture);
    glBindTextureUnit(3, aoBlurTexture);
    glBindTextureUnit(4, shadowDepthTexture);
    deferredShader.Use();
    deferredShader.SetMat4("uView", viewMatrix);
    deferredShader.SetMat4("uInvView", viewMatrix.GetInverse());
    deferredShader.SetMat4("uProjection", projectionMatrix);
    deferredShader.SetMat4("uLightViewProjection", lightViewProjectionMatrix);
    deferredShader.SetVec3("uLightDirection", lightDirection);
    deferredShader.SetVec3("uLightColor", lightColor);
    deferredShader.SetFloat("uLightIntensity", lightIntensity);
    deferredShader.SetVec3("uSkyColor", environmentColor);
    deferredShader.SetVec3("uClearColor", skyColor);
    deferredShader.SetInt("uShadeGeometry", shadeGeometry ? 1 : 0);
    glBindVertexArray(fullscreenVAO);
    glEnable(GL_SAMPLE_SHADING);
    glMinSampleShading(1.0f);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisable(GL_SAMPLE_SHADING);

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);

    glBindTextureUnit(0, shadowDepthTexture);

    shapeShader.Use();
    shapeShader.SetMat4("uView", viewMatrix);
    shapeShader.SetMat4("uProjection", projectionMatrix);
    shapeShader.SetMat4("uLightViewProjection", lightViewProjectionMatrix);
    shapeShader.SetVec3("uLightDirection", lightDirection);
    shapeShader.SetVec3("uLightColor", lightColor);
    shapeShader.SetFloat("uLightIntensity", lightIntensity);
    shapeShader.SetVec3("uCameraPosition", cameraPosition);
    shapeShader.SetVec3("uSkyColor", environmentColor);
    currentShapeShader = &shapeShader;
}

void Renderer::EndFrame()
{
    FlushShapes();

    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SAMPLE_SHADING);
    glDisable(GL_MULTISAMPLE);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, sceneFramebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, presentFramebuffer);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glBlitFramebuffer(0, 0, frameWidth, frameHeight, 0, 0, frameWidth, frameHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glBindTextureUnit(0, presentColorTexture);
    glBindTextureUnit(1, debugColorTexture);
    presentShader.Use();
    glBindVertexArray(fullscreenVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_SHADING);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    currentShapeShader = nullptr;
}
} // namespace muli3
