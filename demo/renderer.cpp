#include "renderer.h"
#include "muli3/color.h"

namespace muli3
{

constexpr int g_shadowMapSize = 4096;
constexpr size_t g_maxShapeBatchCount = 4096;
constexpr int32 g_maxVertexCount = 1024 * 4;
constexpr int32 g_colorCount = 10;
constexpr int32 g_fillPass = 0;
constexpr int32 g_outlinePass = 1;
constexpr float g_shadowViewDistance = 50.0f;
constexpr float g_shadowBoundsPadding = 2.0f;
constexpr float g_shadowDepthPadding = 32.0f;

Vec4 g_colors[g_colorCount];
Vec4 g_colors2[constraint_color_count];
bool g_colorsInitialized = false;

void HashCombine(size_t* seed, size_t value)
{
    *seed ^= value + 0x9e3779b97f4a7c15ull + (*seed << 6) + (*seed >> 2);
}

void HashCombineFloat(size_t* seed, float value)
{
    HashCombine(seed, std::hash<uint32>{}(std::bit_cast<uint32>(value)));
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
    vec3 worldNormal = normalize(mat3(model) * aNormal);
    vWorldPosition = worldPosition.xyz;
    vWorldNormal = worldNormal;
    vTexCoord = aTexCoord;
    vShadowPosition = uLightViewProjection * worldPosition;
    vBaseColor = iColor.rgb;
    gl_Position = uProjection * uView * worldPosition;
}
)";

constexpr const char* g_shapeFragmentShader = R"(
#version 330 core
in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vShadowPosition;
in vec3 vBaseColor;

uniform vec3 uLightDirection;
uniform sampler2DShadow uShadowMap;

out vec4 FragColor;

float CheckerMask(vec2 uv)
{
    vec2 grid = floor(uv);
    return mod(grid.x + grid.y, 2.0);
}

float ComputeShadow(vec3 normal, vec3 lightDir)
{
    vec3 projCoords = vShadowPosition.xyz / vShadowPosition.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (any(lessThan(projCoords, vec3(0.0))) || any(greaterThan(projCoords, vec3(1.0))))
    {
        return 1.0;
    }

    float nDotL = clamp(dot(normal, lightDir), 0.001, 1.0);
    float tanAngle = sqrt(1.0 - nDotL * nDotL) / nDotL;
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
    float edgeFade = clamp(min(edge.x, edge.y) * 10.0, 0.0, 1.0);
    return mix(1.0, visibility, edgeFade);
}

void main()
{
    vec2 tiledUv = vTexCoord * vec2(8.0, 6.0);
    float checker = CheckerMask(tiledUv);

    vec3 colorA = vec3(0.96, 0.96, 0.96);
    vec3 colorB = vec3(0.76, 0.76, 0.76);
    vec3 albedo = mix(colorA, colorB, checker) * vBaseColor;

    vec3 normal = normalize(vWorldNormal);
    vec3 lightDir = normalize(-uLightDirection);
    float diffuse = max(dot(normal, lightDir), 0.0);
    float shadow = ComputeShadow(normal, lightDir);
    float ambient = 0.58;
    float lighting = ambient + diffuse * shadow * (1.0 - ambient);

    FragColor = vec4(albedo * lighting, 1.0);
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
    float zFar = g_shadowViewDistance;
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

    Vec3 lightPosition = frustumCenter - lightDirection * (frustumRadius + g_shadowDepthPadding);
    Mat4 lightView = Mat4::LookAt(lightPosition, frustumCenter, y_axis);

    Vec3 lightMin{ max_float };
    Vec3 lightMax{ -max_float };
    for (int32 i = 0; i < 8; ++i)
    {
        Vec4 p = lightView * Vec4{ frustumCorners[i], 1.0f };
        lightMin = Min(lightMin, Vec3{ p.x, p.y, p.z });
        lightMax = Max(lightMax, Vec3{ p.x, p.y, p.z });
    }

    float halfWidth = Max((lightMax.x - lightMin.x) * 0.5f + g_shadowBoundsPadding, 8.0f);
    float halfHeight = Max((lightMax.y - lightMin.y) * 0.5f + g_shadowBoundsPadding, 8.0f);

    Vec2 center{ (lightMin.x + lightMax.x) * 0.5f, (lightMin.y + lightMax.y) * 0.5f };

    float zNear = Max(-lightMax.z - g_shadowDepthPadding, 0.1f);
    float zFar = Max(-lightMin.z + g_shadowBoundsPadding, zNear + 1.0f);
    Mat4 lightProjection =
        Mat4::Orth(center.x - halfWidth, center.x + halfWidth, center.y - halfHeight, center.y + halfHeight, zNear, zFar);

    // Snap the final shadow matrix to texel increments so camera movement does not shimmer the map.
    Mat4 lightViewProjection = lightProjection * lightView;
    Vec4 shadowOrigin = lightViewProjection * Vec4{ Vec3::zero, 1.0f };
    shadowOrigin *= g_shadowMapSize * 0.5f;

    Vec4 roundedOrigin{
        std::floor(shadowOrigin.x + 0.5f),
        std::floor(shadowOrigin.y + 0.5f),
        shadowOrigin.z,
        shadowOrigin.w,
    };
    Vec4 roundOffset = (roundedOrigin - shadowOrigin) * (2.0f / g_shadowMapSize);
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

    constexpr float stride = 360.0f / g_colorCount;
    for (int32 i = 0; i < g_colorCount; ++i)
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

    return g_colors[mode.colorIndex % g_colorCount];
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
        GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, g_shadowMapSize, g_shadowMapSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr
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
    return status == GL_FRAMEBUFFER_COMPLETE;
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * g_maxVertexCount, nullptr, GL_STREAM_DRAW);
    primitiveCapacity = g_maxVertexCount;
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
}

void Renderer::UploadShapeInstances(const ShapeInstance* instances, size_t count)
{
    MuliAssert(count <= g_maxShapeBatchCount);

    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, g_maxShapeBatchCount * sizeof(ShapeInstance), nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(count * sizeof(ShapeInstance)), instances);
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
    glBufferData(GL_ARRAY_BUFFER, g_maxShapeBatchCount * sizeof(ShapeInstance), nullptr, GL_STREAM_DRAW);

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
    if (!shapeShader.Create(g_shapeVertexShader, g_shapeFragmentShader))
    {
        return false;
    }
    if (!shadowShader.Create(g_shadowVertexShader, g_shadowFragmentShader))
    {
        return false;
    }
    if (!CreateShadowResources())
    {
        return false;
    }
    if (!CreatePrimitiveResources())
    {
        return false;
    }
    InitializeColors();

    shapeShader.Use();
    shapeShader.SetInt("uShadowMap", 0);

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
        return false;
    }

    InstancedMeshBatch* batches[] = {
        &sphereBatch, &capsuleTopBatch, &capsuleBottomBatch, &capsuleMidBatch, &boxBatch, &triangleBatch,
    };
    for (InstancedMeshBatch* batch : batches)
    {
        batch->instances[g_fillPass].reserve(g_maxShapeBatchCount);
        batch->instances[g_outlinePass].reserve(g_maxShapeBatchCount);
    }
    points.resize(g_maxVertexCount);
    lines.resize(g_maxVertexCount);
    initialized = true;
    return true;
}

void Renderer::Shutdown()
{
    if (!initialized)
    {
        return;
    }

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
    DestroyShadowResources();
    primitiveShader.Destroy();
    DestroyPrimitiveResources();
    initialized = false;
}

void Renderer::ClearMeshCache()
{
    InstancedMeshBatch* batches[] = {
        &sphereBatch, &capsuleTopBatch, &capsuleBottomBatch, &capsuleMidBatch, &boxBatch, &triangleBatch,
    };
    for (InstancedMeshBatch* batch : batches)
    {
        batch->instances[g_fillPass].clear();
        batch->instances[g_outlinePass].clear();
    }

    for (auto& [shape, mesh] : heightFieldMeshes)
    {
        MuliNotUsed(shape);
        mesh.Destroy();
    }
    for (auto& [shape, mesh] : meshShapeMeshes)
    {
        MuliNotUsed(shape);
        mesh.Destroy();
    }

    shapeMeshes.clear();
    shapeMeshKeyCache.clear();
    shapeMeshInstances[g_fillPass].clear();
    shapeMeshInstances[g_outlinePass].clear();
    activeShapeMeshKeys[g_fillPass].clear();
    activeShapeMeshKeys[g_outlinePass].clear();
    queuedShapeCount[g_fillPass] = 0;
    queuedShapeCount[g_outlinePass] = 0;
    shapeMeshInstanceCount[g_fillPass] = 0;
    shapeMeshInstanceCount[g_outlinePass] = 0;
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

    int32 pass = wireframe ? g_outlinePass : g_fillPass;

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

        Vec3 localX = x_axis - Dot(x_axis, localAxis) * localAxis;
        if (localX.Normalize() == 0.0f)
        {
            localX = z_axis - Dot(z_axis, localAxis) * localAxis;
            localX.Normalize();
        }

        Vec3 x = transform.q.Rotate(localX);
        x = x - Dot(x, axis) * axis;
        if (x.Normalize() == 0.0f)
        {
            x = z_axis - Dot(z_axis, axis) * axis;
            if (x.Normalize() == 0.0f)
            {
                x = x_axis;
            }
        }
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
        size_t key = GetShapeMeshKey(shape);
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
        size_t key = GetShapeMeshKey(shape);
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
    if (queuedShapeCount[pass] == g_maxShapeBatchCount)
    {
        FlushQueuedShapes(shader, wireframe);
    }
}

size_t Renderer::GetShapeMeshKey(const Shape* shape)
{
    auto [it, inserted] = shapeMeshKeyCache.try_emplace(shape);
    ShapeMeshKeyCache& cache = it->second;
    if (!inserted && cache.frame == frame)
    {
        return cache.key;
    }

    if (shape->GetType() == Shape::convex)
    {
        cache.key = GetConvexMeshKey((const ConvexShape*)shape);
    }
    else
    {
        MuliAssert(shape->GetType() == Shape::polygon);
        cache.key = GetPolygonMeshKey((const PolygonShape*)shape);
    }
    cache.frame = frame;
    return cache.key;
}

size_t Renderer::GetConvexMeshKey(const ConvexShape* shape) const
{
    MuliAssert(shape != nullptr);

    size_t hash = 0;
    HashCombine(&hash, Shape::convex);
    HashCombine(&hash, std::hash<int32>{}(shape->GetVertexCount()));

    for (const Vec3& v : shape->GetVertices())
    {
        HashCombineFloat(&hash, v.x);
        HashCombineFloat(&hash, v.y);
        HashCombineFloat(&hash, v.z);
    }

    std::span<const int32> faceIndices = shape->GetIndices();
    for (const Face& face : shape->GetFaces())
    {
        HashCombine(&hash, std::hash<int32>{}(face.vertexCount));
        for (int32 i = 0; i < face.vertexCount; ++i)
        {
            HashCombine(&hash, std::hash<int32>{}(faceIndices[face.vertexStart + i]));
        }
    }

    for (const Face& face : shape->GetFaces())
    {
        const Vec3& n = face.normal;
        HashCombineFloat(&hash, n.x);
        HashCombineFloat(&hash, n.y);
        HashCombineFloat(&hash, n.z);
    }

    return hash;
}

size_t Renderer::GetPolygonMeshKey(const PolygonShape* shape) const
{
    MuliAssert(shape != nullptr);

    size_t hash = 0;
    HashCombine(&hash, Shape::polygon);
    HashCombine(&hash, std::hash<int32>{}(shape->GetVertexCount()));
    for (const Vec3& vertex : shape->GetVertices())
    {
        HashCombineFloat(&hash, vertex.x);
        HashCombineFloat(&hash, vertex.y);
        HashCombineFloat(&hash, vertex.z);
    }

    return hash;
}

const Renderer::ShapeMeshRange& Renderer::GetConvexMesh(const ConvexShape* shape, size_t key)
{
    auto it = shapeMeshes.find(key);
    if (it != shapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    std::vector<uint32> outlineIndices;
    BuildConvexMesh(&vertices, &indices, &outlineIndices, *shape);
    return StoreShapeMesh(key, vertices, indices, outlineIndices);
}

const Renderer::ShapeMeshRange& Renderer::GetPolygonMesh(const PolygonShape* shape, size_t key)
{
    auto it = shapeMeshes.find(key);
    if (it != shapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    std::vector<uint32> outlineIndices;
    BuildPolygonMesh(&vertices, &indices, &outlineIndices, *shape);
    return StoreShapeMesh(key, vertices, indices, outlineIndices);
}

const Renderer::ShapeMeshRange& Renderer::StoreShapeMesh(
    size_t key, std::span<const MeshVertex> vertices, std::span<const uint32> indices, std::span<const uint32> outlineIndices
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

    auto [inserted, _] = shapeMeshes.emplace(key, range);
    return inserted->second;
}

Mesh& Renderer::GetHeightFieldMesh(const HeightFieldShape* shape)
{
    auto it = heightFieldMeshes.find(shape);
    if (it != heightFieldMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    BuildHeightFieldMesh(&vertices, &indices, *shape);

    Mesh& mesh = heightFieldMeshes[shape];
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
    auto it = meshShapeMeshes.find(shape);
    if (it != meshShapeMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    BuildMeshShapeMesh(&vertices, &indices, *shape);

    Mesh& mesh = meshShapeMeshes[shape];
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

    GLint previousDepthFunc = GL_LESS;
    GLboolean cullFaceEnabled = GL_FALSE;
    if (wireframe)
    {
        glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunc);
        cullFaceEnabled = glIsEnabled(GL_CULL_FACE);
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
    }

    shader.Use();
    UploadShapeInstances(&instance, 1);
    mesh.DrawInstanced(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (wireframe)
    {
        if (cullFaceEnabled)
        {
            glEnable(GL_CULL_FACE);
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(previousDepthFunc);
    }
}

void Renderer::DrawMeshShape(
    const MeshShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
)
{
    ShapeInstance instance{ Mat4(transform), color };
    Mesh& mesh = GetMeshShapeMesh(shape);

    GLint previousDepthFunc = GL_LESS;
    GLboolean cullFaceEnabled = GL_FALSE;
    if (wireframe)
    {
        glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunc);
        cullFaceEnabled = glIsEnabled(GL_CULL_FACE);
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
    }

    shader.Use();
    UploadShapeInstances(&instance, 1);
    mesh.DrawInstanced(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (wireframe)
    {
        if (cullFaceEnabled)
        {
            glEnable(GL_CULL_FACE);
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(previousDepthFunc);
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
    int32 pass = wireframe ? g_outlinePass : g_fillPass;
    if (queuedShapeCount[pass] == 0)
    {
        return;
    }

    GLint previousDepthFunc = GL_LESS;
    if (wireframe)
    {
        glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunc);
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
        glDepthFunc(previousDepthFunc);
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

    UploadShapeInstances(instances.data(), instances.size());
    batch.mesh.DrawInstanced((GLsizei)instances.size(), wireframe);
    instances.clear();
}

void Renderer::FlushShapeMeshes(int32 pass, bool wireframe)
{
    std::unordered_map<size_t, std::vector<ShapeInstance>>& batches = shapeMeshInstances[pass];
    if (shapeMeshInstanceCount[pass] == 0)
    {
        return;
    }
    shapeMeshInstanceBuffer.clear();
    shapeMeshCommands.clear();
    shapeMeshInstanceBuffer.reserve(shapeMeshInstanceCount[pass]);
    shapeMeshCommands.reserve(activeShapeMeshKeys[pass].size());

    for (size_t key : activeShapeMeshKeys[pass])
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
        UploadShapeInstances(shapeMeshInstanceBuffer.data(), shapeMeshInstanceBuffer.size());
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

void Renderer::FlushPoints()
{
    FlushPrimitive(GL_POINTS, points, pointCount);
    pointCount = 0;
}

void Renderer::FlushLines()
{
    FlushPrimitive(GL_LINES, lines, lineCount);
    lineCount = 0;
}

void Renderer::FlushPrimitive(GLenum primitive, const std::vector<Vertex>& vertices, int32 vertexCount)
{
    if (vertexCount == 0)
    {
        return;
    }

    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

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

    if (depthTestEnabled)
    {
        glEnable(GL_DEPTH_TEST);
    }
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
    return g_colors[colorIndex % g_colorCount];
}

void Renderer::BeginFrame(const Camera& camera, float aspectRatio)
{
    ++frame;
    lightDirection = Normalize(Vec3{ 0.45f, -1.0f, -0.35f });
    lightViewProjectionMatrix = ComputeLightViewProjection(camera, aspectRatio, lightDirection);
    SetViewMatrix(camera.GetViewMatrix());
    SetProjectionMatrix(camera.GetProjectionMatrix(aspectRatio));
}

void Renderer::BeginShadowPass()
{
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glViewport(0, 0, g_shadowMapSize, g_shadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_BACK);
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
    glCullFace(GL_BACK);
    currentShapeShader = nullptr;
}

void Renderer::BeginShapePass()
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture);

    shapeShader.Use();
    shapeShader.SetMat4("uView", viewMatrix);
    shapeShader.SetMat4("uProjection", projectionMatrix);
    shapeShader.SetMat4("uLightViewProjection", lightViewProjectionMatrix);
    shapeShader.SetVec3("uLightDirection", lightDirection);
    currentShapeShader = &shapeShader;
}

void Renderer::EndFrame()
{
    FlushShapes();

    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(lineWidth);

    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();

    glLineWidth(1.0f);
    if (depthTestEnabled)
    {
        glEnable(GL_DEPTH_TEST);
    }

    currentShapeShader = nullptr;
}
} // namespace muli3
