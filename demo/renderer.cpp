#include "renderer.h"

namespace muli3
{

namespace
{

constexpr int g_shadowMapSize = 2048;

constexpr const char* g_surfaceVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform mat4 uLightViewProjection;

out vec3 vWorldPosition;
out vec3 vWorldNormal;
out vec2 vTexCoord;
out vec4 vShadowPosition;

void main()
{
    vec4 worldPosition = uModel * vec4(aPosition, 1.0);
    vWorldPosition = worldPosition.xyz;
    vWorldNormal = normalize(mat3(uModel) * aNormal);
    vTexCoord = aTexCoord;
    vShadowPosition = uLightViewProjection * worldPosition;
    gl_Position = uProjection * uView * worldPosition;
}
)";

constexpr const char* g_surfaceFragmentShader = R"(
#version 330 core
in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vShadowPosition;

uniform vec3 uBaseColor;
uniform vec3 uLightDirection;
uniform sampler2D uShadowMap;

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

    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 || projCoords.y < 0.0 || projCoords.y > 1.0)
    {
        return 1.0;
    }

    float bias = max(0.0008, 0.0040 * (1.0 - dot(normal, lightDir)));
    vec2 texelSize = 1.0 / vec2(textureSize(uShadowMap, 0));

    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            float closestDepth = texture(uShadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            visibility += (projCoords.z - bias) <= closestDepth ? 1.0 : 0.0;
        }
    }

    return visibility / 9.0;
}

void main()
{
    vec2 uv = vec2(fract(vTexCoord.x), clamp(vTexCoord.y, 0.0, 1.0));
    vec2 tiledUv = uv * vec2(16.0, 12.0);
    float checker = CheckerMask(tiledUv);

    vec3 colorA = vec3(0.96, 0.96, 0.96);
    vec3 colorB = vec3(0.76, 0.76, 0.76);
    vec3 albedo = mix(colorA, colorB, checker) * uBaseColor;

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

uniform mat4 uModel;
uniform mat4 uLightViewProjection;

void main()
{
    gl_Position = uLightViewProjection * uModel * vec4(aPosition, 1.0);
}
)";

constexpr const char* g_shadowFragmentShader = R"(
#version 330 core
void main()
{
}
)";

Mat4 ComputeLightViewProjection(const Vec3& lightDirection)
{
    const Vec3 sceneCenter{ 0.0f, 2.0f, 0.0f };
    const Vec3 lightPosition = sceneCenter - lightDirection * 22.0f;
    const Mat4 lightView = Mat4::LookAt(lightPosition, sceneCenter, Vec3{ 0.0f, 1.0f, 0.0f });
    const Mat4 lightProjection = Mat4::Orth(-20.0f, 20.0f, -20.0f, 20.0f, 1.0f, 64.0f);
    return lightProjection * lightView;
}

} // namespace

Renderer::~Renderer()
{
    Shutdown();
}

bool Renderer::CreateShadowResources()
{
    glGenFramebuffers(1, &shadowFramebuffer);
    glGenTextures(1, &shadowDepthTexture);

    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, g_shadowMapSize, g_shadowMapSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
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

bool Renderer::Initialize()
{
    if (!surfaceShader.Create(g_surfaceVertexShader, g_surfaceFragmentShader))
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

    surfaceShader.Use();
    surfaceShader.SetInt("uShadowMap", 0);

    sphereMesh.Upload(BuildSphereVertices(48, 24), BuildSphereIndices(48, 24), GL_TRIANGLES);
    initialized = true;
    return true;
}

void Renderer::Shutdown()
{
    if (!initialized)
    {
        return;
    }

    sphereMesh.Destroy();
    surfaceShader.Destroy();
    shadowShader.Destroy();
    DestroyShadowResources();
    initialized = false;
}

void Renderer::Render(const World& world, const Camera& camera, float aspectRatio) const
{
    const Mat4 view = camera.GetViewMatrix();
    const Mat4 projection = camera.GetProjectionMatrix(aspectRatio);
    const Vec3 lightDirection = Normalize(Vec3{ 0.45f, -1.0f, 0.35f });
    const Mat4 lightViewProjection = ComputeLightViewProjection(lightDirection);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glViewport(0, 0, g_shadowMapSize, g_shadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_FRONT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);

    shadowShader.Use();
    shadowShader.SetMat4("uLightViewProjection", lightViewProjection);

    for (const std::unique_ptr<RigidBody>& bodyPtr : world.GetRigidBodies())
    {
        const RigidBody& body = *bodyPtr;

        if (!body.visible || !body.shape || body.shape->GetType() != ShapeType::sphere)
        {
            continue;
        }

        const Sphere* sphere = (const Sphere*)body.shape;
        Transform renderTransform = body.transform;
        renderTransform.scale = renderTransform.scale * Vec3{ sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius() };
        const Mat4 model = MakeTransformMatrix(renderTransform);
        shadowShader.SetMat4("uModel", model);
        sphereMesh.Draw();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glCullFace(GL_BACK);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture);

    surfaceShader.Use();
    surfaceShader.SetMat4("uView", view);
    surfaceShader.SetMat4("uProjection", projection);
    surfaceShader.SetMat4("uLightViewProjection", lightViewProjection);
    surfaceShader.SetVec3("uLightDirection", lightDirection);

    for (const std::unique_ptr<RigidBody>& bodyPtr : world.GetRigidBodies())
    {
        const RigidBody& body = *bodyPtr;

        if (!body.visible || !body.shape || body.shape->GetType() != ShapeType::sphere)
        {
            continue;
        }

        const Sphere* sphere = (const Sphere*)body.shape;
        Transform renderTransform = body.transform;
        renderTransform.scale = renderTransform.scale * Vec3{ sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius() };
        const Mat4 model = MakeTransformMatrix(renderTransform);
        surfaceShader.SetMat4("uModel", model);
        surfaceShader.SetVec3("uBaseColor", body.IsStatic() ? Vec3{ 0.92f, 0.92f, 0.92f } : Vec3{ 1.0f, 1.0f, 1.0f });
        sphereMesh.Draw();
    }
}

} // namespace muli3
