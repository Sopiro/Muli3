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

constexpr const char* g_debugVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPosition;

uniform mat4 uView;
uniform mat4 uProjection;
uniform float uPointSize;

void main()
{
    gl_Position = uProjection * uView * vec4(aPosition, 1.0);
    gl_PointSize = uPointSize;
}
)";

constexpr const char* g_debugFragmentShader = R"(
#version 330 core
uniform vec3 uColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(uColor, 1.0);
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
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, g_shadowMapSize, g_shadowMapSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr
    );
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

bool Renderer::CreateDebugResources()
{
    if (!debugShader.Create(g_debugVertexShader, g_debugFragmentShader))
    {
        return false;
    }

    glGenVertexArrays(1, &debugVAO);
    glGenBuffers(1, &debugVBO);

    glBindVertexArray(debugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return debugVAO != 0 && debugVBO != 0;
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

void Renderer::DestroyDebugResources()
{
    if (debugVBO != 0)
    {
        glDeleteBuffers(1, &debugVBO);
        debugVBO = 0;
    }
    if (debugVAO != 0)
    {
        glDeleteVertexArrays(1, &debugVAO);
        debugVAO = 0;
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
    if (!CreateDebugResources())
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
    debugShader.Destroy();
    DestroyDebugResources();
    initialized = false;
}

void Renderer::DrawBody(const RigidBody& body, const Shader& shader) const
{
    if (!body.shape || body.shape->GetType() != ShapeType::sphere)
    {
        return;
    }

    const Sphere* sphere = (const Sphere*)body.shape;
    Transform renderTransform = body.transform;
    renderTransform.s = renderTransform.s * Vec3{ sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius() };
    shader.SetMat4("uModel", Mat4(renderTransform));
    sphereMesh.Draw();
}

void Renderer::DrawAABB(const AABB& aabb, std::vector<Vec3>& lines) const
{
    const Vec3 v000{ aabb.min.x, aabb.min.y, aabb.min.z };
    const Vec3 v001{ aabb.min.x, aabb.min.y, aabb.max.z };
    const Vec3 v010{ aabb.min.x, aabb.max.y, aabb.min.z };
    const Vec3 v011{ aabb.min.x, aabb.max.y, aabb.max.z };
    const Vec3 v100{ aabb.max.x, aabb.min.y, aabb.min.z };
    const Vec3 v101{ aabb.max.x, aabb.min.y, aabb.max.z };
    const Vec3 v110{ aabb.max.x, aabb.max.y, aabb.min.z };
    const Vec3 v111{ aabb.max.x, aabb.max.y, aabb.max.z };

    const Vec3 edges[] = {
        v000, v001, v001, v011, v011, v010, v010, v000, v100, v101, v101, v111,
        v111, v110, v110, v100, v000, v100, v001, v101, v010, v110, v011, v111,
    };
    lines.insert(lines.end(), std::begin(edges), std::end(edges));
}

void Renderer::DrawDebugPrimitives(
    const Mat4& view,
    const Mat4& projection,
    GLenum primitive,
    const std::vector<Vec3>& vertices,
    const Vec3& color,
    float pointSize
)
{
    if (vertices.empty())
    {
        return;
    }

    debugShader.Use();
    debugShader.SetMat4("uView", view);
    debugShader.SetMat4("uProjection", projection);
    debugShader.SetVec3("uColor", color);
    debugShader.SetFloat("uPointSize", pointSize);

    glBindVertexArray(debugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vertices.size() * sizeof(Vec3)), vertices.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(primitive, 0, (GLsizei)vertices.size());
    glBindVertexArray(0);
}

void Renderer::DrawDebug(const World& world, const Mat4& view, const Mat4& projection, const DebugOptions& options)
{
    std::vector<Vec3> aabbLines;
    if (options.show_aabb)
    {
        for (RigidBody* bodyPtr : world.GetRigidBodies())
        {
            const RigidBody& body = *bodyPtr;
            if (body.shape)
            {
                AABB aabb;
                body.shape->ComputeAABB(body.transform, &aabb);
                DrawAABB(aabb, aabbLines);
            }
        }
    }

    std::vector<Vec3> contactPoints;
    std::vector<Vec3> contactNormalLines;
    if (options.show_contact_point || options.show_contact_normal)
    {
        for (const Contact* contact = world.GetContacts(); contact; contact = contact->GetNext())
        {
            if (contact->IsEnabled() == false || contact->IsTouching() == false)
            {
                continue;
            }

            const ContactManifold& manifold = contact->GetContactManifold();

            for (int32 i = 0; i < manifold.contactCount; ++i)
            {
                const Vec3 p1 = manifold.contactPoints[i].p;

                if (options.show_contact_point)
                {
                    contactPoints.push_back(p1);
                }

                if (options.show_contact_normal)
                {
                    const Vec3 p2 = p1 + manifold.contactNormal * 0.18f;
                    const Vec3 reference =
                        Abs(manifold.contactNormal.y) < 0.8f ? Vec3{ 0.0f, 1.0f, 0.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
                    const Vec3 tangent = NormalizeSafe(Cross(manifold.contactNormal, reference));
                    const Vec3 arrowBase = p2 - manifold.contactNormal * 0.04f;
                    const Vec3 arrowA = arrowBase + tangent * 0.02f;
                    const Vec3 arrowB = arrowBase - tangent * 0.02f;

                    contactNormalLines.push_back(p1);
                    contactNormalLines.push_back(p2);
                    contactNormalLines.push_back(p2);
                    contactNormalLines.push_back(arrowA);
                    contactNormalLines.push_back(p2);
                    contactNormalLines.push_back(arrowB);
                }
            }
        }
    }

    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(2.0f);

    DrawDebugPrimitives(view, projection, GL_LINES, aabbLines, Vec3{ 0.08f, 0.09f, 0.10f });
    DrawDebugPrimitives(view, projection, GL_POINTS, contactPoints, Vec3{ 1.0f, 0.18f, 0.08f }, 5.0f);
    DrawDebugPrimitives(view, projection, GL_LINES, contactNormalLines, Vec3{ 0.05f, 0.25f, 1.0f });

    glLineWidth(1.0f);
    if (depthTestEnabled)
    {
        glEnable(GL_DEPTH_TEST);
    }
}

void Renderer::Render(const World& world, const Camera& camera, float aspectRatio, const DebugOptions& options)
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

    if (options.draw_body || options.draw_wireframe)
    {
        for (RigidBody* bodyPtr : world.GetRigidBodies())
        {
            DrawBody(*bodyPtr, shadowShader);
        }
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

    if (options.draw_body)
    {
        for (RigidBody* bodyPtr : world.GetRigidBodies())
        {
            const RigidBody& body = *bodyPtr;
            surfaceShader.SetVec3("uBaseColor", body.IsStatic() ? Vec3{ 0.92f, 0.92f, 0.92f } : Vec3{ 1.0f, 1.0f, 1.0f });
            DrawBody(body, surfaceShader);
        }
    }

    if (options.draw_wireframe)
    {
        GLint previousDepthFunc = GL_LESS;
        glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunc);
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
        surfaceShader.SetVec3("uBaseColor", Vec3{ 0.03f, 0.04f, 0.05f });
        for (RigidBody* bodyPtr : world.GetRigidBodies())
        {
            DrawBody(*bodyPtr, surfaceShader);
        }
        glEnable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(previousDepthFunc);
    }

    DrawDebug(world, view, projection, options);
}

} // namespace muli3
