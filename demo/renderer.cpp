#include "renderer.h"

namespace muli3
{

namespace
{

constexpr int g_shadowMapSize = 2048;
constexpr size_t g_maxSphereBatchCount = 4096;
constexpr size_t g_maxDebugVertexCount = 8192;

constexpr const char* g_surfaceVertexShader = R"(
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
    vWorldPosition = worldPosition.xyz;
    vWorldNormal = normalize(mat3(model) * aNormal);
    vTexCoord = aTexCoord;
    vShadowPosition = uLightViewProjection * worldPosition;
    vBaseColor = iColor.rgb;
    gl_Position = uProjection * uView * worldPosition;
}
)";

constexpr const char* g_surfaceFragmentShader = R"(
#version 330 core
in vec3 vWorldPosition;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vShadowPosition;
in vec3 vBaseColor;

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
    gl_Position = uLightViewProjection * model * vec4(aPosition, 1.0);
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
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProjection;
uniform float uPointSize;

out vec3 vColor;

void main()
{
    gl_Position = uProjection * uView * vec4(aPosition, 1.0);
    gl_PointSize = uPointSize;
    vColor = aColor;
}
)";

constexpr const char* g_debugFragmentShader = R"(
#version 330 core
in vec3 vColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(vColor, 1.0);
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

void SetInstanceAttribute(GLuint index, GLint size, GLsizei stride, size_t offset)
{
    glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offset));
    glEnableVertexAttribArray(index);
    glVertexAttribDivisor(index, 1);
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
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), reinterpret_cast<void*>(offsetof(DebugVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), reinterpret_cast<void*>(offsetof(DebugVertex, color)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return debugVAO != 0 && debugVBO != 0;
}

bool Renderer::CreateBatchResources()
{
    glGenBuffers(1, &sphereInstanceVBO);

    glBindVertexArray(sphereMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, sphereInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, g_maxSphereBatchCount * sizeof(SphereInstance), nullptr, GL_DYNAMIC_DRAW);

    SetInstanceAttribute(3, 4, sizeof(SphereInstance), offsetof(SphereInstance, model0));
    SetInstanceAttribute(4, 4, sizeof(SphereInstance), offsetof(SphereInstance, model1));
    SetInstanceAttribute(5, 4, sizeof(SphereInstance), offsetof(SphereInstance, model2));
    SetInstanceAttribute(6, 4, sizeof(SphereInstance), offsetof(SphereInstance, model3));
    SetInstanceAttribute(7, 4, sizeof(SphereInstance), offsetof(SphereInstance, color));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return sphereInstanceVBO != 0;
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

void Renderer::DestroyBatchResources()
{
    if (sphereInstanceVBO != 0)
    {
        glDeleteBuffers(1, &sphereInstanceVBO);
        sphereInstanceVBO = 0;
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
    if (!CreateBatchResources())
    {
        return false;
    }

    sphereInstances.reserve(g_maxSphereBatchCount);
    points.reserve(g_maxDebugVertexCount);
    lines.reserve(g_maxDebugVertexCount);
    initialized = true;
    return true;
}

void Renderer::Shutdown()
{
    if (!initialized)
    {
        return;
    }

    DestroyBatchResources();
    sphereMesh.Destroy();
    surfaceShader.Destroy();
    shadowShader.Destroy();
    DestroyShadowResources();
    debugShader.Destroy();
    DestroyDebugResources();
    initialized = false;
}

void Renderer::DrawBody(const RigidBody& body, const Vec3& color, const Shader& shader)
{
    const Shape* shape = body.GetShape();
    if (!shape || shape->GetType() != ShapeType::sphere)
    {
        return;
    }

    const Sphere* sphere = (const Sphere*)shape;
    Transform renderTransform = body.transform;
    renderTransform.s = renderTransform.s * Vec3{ sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius() };
    Mat4 model(renderTransform);
    sphereInstances.push_back(SphereInstance{ model.ex, model.ey, model.ez, model.ew, Vec4{ color, 1.0f } });

    if (sphereInstances.size() == g_maxSphereBatchCount)
    {
        FlushSpheres(shader);
    }
}

void Renderer::DrawPoint(const Vec3& point, const Vec3& color)
{
    points.push_back(DebugVertex{ point, color });
}

void Renderer::DrawLine(const Vec3& p1, const Vec3& p2, const Vec3& color)
{
    lines.push_back(DebugVertex{ p1, color });
    lines.push_back(DebugVertex{ p2, color });
}

void Renderer::DrawAABB(const AABB& aabb, const Vec3& color)
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

void Renderer::FlushSpheres(const Shader& shader)
{
    if (sphereInstances.empty())
    {
        return;
    }

    shader.Use();
    glBindBuffer(GL_ARRAY_BUFFER, sphereInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(sphereInstances.size() * sizeof(SphereInstance)), sphereInstances.data());
    sphereMesh.DrawInstanced((GLsizei)sphereInstances.size());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    sphereInstances.clear();
}

void Renderer::FlushPoints(const Mat4& view, const Mat4& projection, float pointSize)
{
    FlushDebugVertices(view, projection, GL_POINTS, points, pointSize);
    points.clear();
}

void Renderer::FlushLines(const Mat4& view, const Mat4& projection)
{
    FlushDebugVertices(view, projection, GL_LINES, lines, 1.0f);
    lines.clear();
}

void Renderer::FlushDebugVertices(
    const Mat4& view, const Mat4& projection, GLenum primitive, const std::vector<DebugVertex>& vertices, float pointSize
)
{
    if (vertices.empty())
    {
        return;
    }

    debugShader.Use();
    debugShader.SetMat4("uView", view);
    debugShader.SetMat4("uProjection", projection);
    debugShader.SetFloat("uPointSize", pointSize);

    glBindVertexArray(debugVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vertices.size() * sizeof(DebugVertex)), vertices.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(primitive, 0, (GLsizei)vertices.size());
    glBindVertexArray(0);
}

void Renderer::FlushAllDebug(const Mat4& view, const Mat4& projection)
{
    FlushPoints(view, projection, 5.0f);
    FlushLines(view, projection);
}

void Renderer::DrawDebug(const World& world, const Mat4& view, const Mat4& projection, const DebugOptions& options)
{
    if (options.show_bvh || options.show_aabb)
    {
        const Vec3 aabbColor{ 0.08f, 0.09f, 0.10f };
        const AABBTree& tree = world.GetDynamicTree();
        tree.Traverse([&](const AABBTree::Node* node) -> void {
            if (options.show_bvh == false && node->IsLeaf() == false)
            {
                return;
            }

            DrawAABB(node->aabb, aabbColor);
        });
    }

    if (options.show_contact_point || options.show_contact_normal)
    {
        const Vec3 pointColor{ 1.0f, 0.18f, 0.08f };
        const Vec3 normalColor{ 0.05f, 0.25f, 1.0f };
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
                    DrawPoint(p1, pointColor);
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

                    DrawLine(p1, p2, normalColor);
                    DrawLine(p2, arrowA, normalColor);
                    DrawLine(p2, arrowB, normalColor);
                }
            }
        }
    }

    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(1.0f);

    FlushAllDebug(view, projection);

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
        for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
        {
            DrawBody(*body, Vec3{ 1.0f, 1.0f, 1.0f }, shadowShader);
        }
        FlushSpheres(shadowShader);
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
        for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
        {
            const Vec3 color = body->IsStatic() ? Vec3{ 0.92f, 0.92f, 0.92f } : Vec3{ 1.0f, 1.0f, 1.0f };
            DrawBody(*body, color, surfaceShader);
        }
        FlushSpheres(surfaceShader);
    }

    if (options.draw_wireframe)
    {
        GLint previousDepthFunc = GL_LESS;
        glGetIntegerv(GL_DEPTH_FUNC, &previousDepthFunc);
        glDepthFunc(GL_LEQUAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_CULL_FACE);
        for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
        {
            DrawBody(*body, Vec3{ 0.03f, 0.04f, 0.05f }, surfaceShader);
        }
        FlushSpheres(surfaceShader);
        glEnable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthFunc(previousDepthFunc);
    }

    DrawDebug(world, view, projection, options);
}

} // namespace muli3
