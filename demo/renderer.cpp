#include "renderer.h"
#include "muli3/frame.h"

namespace muli3
{

constexpr int g_shadowMapSize = 2048;
constexpr size_t g_maxShapeBatchCount = 4096;
constexpr int32 g_maxVertexCount = 1024 * 3;
constexpr int32 g_colorCount = 10;
constexpr int32 g_fillPass = 0;
constexpr int32 g_outlinePass = 1;

Vec4 g_colors[g_colorCount];
bool g_colorsInitialized = false;

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
    vWorldPosition = worldPosition.xyz;
    vWorldNormal = normalize(mat3(model) * aNormal);
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

    float bias = max(0.00015, 0.0012 * (1.0 - dot(normal, lightDir)));
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
    vec2 tiledUv = uv * vec2(8.0, 6.0);
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

float HueToRGB(float p, float q, float t)
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

Vec3 HSLToRGB(const Vec3& hsl)
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

static bool IsSameBodyPair(const Joint* joint, const RigidBody* bodyA, const RigidBody* bodyB)
{
    return (joint->GetBodyA() == bodyA && joint->GetBodyB() == bodyB) ||
           (joint->GetBodyA() == bodyB && joint->GetBodyB() == bodyA);
}

static Vec3 GetAngularJointAnchor(const World& world, const Joint* joint)
{
    const RigidBody* bodyA = joint->GetBodyA();
    const RigidBody* bodyB = joint->GetBodyB();

    for (const Joint* other = world.GetJoints(); other; other = other->GetNext())
    {
        if (other == joint || other->GetType() != Joint::ball_socket_joint)
        {
            continue;
        }

        if (IsSameBodyPair(other, bodyA, bodyB) == false)
        {
            continue;
        }

        const BallSocketJoint* ballSocketJoint = (const BallSocketJoint*)other;
        Vec3 anchorA = Mul(bodyA->GetTransform(), ballSocketJoint->GetLocalAnchorA());
        Vec3 anchorB = Mul(bodyB->GetTransform(), ballSocketJoint->GetLocalAnchorB());
        return (anchorA + anchorB) * 0.5f;
    }

    return (bodyA->GetPosition() + bodyB->GetPosition()) * 0.5f;
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
        Vec3 rgb = HSLToRGB({ i * stride / 360.0f, 1.0f, 0.8f });
        g_colors[i] = Vec4{ rgb.x, rgb.y, rgb.z, 0.85f };
    }

    g_colorsInitialized = true;
}

Vec4 GetBodyColor(const RigidBody& body, const DebugOptions& options)
{
    if (body.IsStatic())
    {
        return Renderer::default_white;
    }

    if (options.colorize_island == false)
    {
        return body.IsSleeping() ? Vec4{ 0.9f, 0.9f, 0.9f, Renderer::default_white.w } : Renderer::default_white;
    }

    if (body.IsSleeping())
    {
        return Renderer::default_white;
    }

    int32 colorIndex = body.GetIslandID() - 1;
    if (colorIndex < 0)
    {
        return Renderer::default_white;
    }

    return g_colors[colorIndex % g_colorCount];
}

Vec4 GetModeColor(const Renderer::DrawMode& mode)
{
    if (mode.colorIndex < 0)
    {
        return Renderer::default_white;
    }

    return g_colors[mode.colorIndex % g_colorCount];
}

void DrawBasis(Renderer& renderer, const Vec3& origin, const Quat& rotation, float scale, float alpha)
{
    renderer.DrawLine(origin, origin + rotation.Rotate(x_axis) * scale, Vec4{ 0.95f, 0.2f, 0.2f, alpha });
    renderer.DrawLine(origin, origin + rotation.Rotate(y_axis) * scale, Vec4{ 0.2f, 0.85f, 0.2f, alpha });
    renderer.DrawLine(origin, origin + rotation.Rotate(z_axis) * scale, Vec4{ 0.2f, 0.45f, 1.0f, alpha });
}

void DrawAxis(Renderer& renderer, const Vec3& origin, const Vec3& axis, float halfLength, const Vec4& color)
{
    renderer.DrawLine(origin - axis * halfLength, origin + axis * halfLength, color);
}

void DrawConeLimit(Renderer& renderer, const Vec3& origin, const Vec3& axis, float angle, float length, const Vec4& color)
{
    if (angle <= 0.0f)
    {
        return;
    }

    Frame frame = Frame::FromZ(axis);
    float radius = std::tan(angle) * length;
    Vec3 tip = origin + axis * length;
    constexpr int32 segmentCount = 24;

    Vec3 firstPoint = Vec3::zero;
    Vec3 prevPoint = Vec3::zero;
    for (int32 i = 0; i <= segmentCount; ++i)
    {
        float t = two_pi * (float)i / (float)segmentCount;
        Vec3 point = tip + frame.x * std::cos(t) * radius + frame.y * std::sin(t) * radius;

        if (i == 0)
        {
            firstPoint = point;
        }
        else
        {
            renderer.DrawLine(prevPoint, point, color);
        }

        if (i < segmentCount && (i % 6) == 0)
        {
            renderer.DrawLine(origin, point, color);
        }

        prevPoint = point;
    }

    renderer.DrawLine(prevPoint, firstPoint, color);
}

void DrawCircle(Renderer& renderer, const Vec3& origin, const Vec3& normal, float radius, const Vec4& color)
{
    Frame frame = Frame::FromZ(normal);
    constexpr int32 segmentCount = 40;

    Vec3 firstPoint = origin + frame.x * radius;
    Vec3 prevPoint = firstPoint;
    for (int32 i = 1; i <= segmentCount; ++i)
    {
        float t = two_pi * (float)i / (float)segmentCount;
        Vec3 point = origin + (frame.x * std::cos(t) + frame.y * std::sin(t)) * radius;
        renderer.DrawLine(prevPoint, point, color);
        prevPoint = point;
    }
}

void DrawTwistArc(
    Renderer& renderer,
    const Vec3& origin,
    const Vec3& axis,
    const Vec3& t1,
    const Vec3& t2,
    float radius,
    float minAngle,
    float maxAngle,
    float currentAngle,
    const Vec4& limitColor,
    const Vec4& currentColor
)
{
    constexpr int32 segmentCount = 32;
    float span = maxAngle - minAngle;

    if (span > 0.0f)
    {
        Vec3 prevPoint = origin + (t1 * std::cos(minAngle) + t2 * std::sin(minAngle)) * radius;
        for (int32 i = 1; i <= segmentCount; ++i)
        {
            float angle = minAngle + span * (float)i / (float)segmentCount;
            Vec3 point = origin + (t1 * std::cos(angle) + t2 * std::sin(angle)) * radius;
            renderer.DrawLine(prevPoint, point, limitColor);
            prevPoint = point;
        }
    }

    Vec3 minDir = t1 * std::cos(minAngle) + t2 * std::sin(minAngle);
    Vec3 maxDir = t1 * std::cos(maxAngle) + t2 * std::sin(maxAngle);
    Vec3 currentDir = t1 * std::cos(currentAngle) + t2 * std::sin(currentAngle);

    renderer.DrawLine(origin, origin + minDir * radius, limitColor);
    renderer.DrawLine(origin, origin + maxDir * radius, limitColor);
    renderer.DrawLine(origin - axis * 0.4f, origin + axis * 0.4f, Vec4{ 0.12f, 0.12f, 0.12f, 0.65f });
    renderer.DrawLine(origin, origin + currentDir * radius, currentColor);
}

void DrawSwingArc(
    Renderer& renderer,
    const Vec3& origin,
    const Vec3& axis,
    const Vec3& tangent,
    float radius,
    float minAngle,
    float maxAngle,
    float currentAngle,
    const Vec4& limitColor,
    const Vec4& currentColor
)
{
    constexpr int32 segmentCount = 24;
    float span = maxAngle - minAngle;

    if (span > 0.0f)
    {
        Vec3 prevPoint = origin + (axis * std::cos(minAngle) + tangent * std::sin(minAngle)) * radius;
        for (int32 i = 1; i <= segmentCount; ++i)
        {
            float angle = minAngle + span * (float)i / (float)segmentCount;
            Vec3 point = origin + (axis * std::cos(angle) + tangent * std::sin(angle)) * radius;
            renderer.DrawLine(prevPoint, point, limitColor);
            prevPoint = point;
        }
    }

    Vec3 minDir = axis * std::cos(minAngle) + tangent * std::sin(minAngle);
    Vec3 maxDir = axis * std::cos(maxAngle) + tangent * std::sin(maxAngle);
    Vec3 currentDir = axis * std::cos(currentAngle) + tangent * std::sin(currentAngle);

    renderer.DrawLine(origin, origin + minDir * radius, limitColor);
    renderer.DrawLine(origin, origin + maxDir * radius, limitColor);
    renderer.DrawLine(origin, origin + currentDir * radius, currentColor);
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * g_maxVertexCount, nullptr, GL_DYNAMIC_DRAW);
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

bool Renderer::CreateShapeResources()
{
    glGenBuffers(1, &shapeInstanceVBO);

    glBindVertexArray(sphereMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferData(GL_ARRAY_BUFFER, g_maxShapeBatchCount * sizeof(ShapeInstance), nullptr, GL_DYNAMIC_DRAW);

    SetShapeInstanceAttributes();

    glBindVertexArray(capsuleTopMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(capsuleBottomMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(capsuleMidMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(boxMesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return shapeInstanceVBO != 0;
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
    if (primVAO != 0)
    {
        glDeleteVertexArrays(1, &primVAO);
        primVAO = 0;
    }
}

void Renderer::DestroyShapeResources()
{
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

    BuildSphereMesh(&vertices, &indices, 24, 12);
    sphereMesh.Upload(vertices, indices, GL_TRIANGLES);

    BuildCapsuleTopMesh(&vertices, &indices, 24, 12);
    capsuleTopMesh.Upload(vertices, indices, GL_TRIANGLES);

    BuildCapsuleBottomMesh(&vertices, &indices, 24, 12);
    capsuleBottomMesh.Upload(vertices, indices, GL_TRIANGLES);

    BuildCapsuleMidMesh(&vertices, &indices, 24);
    capsuleMidMesh.Upload(vertices, indices, GL_TRIANGLES);

    BuildBoxMesh(&vertices, &indices);
    boxMesh.Upload(vertices, indices, GL_TRIANGLES);
    if (!CreateShapeResources())
    {
        return false;
    }

    sphereInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    sphereInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
    capsuleTopInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    capsuleTopInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
    capsuleBottomInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    capsuleBottomInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
    capsuleMidInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    capsuleMidInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
    boxInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    boxInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
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
    sphereMesh.Destroy();
    capsuleTopMesh.Destroy();
    capsuleBottomMesh.Destroy();
    capsuleMidMesh.Destroy();
    boxMesh.Destroy();
    shapeShader.Destroy();
    shadowShader.Destroy();
    DestroyShadowResources();
    primitiveShader.Destroy();
    DestroyPrimitiveResources();
    initialized = false;
}

void Renderer::ClearMeshCache()
{
    for (auto& [shape, mesh] : convexMeshes)
    {
        MuliNotUsed(shape);
        mesh.Destroy();
    }

    convexMeshes.clear();
}

void Renderer::DrawBody(const RigidBody& body, const Vec4& color, bool wireframe, const Shader& shader)
{
    for (const Collider* collider = body.GetColliderList(); collider; collider = collider->GetNext())
    {
        QueueShape(collider->GetShape(), body.GetTransform(), color, wireframe, shader);
    }
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

void Renderer::QueueShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader)
{
    if (!shape)
    {
        return;
    }

    int32 pass = wireframe ? g_outlinePass : g_fillPass;

    if (shape->GetType() == Shape::sphere)
    {
        const Sphere* sphere = (const Sphere*)shape;
        Transform renderTransform = transform;
        renderTransform.p = Mul(transform, sphere->GetCenter());
        renderTransform.s = renderTransform.s * Vec3{ sphere->GetRadius(), sphere->GetRadius(), sphere->GetRadius() };
        sphereInstances[pass].emplace_back(Mat4(renderTransform), color);

        if (sphereInstances[pass].size() == g_maxShapeBatchCount)
        {
            FlushSpheres(shader, wireframe);
        }
    }
    else if (shape->GetType() == Shape::capsule)
    {
        const Capsule* capsule = (const Capsule*)shape;
        Vec3 a = Mul(transform, capsule->GetVertexA());
        Vec3 b = Mul(transform, capsule->GetVertexB());
        Vec3 axis = b - a;
        float height = axis.Normalize();
        float radius = capsule->GetRadius() * Max(Abs(transform.s.x), Max(Abs(transform.s.y), Abs(transform.s.z)));
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

        capsuleTopInstances[pass].emplace_back(topModel, color);
        capsuleBottomInstances[pass].emplace_back(bottomModel, color);
        capsuleMidInstances[pass].emplace_back(midModel, color);

        if (capsuleTopInstances[pass].size() == g_maxShapeBatchCount)
        {
            FlushCapsules(shader, wireframe);
        }
    }
    else if (shape->GetType() == Shape::box)
    {
        const Box* box = (const Box*)shape;
        Transform renderTransform = transform;
        renderTransform.p = Mul(transform, box->GetCenter());
        renderTransform.q = transform.q * box->GetRotation();
        renderTransform.s = renderTransform.s * box->GetHalfExtents();
        boxInstances[pass].emplace_back(Mat4(renderTransform), color);

        if (boxInstances[pass].size() == g_maxShapeBatchCount)
        {
            FlushBoxes(shader, wireframe);
        }
    }
    else if (shape->GetType() == Shape::convex)
    {
        DrawConvex((const ConvexShape*)shape, transform, color, wireframe, shader);
    }
}

Mesh& Renderer::GetConvexMesh(const ConvexShape* shape)
{
    auto it = convexMeshes.find(shape);
    if (it != convexMeshes.end())
    {
        return it->second;
    }

    std::vector<MeshVertex> vertices;
    std::vector<uint32> indices;
    BuildConvexMesh(&vertices, &indices, *shape);

    Mesh& mesh = convexMeshes[shape];
    mesh.Upload(vertices, indices, GL_TRIANGLES);

    glBindVertexArray(mesh.GetVAO());
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    SetShapeInstanceAttributes();
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return mesh;
}

void Renderer::DrawConvex(
    const ConvexShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
)
{
    ShapeInstance instance{ Mat4(transform), color };
    Mesh& mesh = GetConvexMesh(shape);

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
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(ShapeInstance), &instance);
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
    FlushSpheres(shader, wireframe);
    FlushCapsules(shader, wireframe);
    FlushBoxes(shader, wireframe);
}

void Renderer::FlushSpheres(const Shader& shader, bool wireframe)
{
    std::vector<ShapeInstance>& instances = sphereInstances[wireframe ? g_outlinePass : g_fillPass];
    if (instances.empty())
    {
        return;
    }

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
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(instances.size() * sizeof(ShapeInstance)), instances.data());
    sphereMesh.DrawInstanced((GLsizei)instances.size());
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

    instances.clear();
}

void Renderer::FlushCapsules(const Shader& shader, bool wireframe)
{
    int32 pass = wireframe ? g_outlinePass : g_fillPass;
    if (capsuleTopInstances[pass].empty())
    {
        return;
    }

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
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(
        GL_ARRAY_BUFFER, 0, (GLsizeiptr)(capsuleTopInstances[pass].size() * sizeof(ShapeInstance)),
        capsuleTopInstances[pass].data()
    );
    capsuleTopMesh.DrawInstanced((GLsizei)capsuleTopInstances[pass].size());

    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(
        GL_ARRAY_BUFFER, 0, (GLsizeiptr)(capsuleBottomInstances[pass].size() * sizeof(ShapeInstance)),
        capsuleBottomInstances[pass].data()
    );
    capsuleBottomMesh.DrawInstanced((GLsizei)capsuleBottomInstances[pass].size());

    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(
        GL_ARRAY_BUFFER, 0, (GLsizeiptr)(capsuleMidInstances[pass].size() * sizeof(ShapeInstance)),
        capsuleMidInstances[pass].data()
    );
    capsuleMidMesh.DrawInstanced((GLsizei)capsuleMidInstances[pass].size());

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

    capsuleTopInstances[pass].clear();
    capsuleBottomInstances[pass].clear();
    capsuleMidInstances[pass].clear();
}

void Renderer::FlushBoxes(const Shader& shader, bool wireframe)
{
    std::vector<ShapeInstance>& instances = boxInstances[wireframe ? g_outlinePass : g_fillPass];
    if (instances.empty())
    {
        return;
    }

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
    glBindBuffer(GL_ARRAY_BUFFER, shapeInstanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(instances.size() * sizeof(ShapeInstance)), instances.data());
    boxMesh.DrawInstanced((GLsizei)instances.size());
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

    instances.clear();
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
    if (requiredCount <= (int32)vertices.size())
    {
        return;
    }

    int32 newCapacity = Max<int32>((int32)vertices.size() * 2, requiredCount);
    vertices.resize(newCapacity);

    glBindBuffer(GL_ARRAY_BUFFER, primVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * newCapacity, nullptr, GL_DYNAMIC_DRAW);
}

void Renderer::Render(const World& world, const Camera& camera, float aspectRatio, const DebugOptions& options)
{
    const Mat4 view = camera.GetViewMatrix();
    const Mat4 projection = camera.GetProjectionMatrix(aspectRatio);
    const Vec3 lightDirection = Normalize(Vec3{ 0.45f, -1.0f, -0.35f });
    const Mat4 lightViewProjection = ComputeLightViewProjection(lightDirection);
    SetViewMatrix(view);
    SetProjectionMatrix(projection);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glViewport(0, 0, g_shadowMapSize, g_shadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_BACK);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.5f, 1.0f);

    shadowShader.Use();
    shadowShader.SetMat4("uLightViewProjection", lightViewProjection);

    if (options.draw_body)
    {
        for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
        {
            DrawBody(*body, default_white, false, shadowShader);
        }
        FlushQueuedShapes(shadowShader, false);
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glCullFace(GL_BACK);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTexture);

    shapeShader.Use();
    shapeShader.SetMat4("uView", view);
    shapeShader.SetMat4("uProjection", projection);
    shapeShader.SetMat4("uLightViewProjection", lightViewProjection);
    shapeShader.SetVec3("uLightDirection", lightDirection);

    if (options.draw_body)
    {
        if (options.draw_outlined == false)
        {
            for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
            {
                DrawBody(*body, GetBodyColor(*body, options), false, shapeShader);
            }
            FlushQueuedShapes(shapeShader, false);
        }
    }

    if (options.draw_body && options.draw_outlined)
    {
        for (RigidBody* body = world.GetBodyList(); body; body = body->GetNext())
        {
            DrawBody(*body, default_black, true, shapeShader);
        }
        FlushQueuedShapes(shapeShader, true);
    }

    if (options.draw_joint)
    {
        for (const Joint* joint = world.GetJoints(); joint; joint = joint->GetNext())
        {
            switch (joint->GetType())
            {
            case Joint::grab_joint:
            {
                const RigidBody* body = joint->GetBodyA();
                const GrabJoint* grabJoint = (const GrabJoint*)joint;
                Vec3 anchor = Mul(body->GetTransform(), grabJoint->GetLocalAnchor());
                DrawPoint(anchor);
                DrawPoint(grabJoint->GetTarget());
                DrawLine(anchor, grabJoint->GetTarget());
            }
            break;
            case Joint::fixed_rotation_joint:
            {
                const RigidBody* body = joint->GetBodyA();
                const FixedRotationJoint* fixedRotationJoint = (const FixedRotationJoint*)joint;
                Vec3 position = body->GetPosition();
                DrawPoint(position);
                DrawBasis(*this, position, fixedRotationJoint->GetTargetOrientation(), 0.55f, 0.55f);
            }
            break;
            case Joint::cone_swing_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const ConeSwingJoint* coneSwingJoint = (const ConeSwingJoint*)joint;

                Vec3 positionA = bodyA->GetPosition();
                Vec3 positionB = bodyB->GetPosition();
                Vec3 axisA = bodyA->GetRotation().Rotate(coneSwingJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(coneSwingJoint->GetLocalAxisB());
                DrawAxis(*this, positionB, axisB, 0.7f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawConeLimit(*this, positionA, axisA, coneSwingJoint->GetJointMaxAngle(), 0.8f, Vec4{ 0.9f, 0.2f, 0.2f, 0.5f });
            }
            break;
            case Joint::revolute_angle_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const RevoluteAngleJoint* revoluteAngleJoint = (const RevoluteAngleJoint*)joint;
                Vec3 anchor = GetAngularJointAnchor(world, joint);
                Vec3 axisA = bodyA->GetRotation().Rotate(revoluteAngleJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(revoluteAngleJoint->GetLocalAxisB());
                Vec3 t1, t2;
                CoordinateSystem(axisA, &t1, &t2);

                DrawCircle(*this, anchor, axisA, 0.45f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(*this, anchor, axisA, 0.55f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(*this, anchor, axisB, 0.45f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    *this, anchor, axisA, t1, t2, 0.45f, revoluteAngleJoint->GetJointMinAngle(),
                    revoluteAngleJoint->GetJointMaxAngle(), revoluteAngleJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::revolute_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const RevoluteJoint* revoluteJoint = (const RevoluteJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), revoluteJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), revoluteJoint->GetLocalAnchorB());
                Vec3 anchor = (anchorA + anchorB) * 0.5f;
                Vec3 axisA = bodyA->GetRotation().Rotate(revoluteJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(revoluteJoint->GetLocalAxisB());
                Vec3 refAxisA = bodyA->GetRotation().Rotate(revoluteJoint->GetLocalNormalAxisA());
                Vec3 binormalA = Cross(axisA, refAxisA);
                binormalA.Normalize();

                DrawPoint(anchorA);
                DrawPoint(anchorB);
                DrawLine(anchorA, bodyA->GetPosition());
                DrawLine(anchorB, bodyB->GetPosition());
                DrawLine(anchorA, anchorB, Vec4{ 0.12f, 0.12f, 0.12f, 0.35f });

                DrawCircle(*this, anchor, axisA, 0.45f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(*this, anchor, axisA, 0.55f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(*this, anchor, axisB, 0.45f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    *this, anchor, axisA, refAxisA, binormalA, 0.45f, revoluteJoint->GetJointMinAngle(),
                    revoluteJoint->GetJointMaxAngle(), revoluteJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::twist_angle_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const TwistAngleJoint* twistAngleJoint = (const TwistAngleJoint*)joint;
                Vec3 anchor = GetAngularJointAnchor(world, joint);
                Vec3 axisA = bodyA->GetRotation().Rotate(twistAngleJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(twistAngleJoint->GetLocalAxisB());
                Vec3 t1, t2;
                CoordinateSystem(axisA, &t1, &t2);

                DrawCircle(*this, anchor, axisA, 0.4f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(*this, anchor, axisA, 0.5f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(*this, anchor, axisB, 0.4f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    *this, anchor, axisA, t1, t2, 0.4f, twistAngleJoint->GetJointMinAngle(),
                    twistAngleJoint->GetJointMaxAngle(), twistAngleJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::ball_socket_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const BallSocketJoint* ballSocketJoint = (const BallSocketJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), ballSocketJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), ballSocketJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);
                DrawLine(anchorA, bodyA->GetPosition());
                DrawLine(anchorB, bodyB->GetPosition());
            }
            break;
            case Joint::distance_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const DistanceJoint* distanceJoint = (const DistanceJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), distanceJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), distanceJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);

                Vec3 d = anchorB - anchorA;
                if (d.Normalize() > 0.0f)
                {
                    DrawPoint(anchorA + d * distanceJoint->GetJointMinLength());
                    DrawPoint(anchorA + d * distanceJoint->GetJointMaxLength());
                }

                DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::weld_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const WeldJoint* weldJoint = (const WeldJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), weldJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), weldJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);
                DrawLine(anchorA, bodyA->GetPosition());
                DrawLine(anchorB, bodyB->GetPosition());
            }
            break;
            case Joint::line_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const LineJoint* lineJoint = (const LineJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), lineJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), lineJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);
                DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::prismatic_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const PrismaticJoint* prismaticJoint = (const PrismaticJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), prismaticJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), prismaticJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);
                DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::pulley_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const PulleyJoint* pulleyJoint = (const PulleyJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), pulleyJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), pulleyJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(pulleyJoint->GetGroundAnchorA());
                DrawPoint(anchorB);
                DrawPoint(pulleyJoint->GetGroundAnchorB());
                DrawLine(anchorA, pulleyJoint->GetGroundAnchorA());
                DrawLine(anchorB, pulleyJoint->GetGroundAnchorB());
            }
            break;
            case Joint::motor_joint:
            {
                const RigidBody* bodyA = joint->GetBodyA();
                const RigidBody* bodyB = joint->GetBodyB();
                const MotorJoint* motorJoint = (const MotorJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), motorJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), motorJoint->GetLocalAnchorB());
                DrawPoint(anchorA);
                DrawPoint(anchorB);
            }
            break;
            default:
                break;
            }
        }
    }

    if (options.show_bvh || options.show_aabb)
    {
        const AABBTree& tree = world.GetDynamicTree();
        tree.Traverse([&](const AABBTree::Node* node) -> void {
            if (options.show_bvh == false && node->IsLeaf() == false)
            {
                return;
            }

            DrawAABB(node->aabb, default_black);
        });
    }

    if (options.show_contact_point || options.show_contact_normal)
    {
        const Vec4 pointColor{ 1.0f, 0.18f, 0.08f, 0.9f };
        const Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 0.9f };
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
                    DrawLine(p1, p2, normalColor);
                }
            }
        }
    }

    const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glLineWidth(lineWidth);

    FlushAll();

    glLineWidth(1.0f);
    if (depthTestEnabled)
    {
        glEnable(GL_DEPTH_TEST);
    }
}

} // namespace muli3
