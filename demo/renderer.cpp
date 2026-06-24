#include "renderer.h"
#include "muli3/color.h"

namespace muli3
{

constexpr int g_shadowMapSize = 4096;
constexpr size_t g_maxShapeBatchCount = 4096;
constexpr int32 g_maxVertexCount = 1024 * 3;
constexpr int32 g_colorCount = 10;
constexpr int32 g_fillPass = 0;
constexpr int32 g_outlinePass = 1;
constexpr float g_shadowViewDistance = 50.0f;
constexpr float g_shadowBoundsPadding = 1.0f;
constexpr float g_shadowDepthPadding = 32.0f;

Vec4 g_colors[g_colorCount];
Vec4 g_colors2[constraint_color_count];
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
            visibility += texture(uShadowMap, vec3(projCoords.xy + vec2(x, y) * texelSize, projCoords.z - bias));
        }
    }

    return visibility / 9.0;
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * g_maxVertexCount, nullptr, GL_DYNAMIC_DRAW);
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

    glBindVertexArray(triangleMesh.GetVAO());
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
    primitiveCapacity = 0;
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

    BuildTriangleMesh(&vertices, &indices);
    triangleMesh.Upload(vertices, indices, GL_TRIANGLES);

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
    triangleInstances[g_fillPass].reserve(g_maxShapeBatchCount);
    triangleInstances[g_outlinePass].reserve(g_maxShapeBatchCount);
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
    triangleMesh.Destroy();
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
    for (auto& [shape, mesh] : heightFieldMeshes)
    {
        MuliNotUsed(shape);
        mesh.Destroy();
    }

    convexMeshes.clear();
    heightFieldMeshes.clear();
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
        const CapsuleShape* capsule = (const CapsuleShape*)shape;
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
        const BoxShape* box = (const BoxShape*)shape;
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

        triangleInstances[pass].emplace_back(model, color);

        if (triangleInstances[pass].size() == g_maxShapeBatchCount)
        {
            FlushTriangles(shader, wireframe);
        }
    }
    else if (shape->GetType() == Shape::height_field)
    {
        DrawHeightField((const HeightFieldShape*)shape, transform, color, wireframe, shader);
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
    FlushTriangles(shader, wireframe);
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

void Renderer::FlushTriangles(const Shader& shader, bool wireframe)
{
    std::vector<ShapeInstance>& instances = triangleInstances[wireframe ? g_outlinePass : g_fillPass];
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
    triangleMesh.DrawInstanced((GLsizei)instances.size());
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * primitiveCapacity, nullptr, GL_DYNAMIC_DRAW);
}

Vec4 Renderer::GetColor(int32 colorIndex) const
{
    return g_colors[colorIndex % g_colorCount];
}

void Renderer::BeginFrame(const Camera& camera, float aspectRatio)
{
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
