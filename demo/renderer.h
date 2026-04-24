#pragma once

#include "camera.h"
#include "mesh.h"
#include "options.h"
#include "shader.h"

namespace muli3
{

struct Vertex
{
    Vec3 point;
    Vec4 color;
};

class Renderer : NonCopyable
{
public:
    static constexpr inline Vec4 default_white{ 1.0f, 1.0f, 1.0f, 0.8f };
    static constexpr inline Vec4 default_black{ 0.0f, 0.0f, 0.0f, 0.9f };

    ~Renderer();

    bool Initialize();
    void Shutdown();
    void Render(const World& world, const Camera& camera, float aspectRatio, const DebugOptions& options);

    void SetPointSize(float size);
    void SetLineWidth(float lineWidth) const;
    void SetProjectionMatrix(const Mat4& projection);
    void SetViewMatrix(const Mat4& view);

    void DrawPoint(const Vertex& v);
    void DrawPoint(const Vec3& point, const Vec4& color = default_black);
    void DrawLine(const Vertex& v1, const Vertex& v2);
    void DrawLine(const Vec3& p1, const Vec3& p2, const Vec4& color = default_black);
    void DrawAABB(const AABB& aabb);
    void DrawShape(const Shape* shape, const Transform& transform, const Vec4& color = default_white);

    void FlushAll();
    void FlushPoints();
    void FlushLines();

private:
    struct SphereInstance
    {
        Vec4 model0;
        Vec4 model1;
        Vec4 model2;
        Vec4 model3;
        Vec4 color;
    };

    bool CreateShadowResources();
    bool CreateBatchResources();
    bool CreateShapeResources();
    void DestroyShadowResources();
    void DestroyBatchResources();
    void DestroyShapeResources();
    void DrawBody(const RigidBody& body, const Vec3& color, const Shader& shader);
    void QueueShape(const Shape* shape, const Transform& transform, const Vec4& color, const Shader& shader);
    void DrawAABB(const AABB& aabb, const Vec4& color);
    void FlushSpheres(const Shader& shader);
    void FlushPrimitive(GLenum primitive, const std::vector<Vertex>& vertices, int32 vertexCount);
    void DrawOverlay(const World& world, const DebugOptions& options);
    void EnsurePrimitiveCapacity(std::vector<Vertex>& vertices, int32 requiredCount);

    Shader surfaceShader;
    Shader shadowShader;
    Shader batchShader;
    Mesh sphereMesh;
    bool initialized = false;
    GLuint shadowFramebuffer = 0;
    GLuint shadowDepthTexture = 0;
    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint sphereInstanceVBO = 0;
    std::vector<SphereInstance> sphereInstances;
    std::vector<Vertex> points;
    int32 pointCount = 0;
    std::vector<Vertex> lines;
    int32 lineCount = 0;
    Mat4 viewMatrix{ identity };
    Mat4 projectionMatrix{ identity };
    float pointSize = 5.0f;
};

inline void Renderer::SetPointSize(float size)
{
    pointSize = size;
}

inline void Renderer::SetLineWidth(float lineWidth) const
{
    glLineWidth(lineWidth);
}

inline void Renderer::SetProjectionMatrix(const Mat4& projection)
{
    projectionMatrix = projection;
}

inline void Renderer::SetViewMatrix(const Mat4& view)
{
    viewMatrix = view;
}

inline void Renderer::DrawPoint(const Vertex& v)
{
    EnsurePrimitiveCapacity(points, pointCount + 1);
    points[pointCount] = v;
    ++pointCount;
}

inline void Renderer::DrawPoint(const Vec3& point, const Vec4& color)
{
    EnsurePrimitiveCapacity(points, pointCount + 1);
    points[pointCount] = Vertex{ point, color };
    ++pointCount;
}

inline void Renderer::DrawLine(const Vertex& v1, const Vertex& v2)
{
    EnsurePrimitiveCapacity(lines, lineCount + 2);
    lines[lineCount] = v1;
    ++lineCount;
    lines[lineCount] = v2;
    ++lineCount;
}

inline void Renderer::DrawLine(const Vec3& p1, const Vec3& p2, const Vec4& color)
{
    EnsurePrimitiveCapacity(lines, lineCount + 2);
    lines[lineCount] = Vertex{ p1, color };
    ++lineCount;
    lines[lineCount] = Vertex{ p2, color };
    ++lineCount;
}

inline void Renderer::DrawAABB(const AABB& aabb)
{
    DrawAABB(aabb, default_black);
}

inline void Renderer::FlushAll()
{
    if (sphereInstances.empty() == false) FlushSpheres(surfaceShader);
    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();
}

} // namespace muli3
