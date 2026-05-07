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

    struct DrawMode
    {
        int32 colorIndex = -1;
        bool rounded = false;
        bool outline = false;
        bool fill = true;
    };

    ~Renderer();

    bool Initialize();
    void Shutdown();

    void Render(const World& world, const Camera& camera, float aspectRatio, const DebugOptions& options);

    float GetPointSize() const;
    float GetLineWidth() const;
    void SetPointSize(float size);
    void SetLineWidth(float lineWidth);

    void SetProjectionMatrix(const Mat4& projection);
    void SetViewMatrix(const Mat4& view);

    void DrawPoint(const Vertex& v);
    void DrawPoint(const Vec3& point, const Vec4& color = default_black);
    void DrawLine(const Vertex& v1, const Vertex& v2);
    void DrawLine(const Vec3& p1, const Vec3& p2, const Vec4& color = default_black);
    void DrawAABB(const AABB& aabb);
    void DrawShape(const Shape* shape, const Transform& transform);
    void DrawShape(const Shape* shape, const Transform& transform, const DrawMode& mode);
    void ClearMeshCache();

    void FlushAll();
    void FlushPoints();
    void FlushLines();

private:
    struct ShapeInstance
    {
        Mat4 model;
        Vec4 color;
    };

    bool CreateShadowResources();
    bool CreatePrimitiveResources();
    bool CreateShapeResources();
    void SetShapeInstanceAttributes();
    void DestroyShadowResources();
    void DestroyPrimitiveResources();
    void DestroyShapeResources();

    void DrawBody(const RigidBody& body, const Vec4& color, bool wireframe, const Shader& shader);
    void QueueShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader);
    void DrawAABB(const AABB& aabb, const Vec4& color);
    void FlushQueuedShapes(const Shader& shader, bool wireframe);
    void FlushSpheres(const Shader& shader, bool wireframe);
    void FlushCapsules(const Shader& shader, bool wireframe);
    void FlushBoxes(const Shader& shader, bool wireframe);
    void DrawConvex(
        const ConvexShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
    );
    Mesh& GetConvexMesh(const ConvexShape* shape);
    void FlushPrimitive(GLenum primitive, const std::vector<Vertex>& vertices, int32 vertexCount);
    void EnsurePrimitiveCapacity(std::vector<Vertex>& vertices, int32 requiredCount);

    bool initialized = false;
    Shader shapeShader, shadowShader, primitiveShader;
    Mesh sphereMesh, capsuleTopMesh, capsuleBottomMesh, capsuleMidMesh, boxMesh;

    GLuint shadowFramebuffer;
    GLuint shadowDepthTexture;

    GLuint primVAO, primVBO, shapeInstanceVBO;
    std::vector<ShapeInstance> sphereInstances[2];
    std::vector<ShapeInstance> capsuleTopInstances[2];
    std::vector<ShapeInstance> capsuleBottomInstances[2];
    std::vector<ShapeInstance> capsuleMidInstances[2];
    std::vector<ShapeInstance> boxInstances[2];
    std::unordered_map<const ConvexShape*, Mesh> convexMeshes;

    int32 pointCount = 0;
    std::vector<Vertex> points;
    int32 lineCount = 0;
    std::vector<Vertex> lines;

    Mat4 viewMatrix{ identity };
    Mat4 projectionMatrix{ identity };

    float pointSize = 5.0f;
    float lineWidth = 1.0f;
};

inline float Renderer::GetPointSize() const
{
    return pointSize;
}

inline float Renderer::GetLineWidth() const
{
    return lineWidth;
}

inline void Renderer::SetPointSize(float size)
{
    pointSize = size;
}

inline void Renderer::SetLineWidth(float width)
{
    lineWidth = width;
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

inline void Renderer::DrawShape(const Shape* shape, const Transform& transform)
{
    DrawShape(shape, transform, DrawMode{});
}

inline void Renderer::FlushAll()
{
    FlushQueuedShapes(shapeShader, false);
    FlushQueuedShapes(shapeShader, true);
    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();
}

} // namespace muli3
