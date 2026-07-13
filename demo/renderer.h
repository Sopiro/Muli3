#pragma once

#include "camera.h"
#include "mesh.h"
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

    void BeginFrame(const Camera& camera, float aspectRatio);
    void BeginShadowPass();
    void EndShadowPass();
    void BeginShapePass();
    void EndFrame();

    float GetPointSize() const;
    float GetLineWidth() const;
    void SetPointSize(float size);
    void SetLineWidth(float lineWidth);
    Vec4 GetColor(int32 colorIndex) const;

    void SetProjectionMatrix(const Mat4& projection);
    void SetViewMatrix(const Mat4& view);

    void DrawPoint(const Vertex& v);
    void DrawPoint(const Vec3& point, const Vec4& color = default_black);
    void DrawLine(const Vertex& v1, const Vertex& v2);
    void DrawLine(const Vec3& p1, const Vec3& p2, const Vec4& color = default_black);
    void DrawAABB(const AABB& aabb);
    void DrawShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe = false);
    void DrawShape(const Shape* shape, const Transform& transform);
    void DrawShape(const Shape* shape, const Transform& transform, const DrawMode& mode);
    void FlushShapes();
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

    struct InstancedMeshBatch
    {
        Mesh mesh;
        std::vector<ShapeInstance> instances[2];
    };

    struct ShapeMeshRange
    {
        GLuint firstIndex = 0;
        GLuint indexCount = 0;
        GLuint firstOutlineIndex = 0;
        GLuint outlineIndexCount = 0;
        GLint baseVertex = 0;
    };

    struct ShapeMeshKeyCache
    {
        size_t key = 0;
        uint64 frame = 0;
    };

    struct DrawElementsIndirectCommand
    {
        GLuint count = 0;
        GLuint instanceCount = 0;
        GLuint firstIndex = 0;
        GLint baseVertex = 0;
        GLuint baseInstance = 0;
    };

    bool CreateShadowResources();
    bool CreatePrimitiveResources();
    bool CreateShapeResources();
    void SetShapeInstanceAttributes();
    void UploadShapeInstances(const ShapeInstance* instances, size_t count);
    void DestroyShadowResources();
    void DestroyPrimitiveResources();
    void DestroyShapeResources();

    void QueueShape(const Shape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader);
    void DrawAABB(const AABB& aabb, const Vec4& color);
    void FlushQueuedShapes(const Shader& shader, bool wireframe);
    void FlushInstancedMesh(InstancedMeshBatch& batch, int32 pass, bool wireframe);
    void FlushShapeMeshes(int32 pass, bool wireframe);
    void DrawHeightField(
        const HeightFieldShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
    );
    void DrawMeshShape(
        const MeshShape* shape, const Transform& transform, const Vec4& color, bool wireframe, const Shader& shader
    );
    size_t GetShapeMeshKey(const Shape* shape);
    size_t GetConvexMeshKey(const ConvexShape* shape) const;
    size_t GetPolygonMeshKey(const PolygonShape* shape) const;
    const ShapeMeshRange& GetConvexMesh(const ConvexShape* shape, size_t key);
    const ShapeMeshRange& GetPolygonMesh(const PolygonShape* shape, size_t key);
    const ShapeMeshRange& StoreShapeMesh(
        size_t key,
        std::span<const MeshVertex> vertices,
        std::span<const uint32> indices,
        std::span<const uint32> outlineIndices
    );
    Mesh& GetHeightFieldMesh(const HeightFieldShape* shape);
    Mesh& GetMeshShapeMesh(const MeshShape* shape);
    void FlushPrimitive(GLenum primitive, const std::vector<Vertex>& vertices, int32 vertexCount);
    void EnsurePrimitiveCapacity(std::vector<Vertex>& vertices, int32 requiredCount);

    bool initialized = false;
    Shader shapeShader, shadowShader, primitiveShader;
    InstancedMeshBatch sphereBatch, capsuleTopBatch, capsuleBottomBatch, capsuleMidBatch, boxBatch, triangleBatch;

    GLuint shadowFramebuffer;
    GLuint shadowDepthTexture;

    GLuint primVAO, primVBO, shapeInstanceVBO;
    GLuint shapeMeshVAO = 0, shapeMeshOutlineVAO = 0, shapeMeshVBO = 0, shapeMeshEBO = 0, shapeMeshOutlineEBO = 0,
           shapeMeshIndirectVBO = 0;
    int32 primitiveCapacity = 0;
    size_t queuedShapeCount[2]{};
    std::unordered_map<size_t, ShapeMeshRange> shapeMeshes;
    std::unordered_map<const Shape*, ShapeMeshKeyCache> shapeMeshKeyCache;
    std::unordered_map<size_t, std::vector<ShapeInstance>> shapeMeshInstances[2];
    std::vector<size_t> activeShapeMeshKeys[2];
    size_t shapeMeshInstanceCount[2]{};
    std::vector<MeshVertex> shapeMeshVertices;
    std::vector<uint32> shapeMeshIndices;
    std::vector<uint32> shapeMeshOutlineIndices;
    size_t shapeMeshVertexCapacity = 0;
    size_t shapeMeshIndexCapacity = 0;
    size_t shapeMeshOutlineIndexCapacity = 0;
    std::vector<ShapeInstance> shapeMeshInstanceBuffer;
    std::vector<DrawElementsIndirectCommand> shapeMeshCommands;
    std::unordered_map<const HeightFieldShape*, Mesh> heightFieldMeshes;
    std::unordered_map<const MeshShape*, Mesh> meshShapeMeshes;

    int32 pointCount = 0;
    std::vector<Vertex> points;
    int32 lineCount = 0;
    std::vector<Vertex> lines;

    Mat4 viewMatrix{ identity };
    Mat4 projectionMatrix{ identity };

    Mat4 lightViewProjectionMatrix{ identity };
    Vec3 lightDirection{ 0.0f, -1.0f, 0.0f };
    Shader* currentShapeShader = nullptr;
    GLint viewport[4]{};
    uint64 frame = 0;

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
