#pragma once

#include "common.h"

namespace muli3
{

class ConvexShape;
class HeightFieldShape;
class MeshShape;
class PolygonShape;

struct MeshVertex
{
    Vec3 position;
    Vec3 normal;
    Vec2 uv;
};

class Mesh : NonCopyable
{
public:
    Mesh() = default;
    ~Mesh();

    void Upload(const std::vector<MeshVertex>& vertices, const std::vector<uint32>& indices, GLenum primitiveType);
    void Upload(
        const std::vector<MeshVertex>& vertices,
        const std::vector<uint32>& triangleIndices,
        const std::vector<uint32>& lineIndices
    );
    void Destroy();
    void Draw() const;
    void DrawInstanced(GLsizei instanceCount, bool outline = false, GLuint baseInstance = 0) const;
    GLuint GetVAO() const;
    GLuint GetOutlineVAO() const;

private:
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    GLuint outlineVao = 0;
    GLuint outlineEbo = 0;
    GLsizei indexCount = 0;
    GLsizei outlineIndexCount = 0;
    GLenum primitive = GL_TRIANGLES;
};

void BuildSphereMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    int32 segments,
    int32 rings
);
void BuildCapsuleMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings, float height, float radius
);
void BuildCapsuleTopMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    int32 segments,
    int32 rings
);
void BuildCapsuleBottomMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    int32 segments,
    int32 rings
);
void BuildCapsuleMidMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* triangleIndices, std::vector<uint32>* lineIndices, int32 segments
);
void BuildBoxMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* triangleIndices, std::vector<uint32>* lineIndices
);
void BuildGridMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 halfExtent, float spacing);
void BuildPlaneMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices);
void BuildConvexMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    const ConvexShape& shape
);
void BuildPolygonMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    const PolygonShape& shape
);
void BuildTriangleMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* triangleIndices, std::vector<uint32>* lineIndices
);
void BuildHeightFieldMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const HeightFieldShape& shape);
void BuildMeshShapeMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const MeshShape& shape);

} // namespace muli3
