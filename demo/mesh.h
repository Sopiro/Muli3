#pragma once

#include "common.h"

namespace muli3
{

class ConvexShape;
class HeightFieldShape;

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
    void Destroy();
    void Draw() const;
    void DrawInstanced(GLsizei instanceCount) const;
    GLuint GetVAO() const;

private:
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    GLsizei indexCount = 0;
    GLenum primitive = GL_TRIANGLES;
};

void BuildSphereMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings);
void BuildCapsuleMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings, float height, float radius
);
void BuildCapsuleTopMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings);
void BuildCapsuleBottomMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings);
void BuildCapsuleMidMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments);
void BuildBoxMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices);
void BuildGridMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 halfExtent, float spacing);
void BuildPlaneMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices);
void BuildConvexMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const ConvexShape& shape);
void BuildTriangleMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices);
void BuildQuadMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices);
void BuildHeightFieldMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const HeightFieldShape& shape);

} // namespace muli3
