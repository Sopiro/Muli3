#pragma once

#include "common.h"

namespace muli3
{

struct Vertex
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

    void Upload(const std::vector<Vertex>& vertices, const std::vector<uint32>& indices, GLenum primitiveType);
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

std::vector<Vertex> BuildSphereVertices(int32 segments, int32 rings);
std::vector<uint32> BuildSphereIndices(int32 segments, int32 rings);
std::vector<Vertex> BuildGridVertices(int32 halfExtent, float spacing);
std::vector<uint32> BuildGridIndices(int32 halfExtent);
std::vector<Vertex> BuildPlaneVertices();
std::vector<uint32> BuildPlaneIndices();

} // namespace muli3
