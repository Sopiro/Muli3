#include "mesh.h"

namespace muli3
{

Mesh::~Mesh()
{
    Destroy();
}

void Mesh::Upload(const std::vector<MeshVertex>& vertices, const std::vector<uint32>& indices, GLenum primitiveType)
{
    Destroy();

    primitive = primitiveType;
    indexCount = (GLsizei)indices.size();

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vertices.size() * sizeof(MeshVertex)), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(indices.size() * sizeof(uint32)), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void Mesh::Destroy()
{
    if (ebo != 0)
    {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (vao != 0)
    {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }

    indexCount = 0;
}

void Mesh::Draw() const
{
    glBindVertexArray(vao);
    glDrawElements(primitive, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::DrawInstanced(GLsizei instanceCount) const
{
    glBindVertexArray(vao);
    glDrawElementsInstanced(primitive, indexCount, GL_UNSIGNED_INT, nullptr, instanceCount);
    glBindVertexArray(0);
}

GLuint Mesh::GetVAO() const
{
    return vao;
}

std::vector<MeshVertex> BuildSphereVertices(int32 segments, int32 rings)
{
    std::vector<MeshVertex> vertices;
    vertices.reserve((size_t)((segments + 1) * (rings + 1)));

    for (int32 ring = 0; ring <= rings; ++ring)
    {
        const float v = (float)ring / (float)rings;
        const float theta = v * pi;

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;

            const Vec3 normal{
                std::sin(theta) * std::cos(phi),
                std::cos(theta),
                std::sin(theta) * std::sin(phi),
            };

            vertices.push_back(MeshVertex{ normal, normal, Vec2{ u, v } });
        }
    }

    return vertices;
}

std::vector<uint32> BuildSphereIndices(int32 segments, int32 rings)
{
    std::vector<uint32> indices;
    indices.reserve((size_t)(segments * rings * 6));

    for (int32 ring = 0; ring < rings; ++ring)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            const uint32 a = (uint32)(ring * (segments + 1) + segment);
            const uint32 b = a + (uint32)(segments + 1);
            const uint32 c = a + 1;
            const uint32 d = b + 1;

            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);

            indices.push_back(c);
            indices.push_back(d);
            indices.push_back(b);
        }
    }

    return indices;
}

std::vector<MeshVertex> BuildGridVertices(int32 halfExtent, float spacing)
{
    std::vector<MeshVertex> vertices;
    vertices.reserve((size_t)((halfExtent * 2 + 1) * 4));

    for (int32 i = -halfExtent; i <= halfExtent; ++i)
    {
        const float value = (float)i * spacing;

        vertices.push_back(MeshVertex{ Vec3{ value, 0.0f, -halfExtent * spacing }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } });
        vertices.push_back(MeshVertex{ Vec3{ value, 0.0f, halfExtent * spacing }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } });

        vertices.push_back(MeshVertex{ Vec3{ -halfExtent * spacing, 0.0f, value }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } });
        vertices.push_back(MeshVertex{ Vec3{ halfExtent * spacing, 0.0f, value }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } });
    }

    return vertices;
}

std::vector<uint32> BuildGridIndices(int32 halfExtent)
{
    const int32 lineCount = halfExtent * 2 + 1;
    std::vector<uint32> indices;
    indices.reserve((size_t)(lineCount * 4));

    uint32 vertex = 0;
    for (int32 i = 0; i < lineCount; ++i)
    {
        indices.push_back(vertex + 0);
        indices.push_back(vertex + 1);
        indices.push_back(vertex + 2);
        indices.push_back(vertex + 3);
        vertex += 4;
    }

    return indices;
}

std::vector<MeshVertex> BuildPlaneVertices()
{
    return {
        MeshVertex{ Vec3{ -1.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, 0.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },
    };
}

std::vector<uint32> BuildPlaneIndices()
{
    return { 0, 2, 1, 0, 3, 2 };
}

} // namespace muli3
