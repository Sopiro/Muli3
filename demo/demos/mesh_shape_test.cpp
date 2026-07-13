#include "demo.h"
#include "game.h"

#include <fstream>
#include <sstream>

namespace muli3
{

static bool LoadObj(
    const char* path, const Transform& transform, const Vec3& scale, std::vector<Vec3>* vertices, std::vector<int32>* indices
)
{
    std::ifstream file(path);
    if (file.is_open() == false)
    {
        return false;
    }

    vertices->clear();
    indices->clear();

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream stream(line);
        std::string type;
        stream >> type;
        if (type == "v")
        {
            Vec3 vertex;
            stream >> vertex.x >> vertex.y >> vertex.z;
            vertices->push_back(Mul(transform, scale * vertex));
        }
        else if (type == "f")
        {
            int32 face[32];
            int32 count = 0;
            std::string token;
            while (stream >> token)
            {
                size_t slash = token.find('/');
                int32 index = std::stoi(token.substr(0, slash));
                MuliAssert(count < int32(std::size(face)));
                face[count++] = index > 0 ? index - 1 : int32(vertices->size()) + index;
            }

            // OBJ faces may be polygons. Preserve winding while triangulating them as a fan.
            for (int32 i = 1; i + 1 < count; ++i)
            {
                indices->push_back(face[0]);
                indices->push_back(face[i]);
                indices->push_back(face[i + 1]);
            }
        }
    }

    return vertices->empty() == false && indices->empty() == false;
}

class MeshShapeDemo : public Demo
{
public:
    MeshShapeDemo(Game& game)
        : Demo(game)
    {
        std::vector<Vec3> vertices;
        std::vector<int32> indices;
        bool loaded = LoadObj(MULI3_RES_DIR "/background.obj", identity, Vec3{ 10.0f }, &vertices, &indices);
        MuliAssert(loaded);

        world->CreateMesh(vertices, indices);

        camera.SetPosition(Vec3{ 0.0f, 3.0f, 10.0f });
    }
};

static Demo* CreateMeshShapeDemo(Game& game)
{
    return new MeshShapeDemo(game);
}

static int32 mesh_shape = register_demo("Shapes", "Mesh shape", CreateMeshShapeDemo, 4);

} // namespace muli3
