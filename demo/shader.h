#pragma once

#include "common.h"

namespace muli3
{

class Shader : NonCopyable
{
public:
    Shader() = default;
    ~Shader();

    bool Create(const char* vertexSource, const char* fragmentSource);
    void Destroy();
    void Use() const;

    void SetInt(const char* name, int value) const;
    void SetFloat(const char* name, float value) const;
    void SetMat4(const char* name, const Mat4& value) const;
    void SetVec3(const char* name, const Vec3& value) const;

private:
    GLuint CompileStage(GLenum type, const char* source) const;

    GLuint program = 0;
};

} // namespace muli3
