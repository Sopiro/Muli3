#include "shader.h"

namespace muli3
{

Shader::~Shader()
{
    Destroy();
}

bool Shader::Create(const char* vertexSource, const char* fragmentSource)
{
    Destroy();

    const GLuint vertexShader = CompileStage(GL_VERTEX_SHADER, vertexSource);
    if (vertexShader == 0)
    {
        return false;
    }

    const GLuint fragmentShader = CompileStage(GL_FRAGMENT_SHADER, fragmentSource);
    if (fragmentShader == 0)
    {
        glDeleteShader(vertexShader);
        return false;
    }

    program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    if (linked == GL_FALSE)
    {
        GLint logLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::string log((size_t)logLength, '\0');
        glGetProgramInfoLog(program, logLength, nullptr, log.data());
        std::fprintf(stderr, "Program link failed: %s\n", log.c_str());
        Destroy();
        return false;
    }

    return true;
}

void Shader::Destroy()
{
    if (program != 0)
    {
        glDeleteProgram(program);
        program = 0;
    }
}

void Shader::Use() const
{
    glUseProgram(program);
}

void Shader::SetInt(const char* name, int value) const
{
    glUniform1i(glGetUniformLocation(program, name), value);
}

void Shader::SetFloat(const char* name, float value) const
{
    glUniform1f(glGetUniformLocation(program, name), value);
}

void Shader::SetMat4(const char* name, const Mat4& value) const
{
    glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, &value.ex.x);
}

void Shader::SetVec3(const char* name, const Vec3& value) const
{
    glUniform3f(glGetUniformLocation(program, name), value.x, value.y, value.z);
}

GLuint Shader::CompileStage(GLenum type, const char* source) const
{
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_FALSE)
    {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::string log((size_t)logLength, '\0');
        glGetShaderInfoLog(shader, logLength, nullptr, log.data());
        std::fprintf(stderr, "Shader compile failed: %s\n", log.c_str());
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

} // namespace muli3
