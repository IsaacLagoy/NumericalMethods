#ifndef SHADER_SERVER_H
#define SHADER_SERVER_H

// this class will manage shaders

#include <glad/glad.h>
#include <string>
#include <unordered_map>

class Shader {
private:
    GLuint vShaderID;
    GLuint fShaderID;
    GLuint programID;

    std::unordered_map<std::string, GLuint> uniformMap;
    std::unordered_map<std::string, GLuint> attributeMap;

public:
    Shader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath);
    ~Shader();

    GLuint getProgramID();
    GLuint getUniformLocation(const std::string& uniformName);
    GLuint getAttributeLocation(const std::string& attributeName);

    void bind();
    void unbind();
};

class ShaderServer {
private:
    std::unordered_map<std::string, Shader*> shaderMap;

public:
    ShaderServer() = default;
    ~ShaderServer();

    // creates shader from source and binds it
    void loadShader(const std::string& shaderName, const std::string& vertexShaderPath, const std::string& fragmentShaderPath);

    // returns shader ID
    Shader* getShader(const std::string& shaderName);
};

#endif // SHADER_SERVER_H