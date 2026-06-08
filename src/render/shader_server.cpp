#include "shader_server.h"
#include "GLSL.h"

// ------------------------------------------------------------
// Shader class
// ------------------------------------------------------------
Shader::Shader(const std::string& vertexShaderPath, const std::string& fragmentShaderPath)
{
    // create shader handles
    GLuint vShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint fShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    // read shader source
    std::string vShaderName = RESOURCE_DIR + vertexShaderPath;
    std::string fShaderName = RESOURCE_DIR + fragmentShaderPath;
    const char* vShaderText = GLSL::textFileRead(vShaderName.c_str());
    const char* fShaderText = GLSL::textFileRead(fShaderName.c_str());
    glShaderSource(vShaderID, 1, &vShaderText, NULL);
    glShaderSource(fShaderID, 1, &fShaderText, NULL);

    // compile vertex sahder
    int rc;
    glCompileShader(vShaderID);
    glGetShaderiv(vShaderID, GL_COMPILE_STATUS, &rc);
    if (rc == GL_FALSE) {
        throw std::runtime_error("Vertex shader compilation failed");
    }

    // compile fragment shader
    glCompileShader(fShaderID);
    glGetShaderiv(fShaderID, GL_COMPILE_STATUS, &rc);
    if (rc == GL_FALSE) {
        throw std::runtime_error("Fragment shader compilation failed");
    }

    // link shader
    GLuint programID = glCreateProgram();
    glAttachShader(programID, vShaderID);
    glAttachShader(programID, fShaderID);
    glLinkProgram(programID);
    glGetProgramiv(programID, GL_LINK_STATUS, &rc);
    if (rc == GL_FALSE) {
        throw std::runtime_error("Shader linking failed");
    }

    // add uniform mappings
    GLint numUniforms;
    glGetProgramiv(programID, GL_ACTIVE_UNIFORMS, &numUniforms);
    for (int i = 0; i < numUniforms; i++) {
        char name[100];
        GLint size;
        GLenum type;
        glGetActiveUniform(programID, i, sizeof(name), NULL, &size, &type, name);
        uniformMap[name] = glGetUniformLocation(programID, name);
    }

    // add attribute mappings
    GLint numAttributes;
    glGetProgramiv(programID, GL_ACTIVE_ATTRIBUTES, &numAttributes);
    for (int i = 0; i < numAttributes; i++) {
        char name[100];
        GLint size;
        GLenum type;
        glGetActiveAttrib(programID, i, sizeof(name), NULL, &size, &type, name);
        attributeMap[name] = glGetAttribLocation(programID, name);
    }

    // store shader
    this->vShaderID = vShaderID;
    this->fShaderID = fShaderID;
    this->programID = programID;
}

Shader::~Shader()
{
    glDeleteShader(vShaderID);
    glDeleteShader(fShaderID);
    glDeleteProgram(programID);
}

GLuint Shader::getProgramID()
{
    return programID;
}

GLuint Shader::getUniformLocation(const std::string& uniformName)
{
    return uniformMap[uniformName];
}

GLuint Shader::getAttributeLocation(const std::string& attributeName)
{
    return attributeMap[attributeName];
}

void Shader::bind()
{
    glUseProgram(programID);
}

void Shader::unbind()
{
    glUseProgram(0);
}

// ------------------------------------------------------------
// ShaderServer class
// ------------------------------------------------------------

ShaderServer::~ShaderServer()
{
    for (auto& shader : shaderMap) {
        delete shader.second;
    }
}

void ShaderServer::loadShader(const std::string& shaderName, const std::string& vertexShaderPath, const std::string& fragmentShaderPath)
{
    // check if shader already exists
    if (shaderMap.find(shaderName) != shaderMap.end()) {
        throw std::runtime_error("Shader already exists");
    }

    // create shader
    Shader* shader = new Shader(vertexShaderPath, fragmentShaderPath);
    shaderMap[shaderName] = shader;
}

Shader* ShaderServer::getShader(const std::string& shaderName)
{
    return shaderMap[shaderName];
}