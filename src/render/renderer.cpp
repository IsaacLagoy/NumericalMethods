#include "renderer.h"
#include "shader_server.h"
#include "obj_server.h"
#include "matrix_stack.h"
#include "GLSL.h"

#include <iostream>
#include <stdexcept>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

ShaderServer Renderer::shaderServer;
ObjServer Renderer::objServer;

Renderer::Renderer(Robot* robot)
    : window(nullptr),
      robot(robot),
      cursorCaptured(false),
      firstMouse(true),
      lastMouseX(0.0),
      lastMouseY(0.0)
{
    initWindow();
    initGL();
    setupCallbacks();

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    shaderServer.loadShader("default", "vert.glsl", "frag.glsl");
    objServer.loadMesh("cube", "cube.obj");
    objServer.loadMesh("sphere", "sphere.obj");
}

Renderer::~Renderer()
{
    if (window) 
    {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void Renderer::initWindow()
{
    if (!glfwInit()) 
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    window = glfwCreateWindow(800, 600, "Robot", nullptr, nullptr);
    if (!window) 
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create window");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetTime(0.0);
}

void Renderer::initGL()
{
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) 
    {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    GLSL::checkVersion();
}

void Renderer::setupCallbacks()
{
    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);
}

void Renderer::setCursorCaptured(bool captured)
{
    cursorCaptured = captured;
    glfwSetInputMode(window, GLFW_CURSOR,
                     captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    if (captured) 
    {
        firstMouse = true;
    }
}

void Renderer::framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glfwMakeContextCurrent(window);
    glViewport(0, 0, width, height);
}

void Renderer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (!renderer->cursorCaptured) 
    {
        return;
    }

    if (renderer->firstMouse) 
    {
        renderer->lastMouseX = xpos;
        renderer->lastMouseY = ypos;
        renderer->firstMouse = false;
    }

    const float xoffset = static_cast<float>(xpos - renderer->lastMouseX);
    const float yoffset = static_cast<float>(renderer->lastMouseY - ypos);
    renderer->lastMouseX = xpos;
    renderer->lastMouseY = ypos;

    renderer->camera.processMouseMovement(xoffset, yoffset);
}

void Renderer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    (void)mods;
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !renderer->cursorCaptured) 
    {
        renderer->setCursorCaptured(true);
    }
}

void Renderer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) 
    {
        renderer->setCursorCaptured(false);
    }

    if (key == GLFW_KEY_COMMA && action == GLFW_PRESS)
    {
        renderer->robot->iterate(-1);
    }

    if (key == GLFW_KEY_PERIOD && action == GLFW_PRESS)
    {
        renderer->robot->iterate(1);
    }
}

void Renderer::processInput(float deltaTime)
{
    camera.processKeyboard(
        glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS,
        deltaTime
    );

    robot->processInput(
        glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS,
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS,
        deltaTime
    );
}

void Renderer::render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    const float aspect = width / static_cast<float>(height);
    const glm::mat4 projMat = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    MatrixStack MV;
    MV.multMatrix(camera.getViewMatrix());

    Shader* shader = shaderServer.getShader("default");
    shader->bind();

    const GLint projLoc = shader->getUniformLocation("P");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projMat));

    const GLint posLoc = shader->getAttributeLocation("aPos");
    glEnableVertexAttribArray(posLoc);
    const GLint norLoc = shader->getAttributeLocation("aNor");
    glEnableVertexAttribArray(norLoc);

    const GLint MVLoc = shader->getUniformLocation("MV");
    robot->draw(MV, posLoc, norLoc, MVLoc);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(posLoc);
    glDisableVertexAttribArray(norLoc);
    shader->unbind();
}
