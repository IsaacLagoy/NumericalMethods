#ifndef RENDERER_H
#define RENDERER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "camera.h"
#include "../skeleton/robot.h"

class ShaderServer;
class ObjServer;
class SolverManager;

class Renderer {
public:
    static ShaderServer shaderServer;
    static ObjServer objServer;

private:
    GLFWwindow* window;
    Camera camera;
    Robot* robot;
    SolverManager* solverManager;

    bool cursorCaptured;
    bool firstMouse;
    double lastMouseX;
    double lastMouseY;

    void initWindow();
    void initGL();
    void setupCallbacks();
    void setCursorCaptured(bool captured);

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

public:
    Renderer(Robot* robot, SolverManager* solverManager);
    ~Renderer();

    GLFWwindow* getWindow() const { return window; }

    void captureCursor() { setCursorCaptured(true); }
    void processInput(float deltaTime);
    void render();
};

#endif
