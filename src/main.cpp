#include "render/renderer.h"

#include <GLFW/glfw3.h>

int main()
{
    Renderer renderer;
    GLFWwindow* window = renderer.getWindow();
    renderer.captureCursor();

    double lastFrameTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) 
    {
        const double currentTime = glfwGetTime();
        const float deltaTime = static_cast<float>(currentTime - lastFrameTime);
        lastFrameTime = currentTime;

        renderer.processInput(deltaTime);
        renderer.render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return 0;
}
