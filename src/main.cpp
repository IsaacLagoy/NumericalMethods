#include "render/renderer.h"
#include "solver/solver_manager.h"

#include <GLFW/glfw3.h>

int main()
{
    Robot robot;
    SolverManager solverManager(&robot);
    Renderer renderer(&robot, &solverManager);
    GLFWwindow* window = renderer.getWindow();
    renderer.captureCursor();

    double lastFrameTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) 
    {
        const double currentTime = glfwGetTime();
        if (0.016f + lastFrameTime > currentTime) continue;
        const float deltaTime = static_cast<float>(currentTime - lastFrameTime);
        lastFrameTime = currentTime;

        renderer.processInput(deltaTime);

        solverManager.step();

        renderer.render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return 0;
}
