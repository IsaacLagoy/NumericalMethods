#include "render/renderer.h"
#include "solver/gn_solver.h"

#include <GLFW/glfw3.h>

int main()
{
    Robot robot;
    GaussNewtonSolver solver(&robot);
    Renderer renderer(&robot);
    GLFWwindow* window = renderer.getWindow();
    renderer.captureCursor();

    double lastFrameTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) 
    {
        const double currentTime = glfwGetTime();
        const float deltaTime = static_cast<float>(currentTime - lastFrameTime);
        lastFrameTime = currentTime;

        renderer.processInput(deltaTime);

        solver.step();

        renderer.render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return 0;
}
