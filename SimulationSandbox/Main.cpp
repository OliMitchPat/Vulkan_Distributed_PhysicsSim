#include "Renderer.h"
#include <iostream>

int gForceShading = -1;

int RunSandbox(GLFWwindow* window, Renderer& renderer); // forward decl

int main()
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    GLFWwindow* const window = glfwCreateWindow(800, 600, "Simulation Sandbox", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }

    try
    {
        Renderer renderer(window);

        glfwSetWindowUserPointer(window, &renderer);
        glfwSetFramebufferSizeCallback(window, Renderer::framebufferResizeCallback);

        // Single run (no old reset bridge)
        RunSandbox(window, renderer);

        renderer.waitIdle();

        glfwDestroyWindow(window);
        glfwTerminate();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }
}
