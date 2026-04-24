#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include "Renderer.h"
#include "PeerConfig.h"
#include "NetAddress.h"
#include <iostream>
#include <string>

int gForceShading = -1;

int RunSandbox(GLFWwindow* window, Renderer& renderer); // forward decl

// Print a summary of the loaded peer configuration to stdout.
static void PrintConfigSummary(const Net::PeerConfig& cfg)
{
    std::cout << "=== Peer Configuration ===\n";
    std::cout << "  My peer id  : " << cfg.peer_id << "\n";
    std::cout << "  Bind address: " << cfg.bind_ip << ":" << cfg.bind_port << "\n";
    std::cout << "  Loop Hz     : render=" << cfg.render_hz
              << "  network=" << cfg.network_hz
              << "  simulation=" << cfg.simulation_hz << "\n";
    std::cout << "  Peers (" << cfg.peers.size() << "):\n";
    for (const auto& p : cfg.peers)
    {
        std::cout << "    [" << p.peerId << "] " << p.host << ":" << p.port;
        if (p.peerId == cfg.peer_id)
            std::cout << "  <-- self";
        std::cout << "\n";
    }
    std::cout << "==========================\n";
}

int main(int argc, char* argv[])
{
    // ---- Winsock2 initialisation (required before any socket / name-resolution calls) ----
    WSADATA wsaData{};
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed\n";
        return EXIT_FAILURE;
    }

    // ---- Command-line: --config <path> (default: config/peer1.ini) ----
    std::string configPath = "config/peer1.ini";
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--config" && i + 1 < argc)
        {
            configPath = argv[i + 1];
            ++i;
        }
    }

    // ---- Load peer configuration ----
    Net::PeerConfig cfg;
    std::string     configError;
    if (!Net::ParsePeerConfig(configPath, cfg, configError))
    {
        std::cerr << "Failed to load config \"" << configPath << "\": "
                  << configError << "\n";
        WSACleanup();
        return EXIT_FAILURE;
    }

    PrintConfigSummary(cfg);

    // ---- Resolve peer addresses (scaffolding — sockets created later) ----
    for (const auto& peer : cfg.peers)
    {
        sockaddr_storage addr{};
        std::string      resolveError;
        if (Net::ResolveAddress(peer.host, peer.port, addr, resolveError))
        {
            std::cout << "  Resolved peer " << peer.peerId
                      << " -> " << Net::AddressToString(addr) << "\n";
        }
        else
        {
            std::cerr << "  Warning: could not resolve peer " << peer.peerId
                      << " (" << peer.host << ":" << peer.port
                      << "): " << resolveError << "\n";
        }
    }

    // ---- Graphics initialisation ----
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        WSACleanup();
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    GLFWwindow* const window = glfwCreateWindow(800, 600, "Simulation Sandbox", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        WSACleanup();
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
        WSACleanup();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        WSACleanup();
        return EXIT_FAILURE;
    }
}

