#include <vector>

struct PendingMessage
{
    std::vector<char> data;
    uint32_t seq;
    float timer;
};

struct Peer
{
    int peerId = 0;
    sockaddr_storage addr{};

    uint32_t nextSeq = 1;
    uint32_t lastReceivedSeq = 0;

    std::vector<PendingMessage> resendQueue;
};

struct GlobalCommandPayload
{
    uint8_t commandType;
};

enum class GlobalCommandType : uint8_t
{
    ToggleGravity = 1,
    ResetScene = 2
};