#pragma once

// ============================================================================
// OusterStreamCommon.h - Shared includes and forward declarations
// ============================================================================
// This header consolidates common system headers used across the Ouster
// streaming implementation to reduce duplication and improve compile times.
// ============================================================================

// Standard library
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>

// Platform-specific networking headers
#ifdef __APPLE__
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#elif __linux__
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#endif

// GLM for math types
#include <glm/glm.hpp>

// ============================================================================
// Forward declarations for Ouster SDK types
// ============================================================================
// These are always declared (even when SDK not available) to avoid errors.
// The actual usage is guarded by #ifdef OUSTER_SDK_AVAILABLE
namespace ouster {
namespace sdk {
namespace sensor {
class SensorScanSource;
}
namespace core {
class SensorInfo;
// Note: XYZLut is a typedef, we handle it in implementation files
}
}
}

// ============================================================================
// Logging throttle helper
// ============================================================================
// Use this to avoid spamming logs. Thread-safe via atomic counter.
namespace Engine {
namespace Editors {
namespace CameraSystem {

// Simple log throttle helper - returns true every N calls
class LogThrottle {
public:
    explicit LogThrottle(int interval = 100) : m_interval(interval), m_count(0) {}
    
    bool shouldLog() {
        int count = ++m_count;
        return (count <= 5 || count % m_interval == 0);
    }
    
    int count() const { return m_count.load(); }
    void reset() { m_count = 0; }
    
private:
    int m_interval;
    std::atomic<int> m_count;
};

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
