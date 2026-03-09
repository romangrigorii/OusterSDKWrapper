// ============================================================================
// OusterLogging.cpp - Verbose logging control implementation
// ============================================================================

#include "OusterLogging.h"
#include <cstdlib>
#include <atomic>

namespace Engine {
namespace Editors {
namespace CameraSystem {

static std::atomic<bool> g_ousterVerbose{false};
static std::atomic<bool> g_ousterVerboseInitialized{false};

static void InitFromEnv() {
    if (g_ousterVerboseInitialized.exchange(true)) return;
    const char* v = std::getenv("OUSTER_VERBOSE");
    if (v && (v[0] == '1' || v[0] == 't' || v[0] == 'T' || v[0] == 'y' || v[0] == 'Y'))
        g_ousterVerbose = true;
}

bool IsOusterVerbose() {
    InitFromEnv();
    return g_ousterVerbose.load();
}

void SetOusterVerbose(bool verbose) {
    g_ousterVerboseInitialized = true;
    g_ousterVerbose = verbose;
}

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
