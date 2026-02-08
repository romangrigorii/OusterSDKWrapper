#pragma once

// ============================================================================
// OusterStreamUtilities.h - Port management utilities
// ============================================================================

namespace Engine {
namespace Editors {
namespace CameraSystem {
namespace OusterUtilities {

// Kill processes using a specific port (macOS/Linux)
// Excludes the current process to avoid killing ourselves
// Returns true if ports appear to be free after cleanup
bool KillProcessesOnPort(int port);

// Verify that a port is actually free by checking lsof
bool VerifyPortIsFree(int port);

} // namespace OusterUtilities
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
