#pragma once

#include "OusterStream.h"
#include <thread>

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Utility functions for Ouster stream management
namespace OusterUtilities {

// Kill processes using a specific port (macOS/Linux)
// Excludes the current process to avoid killing ourselves
// Returns true if ports appear to be free after cleanup
bool KillProcessesOnPort(int port);

// Verify that a port is actually free by attempting to bind to it
bool VerifyPortIsFree(int port);

} // namespace OusterUtilities

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
