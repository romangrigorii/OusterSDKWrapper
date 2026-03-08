#pragma once

// ============================================================================
// OusterLogging.h - Verbose logging control for Ouster LiDAR
// ============================================================================
// Enable verbose logs by:
//   - Environment: OUSTER_VERBOSE=1
//   - Or call SetOusterVerbose(true) when your app parses --verbose (e.g. in main).
// When false, only summary logs and path (SDK vs manual) are printed.
// ============================================================================

namespace Engine {
namespace Editors {
namespace CameraSystem {

/// Returns true if verbose Ouster logging is enabled (--verbose or OUSTER_VERBOSE=1).
bool IsOusterVerbose();

/// Enable or disable verbose logging. Call with true when application sees --verbose.
void SetOusterVerbose(bool verbose);

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
