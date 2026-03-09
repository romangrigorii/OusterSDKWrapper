#pragma once

// ============================================================================
// OusterStreamLiveStreaming.h - Live sensor streaming
// ============================================================================

#include <cstdint>
#include <cstddef>

namespace Engine {
namespace Editors {
namespace CameraSystem {

class OusterStream;

namespace OusterLiveStreaming {

// Connect to sensor (configure UDP destination)
bool Connect(OusterStream* stream);

// Disconnect from sensor
void Disconnect(OusterStream* stream);

// Start streaming (creates streaming thread)
void StartStreaming(OusterStream* stream);

// Stop streaming (joins streaming thread)
void StopStreaming(OusterStream* stream);

// Main streaming thread function
void StreamingThreadImpl(OusterStream* stream);

// Process lidar packet from live stream
void ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size);

} // namespace OusterLiveStreaming
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
