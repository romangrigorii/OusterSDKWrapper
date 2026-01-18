#pragma once

#include "OusterStream.h"
#include <string>

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Forward declaration
class OusterStream;

// Live streaming functionality for Ouster streams
namespace OusterLiveStreaming {

// Connect to sensor
bool Connect(OusterStream* stream);

// Disconnect from sensor
void Disconnect(OusterStream* stream);

// Start streaming (creates streaming thread)
void StartStreaming(OusterStream* stream);

// Stop streaming (joins streaming thread)
void StopStreaming(OusterStream* stream);

// Main streaming thread function (internal implementation - friend function)
void StreamingThreadImpl(OusterStream* stream);

// Process lidar packet from live stream (internal implementation - friend function)
void ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size);

} // namespace OusterLiveStreaming

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
