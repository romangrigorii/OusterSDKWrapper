#pragma once

#include "OusterStream.h"
#include <string>

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Forward declaration
class OusterStream;

// File loading functionality for Ouster streams
namespace OusterFileLoader {

// Load metadata from JSON file (for use with live streaming)
// This allows faster initialization and offline operation
bool LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile);

// Load and parse a PCAP file with metadata JSON
bool LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);

// Load and parse a saved packet file (for debugging)
bool LoadPacketFromFile(OusterStream* stream, const std::string& filename);

// Parse metadata JSON file and extract beam angles, packet format, etc.
// Internal implementation (friend function)
bool ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile);

// Process lidar packet when loading from file (uses member variables)
// Internal implementation (friend function)
void ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size);

// Manual parsing functions (fallback when SDK not available)
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size);

} // namespace OusterFileLoader

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
