#pragma once

// ============================================================================
// OusterStreamFileLoader.h - File loading for Ouster streams
// ============================================================================

#include <string>
#include <cstdint>

namespace Engine {
namespace Editors {
namespace CameraSystem {

class OusterStream;

namespace OusterFileLoader {

// Load metadata from JSON file (for SDK initialization)
bool LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile);

// Load and parse a PCAP file with metadata JSON
bool LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);

// Load and parse a saved packet file (for debugging)
bool LoadPacketFromFile(OusterStream* stream, const std::string& filename);

// Parse metadata JSON file - internal implementation
bool ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile);

// Process lidar packet from file - internal implementation
void ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size);

} // namespace OusterFileLoader
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
