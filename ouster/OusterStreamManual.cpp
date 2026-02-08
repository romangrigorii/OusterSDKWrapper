// ============================================================================
// OusterStreamManual.cpp - Manual packet parsing (fallback/bridge mode)
// ============================================================================
// This file contains manual packet parsing for:
// 1. Bridge mode (when useBridge=true on macOS)
// 2. Fallback when SDK is not available
// ============================================================================

#include "OusterStream.h"
#include "platform/Logger.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

namespace Engine {
namespace Editors {
namespace CameraSystem {

// ============================================================================
// Packet structures
// ============================================================================

#pragma pack(push, 1)

// Column header for LEGACY UDP profile (16 bytes)
struct LidarColumnHeader {
    uint64_t timestamp;
    uint16_t measurementId;
    uint16_t frameId;
    uint32_t encoderCount;
};

struct LidarPixel {
    uint32_t range;
    uint16_t signal;
    uint16_t reflectivity;
    uint16_t nearIr;
    uint16_t reserved;
};

#pragma pack(pop)

// ============================================================================
// Log throttling helper
// ============================================================================

static int g_packetCount = 0;
static int g_debugCount = 0;

static bool shouldLogPacket() {
    g_packetCount++;
    return (g_packetCount <= 5 || g_packetCount % 100 == 0);
}

static bool shouldLogDebug() {
    return (++g_debugCount < 50);
}

// ============================================================================
// ProcessLidarPacketManual - Main packet parsing for live streaming
// ============================================================================
// Used by bridge mode when receiving packets via Unix socket

void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size) {
    if (!stream || !data || size == 0) return;
    
    const size_t columnHeaderSize = sizeof(LidarColumnHeader);
    const size_t blockHdrSize = static_cast<size_t>(stream->GetMeasurementBlockHeaderSize());
    const size_t pixelStride = static_cast<size_t>(stream->GetPixelStride());
    const size_t columnDataSize = static_cast<size_t>(stream->GetBeamCount()) * pixelStride;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize;
    
    // Get beam angles
    std::vector<float>& beamAltitudeAngles = stream->GetBeamAltitudeAngles();
    std::vector<float>& beamAzimuthAngles = stream->GetBeamAzimuthAngles();
    
    // Initialize default angles if not loaded from metadata
    if (beamAltitudeAngles.empty() || beamAzimuthAngles.empty()) {
        int beamCount = stream->GetBeamCount();
        beamAltitudeAngles.resize(beamCount);
        beamAzimuthAngles.resize(beamCount);
        for (int i = 0; i < beamCount; i++) {
            beamAltitudeAngles[i] = 45.0f - 90.0f * i / (beamCount - 1);
            beamAzimuthAngles[i] = 0.0f;
        }
    }
    
    if (size < columnSize) return;
    
    size_t offset = 0;
    int totalColumns = 0;
    int totalPointsProcessed = 0;
    const OusterConfig& config = stream->GetConfig();
    
    while (offset + columnSize <= size && totalColumns < stream->GetColumnsPerPacket()) {
        const uint8_t* colBase = data + offset;
        const LidarColumnHeader* header = reinterpret_cast<const LidarColumnHeader*>(colBase);
        
        if (header->timestamp == 0) {
            offset += columnSize;
            continue;
        }
        
        totalColumns++;
        
        // Log first few column headers for debugging
        if (shouldLogDebug()) {
            LOG_INFO_FMT("OusterStream", "Col %d: ts=%llu measId=%u frameId=%u encoder=%u", 
                        totalColumns, header->timestamp, header->measurementId, header->frameId, header->encoderCount);
        }
        
        uint16_t measurementId = header->measurementId;
        uint32_t encoderCount = header->encoderCount % stream->GetEncoderTicksPerRev();
        
        // Frame boundary detection
        if (!stream->IsFileLoadFrameStarted() || header->frameId != stream->GetFileLoadFrameId()) {
            // New frame - flush previous
            if (stream->IsFileLoadFrameStarted() && !stream->GetCurrentFrame().empty()) {
                stream->FlushCurrentFrame(stream->GetFileLoadLastTimestamp(), 
                                         stream->GetFileLoadMinEncoder(), 
                                         stream->GetFileLoadMaxEncoder(), 
                                         stream->IsFileLoadEncoderInitialized());
            }
            
            stream->GetCurrentFrame().clear();
            stream->SetFileLoadFrameId(header->frameId);
            stream->SetFileLoadFrameStarted(true);
            stream->SetFileLoadLastTimestamp(header->timestamp);
            stream->GetLiveLastColumnTimestampNs().store(header->timestamp);
            stream->SetFileLoadMinEncoder(encoderCount);
            stream->SetFileLoadMaxEncoder(encoderCount);
            stream->SetFileLoadEncoderInitialized(true);
        } else {
            stream->SetFileLoadLastTimestamp(header->timestamp);
            stream->GetLiveLastColumnTimestampNs().store(header->timestamp);
            
            if (stream->IsFileLoadEncoderInitialized()) {
                stream->SetFileLoadMinEncoder(std::min(stream->GetFileLoadMinEncoder(), encoderCount));
                stream->SetFileLoadMaxEncoder(std::max(stream->GetFileLoadMaxEncoder(), encoderCount));
            } else {
                stream->SetFileLoadMinEncoder(encoderCount);
                stream->SetFileLoadMaxEncoder(encoderCount);
                stream->SetFileLoadEncoderInitialized(true);
            }
        }
        
        // Process pixels
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        const uint8_t* colEnd = colBase + columnSize;
        
        for (int beam = 0; beam < stream->GetBeamCount(); beam += config.downsampleFactor) {
            const uint8_t* p = pxBase + beam * pixelStride;
            
            if (p + pixelStride > colEnd) break;
            
            uint32_t range_counts;
            uint16_t signal;
            std::memcpy(&range_counts, p + 0, sizeof(uint32_t));
            std::memcpy(&signal, p + 4, sizeof(uint16_t));
            
            float range = static_cast<float>(range_counts) * stream->GetRangeUnit();
            
            if (range <= 0.0f || range < config.minRange || range > config.maxRange) {
                continue;
            }
            
            totalPointsProcessed++;
            stream->GetCurrentFrame().push_back(
                stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles)
            );
        }
        
        offset += columnSize;
    }
    
    // Log stats periodically
    if (shouldLogPacket()) {
        LOG_INFO_FMT("OusterStream", "Packet %d: %d columns, %d points, frame %u (%zu pts accumulated)", 
                     g_packetCount, totalColumns, totalPointsProcessed, 
                     stream->GetFileLoadFrameId(), stream->GetCurrentFrame().size());
    }
}

// ============================================================================
// ProcessLidarPacketFromFileManual - Packet parsing for file loading
// ============================================================================
// Similar to live parsing but accumulates into loadedPacketPoints

void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size) {
    if (!stream || !data || size == 0) return;
    
    const size_t columnHeaderSize = sizeof(LidarColumnHeader);
    const size_t blockHdrSize = static_cast<size_t>(stream->GetMeasurementBlockHeaderSize());
    const size_t pixelStride = static_cast<size_t>(stream->GetPixelStride());
    const size_t columnDataSize = static_cast<size_t>(stream->GetBeamCount()) * pixelStride;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize;
    
    static bool configLogged = false;
    if (!configLogged) {
        LOG_INFO_FMT("OusterStream", "Packet processing config: beamCount=%d, pixelStride=%zu, columnsPerPacket=%d", 
                     stream->GetBeamCount(), pixelStride, stream->GetColumnsPerPacket());
        configLogged = true;
    }
    
    std::vector<float>& beamAltitudeAngles = stream->GetBeamAltitudeAngles();
    std::vector<float>& beamAzimuthAngles = stream->GetBeamAzimuthAngles();
    
    if (beamAltitudeAngles.empty() || beamAzimuthAngles.empty()) {
        int beamCount = stream->GetBeamCount();
        beamAltitudeAngles.resize(beamCount);
        beamAzimuthAngles.resize(beamCount);
        for (int i = 0; i < beamCount; i++) {
            beamAltitudeAngles[i] = 45.0f - 90.0f * i / (beamCount - 1);
            beamAzimuthAngles[i] = 0.0f;
        }
    }
    
    if (size < columnSize) return;
    
    size_t offset = 0;
    int totalColumns = 0;
    const OusterConfig& config = stream->GetConfig();
    
    while (offset + columnSize <= size && totalColumns < stream->GetColumnsPerPacket()) {
        const uint8_t* colBase = data + offset;
        const LidarColumnHeader* header = reinterpret_cast<const LidarColumnHeader*>(colBase);
        
        if (header->timestamp == 0) {
            offset += columnSize;
            continue;
        }
        
        totalColumns++;
        uint16_t measurementId = header->measurementId;
        uint32_t encoderCount = header->encoderCount % stream->GetEncoderTicksPerRev();
        
        // Frame boundary detection
        if (!stream->IsFileLoadFrameStarted() || header->frameId != stream->GetFileLoadFrameId()) {
            if (stream->IsFileLoadFrameStarted() && !stream->GetCurrentFrame().empty()) {
                stream->GetLoadedPacketPoints().insert(
                    stream->GetLoadedPacketPoints().end(),
                    stream->GetCurrentFrame().begin(), 
                    stream->GetCurrentFrame().end()
                );
                LOG_INFO_FMT("OusterStream", "Frame %d complete: %zu points (total: %zu)", 
                            stream->GetFileLoadFrameId(), stream->GetCurrentFrame().size(), 
                            stream->GetLoadedPacketPoints().size());
            }
            
            stream->GetCurrentFrame().clear();
            stream->SetFileLoadFrameId(header->frameId);
            stream->SetFileLoadFrameStarted(true);
            stream->SetFileLoadLastTimestamp(header->timestamp);
            stream->SetFileLoadMinEncoder(encoderCount);
            stream->SetFileLoadMaxEncoder(encoderCount);
            stream->SetFileLoadEncoderInitialized(true);
        } else {
            stream->SetFileLoadLastTimestamp(header->timestamp);
            if (stream->IsFileLoadEncoderInitialized()) {
                stream->SetFileLoadMinEncoder(std::min(stream->GetFileLoadMinEncoder(), encoderCount));
                stream->SetFileLoadMaxEncoder(std::max(stream->GetFileLoadMaxEncoder(), encoderCount));
            }
        }
        
        // Process pixels
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        const uint8_t* colEnd = colBase + columnSize;
        
        for (int beam = 0; beam < stream->GetBeamCount(); beam += config.downsampleFactor) {
            const uint8_t* p = pxBase + beam * pixelStride;
            if (p + pixelStride > colEnd) break;
            
            uint32_t range_counts;
            uint16_t signal;
            std::memcpy(&range_counts, p + 0, sizeof(uint32_t));
            std::memcpy(&signal, p + 4, sizeof(uint16_t));
            
            float range = static_cast<float>(range_counts) * stream->GetRangeUnit();
            
            if (range <= 0.0f || range < config.minRange || range > config.maxRange) {
                continue;
            }
            
            stream->GetCurrentFrame().push_back(
                stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles)
            );
        }
        
        offset += columnSize;
    }
}

// ============================================================================
// LoadPcapFromFileManual - Manual PCAP loading (fallback when SDK unavailable)
// ============================================================================

bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
    // This function is kept for backward compatibility but SDK is preferred
    LOG_WARNING("OusterStream", "LoadPcapFromFileManual is deprecated - SDK parsing is recommended");
    
#ifdef __APPLE__
    LOG_INFO_FMT("OusterStream", "Loading PCAP manually: %s", pcapFile.c_str());
    
    FILE* pcapFileHandle = fopen(pcapFile.c_str(), "rb");
    if (!pcapFileHandle) {
        LOG_ERROR_FMT("OusterStream", "Failed to open PCAP file: %s", pcapFile.c_str());
        return false;
    }
    
    // Read PCAP global header
    struct PcapGlobalHeader {
        uint32_t magic_number;
        uint16_t version_major;
        uint16_t version_minor;
        int32_t  thiszone;
        uint32_t sigfigs;
        uint32_t snaplen;
        uint32_t network;
    } globalHeader;
    
    if (fread(&globalHeader, sizeof(globalHeader), 1, pcapFileHandle) != 1) {
        LOG_ERROR("OusterStream", "Failed to read PCAP global header");
        fclose(pcapFileHandle);
        return false;
    }
    
    bool swapEndian = (globalHeader.magic_number == 0xd4c3b2a1);
    if (globalHeader.magic_number != 0xa1b2c3d4 && globalHeader.magic_number != 0xd4c3b2a1) {
        LOG_ERROR("OusterStream", "Invalid PCAP magic number");
        fclose(pcapFileHandle);
        return false;
    }
    
    auto swap32 = [swapEndian](uint32_t val) -> uint32_t {
        if (!swapEndian) return val;
        return ((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) |
               ((val & 0x0000ff00) << 8) | ((val & 0x000000ff) << 24);
    };
    
    // Reset state
    stream->GetLoadedPacketPoints().clear();
    stream->SetPacketLoadedFromFile(true);
    stream->SetFileLoadFrameId(0);
    stream->SetFileLoadFrameStarted(false);
    stream->GetCurrentFrame().clear();
    
    int packetCount = 0;
    int lidarPacketCount = 0;
    const OusterConfig& config = stream->GetConfig();
    
    while (!feof(pcapFileHandle)) {
        struct PcapPacketHeader {
            uint32_t ts_sec;
            uint32_t ts_usec;
            uint32_t incl_len;
            uint32_t orig_len;
        } packetHeader;
        
        if (fread(&packetHeader, sizeof(packetHeader), 1, pcapFileHandle) != 1) {
            if (feof(pcapFileHandle)) break;
            break;
        }
        
        if (swapEndian) {
            packetHeader.incl_len = swap32(packetHeader.incl_len);
        }
        
        if (packetHeader.incl_len == 0 || packetHeader.incl_len > 65535) break;
        
        packetCount++;
        
        std::vector<uint8_t> packetData(packetHeader.incl_len);
        if (fread(packetData.data(), 1, packetHeader.incl_len, pcapFileHandle) != packetHeader.incl_len) {
            break;
        }
        
        // Parse Ethernet + IP + UDP headers
        if (packetData.size() < 34) continue;
        
        uint16_t etherType = (packetData[12] << 8) | packetData[13];
        if (etherType != 0x0800) continue;  // Not IPv4
        
        uint8_t ipHeaderLen = (packetData[14] & 0x0F) * 4;
        if (ipHeaderLen < 20 || packetData.size() < 14 + ipHeaderLen + 8) continue;
        
        uint8_t protocol = packetData[23];
        if (protocol != 17) continue;  // Not UDP
        
        size_t udpOffset = 14 + ipHeaderLen;
        uint16_t dstPort = (packetData[udpOffset + 2] << 8) | packetData[udpOffset + 3];
        if (dstPort != config.lidarPort) continue;
        
        uint16_t udpLength = (packetData[udpOffset + 4] << 8) | packetData[udpOffset + 5];
        if (udpLength < 8) continue;
        
        size_t udpPayloadLen = udpLength - 8;
        if (packetData.size() < udpOffset + 8 + udpPayloadLen) continue;
        
        std::vector<uint8_t> lidarPacket(udpPayloadLen);
        std::memcpy(lidarPacket.data(), packetData.data() + udpOffset + 8, udpPayloadLen);
        
        ProcessLidarPacketFromFileManual(stream, lidarPacket.data(), lidarPacket.size());
        lidarPacketCount++;
    }
    
    fclose(pcapFileHandle);
    
    // Flush final frame
    if (stream->IsFileLoadFrameStarted() && !stream->GetCurrentFrame().empty()) {
        stream->GetLoadedPacketPoints().insert(
            stream->GetLoadedPacketPoints().end(),
            stream->GetCurrentFrame().begin(),
            stream->GetCurrentFrame().end()
        );
    }
    
    LOG_INFO_FMT("OusterStream", "PCAP loaded: %d packets, %d lidar, %zu points", 
                 packetCount, lidarPacketCount, stream->GetLoadedPacketPoints().size());
    
    if (stream->GetLoadedPacketPoints().empty()) {
        LOG_WARNING("OusterStream", "No points extracted from PCAP file");
        return false;
    }
    
    stream->SetConnected(true);
    stream->SetStreaming(false);
    return true;
#else
    LOG_ERROR("OusterStream", "Manual PCAP loading only supported on macOS");
    return false;
#endif
}

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
