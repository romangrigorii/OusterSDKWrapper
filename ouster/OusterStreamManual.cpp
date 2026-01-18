#include "OusterStream.h"
#include "../../engine/platform/Logger.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <mutex>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

using EngineLogging::Logger;

namespace Engine {
namespace Editors {
namespace CameraSystem {

#pragma pack(push, 1)

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

// Manual PCAP loading implementation
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
#ifdef __APPLE__
    LOG_INFO_FMT("OusterStream", "Loading PCAP manually: %s with metadata: %s", pcapFile.c_str(), metadataFile.c_str());
    
    // Try multiple path resolutions
    std::vector<std::string> pcapPathsToTry;
    std::vector<std::string> metadataPathsToTry;
    
    // Helper function to generate paths
    auto generatePaths = [](const std::string& filename) -> std::vector<std::string> {
        std::vector<std::string> paths;
        paths.push_back(filename);
        
        try {
            paths.push_back((std::filesystem::current_path() / filename).string());
        } catch (...) {}
        
        char exePath[1024];
        uint32_t size = sizeof(exePath);
        if (_NSGetExecutablePath(exePath, &size) == 0) {
            try {
                std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
                paths.push_back((exeDir / "../" / filename).string());
                paths.push_back((exeDir / "../../" / filename).string());
                paths.push_back((exeDir / filename).string());
            } catch (...) {}
        }
        
        paths.push_back("../" + filename);
        paths.push_back("../../" + filename);
        paths.push_back("../../../" + filename);
        
        return paths;
    };
    
    pcapPathsToTry = generatePaths(pcapFile);
    metadataPathsToTry = generatePaths(metadataFile);
    
    FILE* pcapFileHandle = nullptr;
    std::string actualPcapPath;
    
    for (const auto& path : pcapPathsToTry) {
        try {
            if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
                pcapFileHandle = fopen(path.c_str(), "rb");
                if (pcapFileHandle) {
                    actualPcapPath = path;
                    LOG_INFO_FMT("OusterStream", "Found PCAP file at: %s", path.c_str());
                    break;
                }
            }
        } catch (...) {
            continue;
        }
    }
    
    if (!pcapFileHandle) {
        LOG_ERROR_FMT("OusterStream", "Failed to open PCAP file: %s", pcapFile.c_str());
        return false;
    }
    
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
    
    bool swapEndian = false;
    if (globalHeader.magic_number == 0xd4c3b2a1) {
        swapEndian = true;
    } else if (globalHeader.magic_number != 0xa1b2c3d4) {
        LOG_ERROR("OusterStream", "Invalid PCAP magic number");
        fclose(pcapFileHandle);
        return false;
    }
    
    auto swap32 = [swapEndian](uint32_t val) -> uint32_t {
        if (!swapEndian) return val;
        return ((val & 0xff000000) >> 24) |
               ((val & 0x00ff0000) >> 8) |
               ((val & 0x0000ff00) << 8) |
               ((val & 0x000000ff) << 24);
    };
    
    if (swapEndian) {
        globalHeader.snaplen = swap32(globalHeader.snaplen);
        globalHeader.network = swap32(globalHeader.network);
    }
    
    LOG_INFO_FMT("OusterStream", "PCAP: snaplen=%u, network=%u", globalHeader.snaplen, globalHeader.network);
    
    std::string actualMetadataPath;
    for (const auto& path : metadataPathsToTry) {
        try {
            if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
                actualMetadataPath = path;
                LOG_INFO_FMT("OusterStream", "Found metadata file at: %s", path.c_str());
                break;
            }
        } catch (...) {
            continue;
        }
    }
    
    if (actualMetadataPath.empty()) {
        LOG_WARNING_FMT("OusterStream", "Metadata file not found: %s, using default beam angles", metadataFile.c_str());
    } else {
        if (!stream->ParseMetadataFile(actualMetadataPath)) {
            LOG_WARNING("OusterStream", "Failed to parse metadata, using default beam angles");
        } else {
            LOG_INFO("OusterStream", "Successfully parsed metadata file");
        }
    }
    
    stream->m_loadedPacketPoints.clear();
    stream->m_packetLoadedFromFile = true;
    
    stream->m_fileLoadFrameId = 0;
    stream->m_fileLoadFrameStarted = false;
    stream->m_fileLoadLastTimestamp = 0;
    stream->m_fileLoadMinEncoder = 0;
    stream->m_fileLoadMaxEncoder = 0;
    stream->m_fileLoadEncoderInitialized = false;
    stream->m_currentFrame.clear();
    
    int packetCount = 0;
    int lidarPacketCount = 0;
    
    while (!feof(pcapFileHandle)) {
        struct PcapPacketHeader {
            uint32_t ts_sec;
            uint32_t ts_usec;
            uint32_t incl_len;
            uint32_t orig_len;
        } packetHeader;
        
        if (fread(&packetHeader, sizeof(packetHeader), 1, pcapFileHandle) != 1) {
            if (feof(pcapFileHandle)) break;
            LOG_ERROR("OusterStream", "Failed to read PCAP packet header");
            break;
        }
        
        if (swapEndian) {
            packetHeader.incl_len = swap32(packetHeader.incl_len);
            packetHeader.orig_len = swap32(packetHeader.orig_len);
        }
        
        if (packetHeader.incl_len == 0 || packetHeader.incl_len > 65535) {
            break;
        }
        
        packetCount++;
        
        std::vector<uint8_t> packetData(packetHeader.incl_len);
        if (fread(packetData.data(), 1, packetHeader.incl_len, pcapFileHandle) != packetHeader.incl_len) {
            LOG_WARNING("OusterStream", "Failed to read complete packet data");
            break;
        }
        
        if (packetData.size() < 14) continue;
        
        uint16_t etherType = (packetData[12] << 8) | packetData[13];
        if (etherType != 0x0800) continue;
        
        if (packetData.size() < 34) continue;
        
        uint8_t ipHeaderLen = (packetData[14] & 0x0F) * 4;
        if (ipHeaderLen < 20 || packetData.size() < 14 + ipHeaderLen + 8) continue;
        
        uint8_t protocol = packetData[23];
        if (protocol != 17) continue;
        
        size_t udpOffset = 14 + ipHeaderLen;
        if (packetData.size() < udpOffset + 8) continue;
        
        uint16_t dstPort = (packetData[udpOffset + 2] << 8) | packetData[udpOffset + 3];
        if (dstPort != stream->m_config.lidarPort) continue;
        
        uint16_t udpLength = (packetData[udpOffset + 4] << 8) | packetData[udpOffset + 5];
        
        if (udpLength < 8) continue;
        
        size_t udpPayloadLen = udpLength - 8;
        if (packetData.size() < udpOffset + 8 + udpPayloadLen) {
            LOG_WARNING_FMT("OusterStream", "UDP payload length mismatch: expected %zu, have %zu", 
                           udpOffset + 8 + udpPayloadLen, packetData.size());
            continue;
        }
        
        const size_t minLidarPacketSize = 16 + 12;
        if (udpPayloadLen < minLidarPacketSize) {
            continue;
        }
        
        std::vector<uint8_t> lidarPacket(udpPayloadLen);
        std::memcpy(lidarPacket.data(), packetData.data() + udpOffset + 8, udpPayloadLen);
        
        if (lidarPacketCount < 3) {
            LOG_INFO_FMT("OusterStream", "Packet %d: UDP payload size = %zu bytes", lidarPacketCount, udpPayloadLen);
        }
        
        ProcessLidarPacketFromFileManual(stream, lidarPacket.data(), lidarPacket.size());
        lidarPacketCount++;
        
        if (lidarPacketCount <= 5 || lidarPacketCount % 100 == 0) {
            LOG_INFO_FMT("OusterStream", "Processed %d lidar packets from PCAP", lidarPacketCount);
        }
    }
    
    fclose(pcapFileHandle);
    
    if (stream->m_fileLoadFrameStarted && !stream->m_currentFrame.empty()) {
        double timestamp = static_cast<double>(stream->m_fileLoadLastTimestamp) / 1e9;
        stream->m_loadedPacketPoints.insert(stream->m_loadedPacketPoints.end(), 
                                           stream->m_currentFrame.begin(), stream->m_currentFrame.end());
        stream->m_loadedPacketTimestamp = timestamp;
        LOG_INFO_FMT("OusterStream", "Final frame complete: %zu points added", stream->m_currentFrame.size());
    }
    
    {
        std::lock_guard<std::mutex> lock(stream->m_pointCloudMutex);
        while (!stream->m_pointCloudQueue.empty()) {
            auto& frame = stream->m_pointCloudQueue.front();
            stream->m_loadedPacketPoints.insert(stream->m_loadedPacketPoints.end(), 
                                               frame.first.begin(), frame.first.end());
            stream->m_loadedPacketTimestamp = frame.second;
            stream->m_pointCloudQueue.pop();
        }
    }
    
    LOG_INFO_FMT("OusterStream", "PCAP loaded: %d total packets, %d lidar packets, %zu points extracted", 
                 packetCount, lidarPacketCount, stream->m_loadedPacketPoints.size());
    
    if (stream->m_loadedPacketPoints.empty()) {
        LOG_WARNING("OusterStream", "No points extracted from PCAP file");
        return false;
    }
    
    stream->m_connected = true;
    stream->m_streaming = false;
    
    return true;
#else
    LOG_ERROR("OusterStream", "LoadPcapFromFileManual: Only macOS is currently supported");
    return false;
#endif
}

// Manual packet processing for file loading
void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size) {
    const size_t columnHeaderSize = sizeof(LidarColumnHeader);
    const size_t blockHdrSize = static_cast<size_t>(stream->m_measurementBlockHeaderSize);
    const size_t pixelStride = static_cast<size_t>(stream->m_pixelStride);
    const size_t columnDataSize = static_cast<size_t>(stream->m_beamCount) * pixelStride;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize;
    
    static bool configLogged = false;
    if (!configLogged) {
        LOG_INFO_FMT("OusterStream", "Packet processing config: beamCount=%d, pixelStride=%d, columnsPerPacket=%d, encoderTicksPerRev=%d, rangeUnit=%.6f", 
                     stream->m_beamCount, stream->m_pixelStride, stream->m_columnsPerPacket, stream->m_encoderTicksPerRev, stream->m_rangeUnit);
        LOG_INFO_FMT("OusterStream", "  Column: header=%zu, blockHdr=%zu, data=%zu, total=%zu", columnHeaderSize, blockHdrSize, columnDataSize, columnSize);
        configLogged = true;
    }
    
    std::vector<float> beamAltitudeAngles = stream->m_beamAltitudeAngles;
    std::vector<float> beamAzimuthAngles = stream->m_beamAzimuthAngles;
    
    if (beamAltitudeAngles.empty() || beamAzimuthAngles.empty()) {
        beamAltitudeAngles.resize(stream->m_beamCount);
        beamAzimuthAngles.resize(stream->m_beamCount);
        for (int i = 0; i < stream->m_beamCount; i++) {
            beamAltitudeAngles[i] = 45.0f - 90.0f * i / (stream->m_beamCount - 1);
            beamAzimuthAngles[i] = 0.0f;
        }
    }
    
    size_t offset = 0;
    int totalColumns = 0;
    int skippedColumns = 0;
    int totalPointsProcessed = 0;
    
    if (size < columnSize) {
        return;
    }
    
    while (offset + columnSize <= size && totalColumns < stream->m_columnsPerPacket) {
        if (offset + columnSize > size) {
            break;
        }
        
        const uint8_t* colBase = data + offset;
        const LidarColumnHeader* header = reinterpret_cast<const LidarColumnHeader*>(colBase);
        
        static bool firstHeaderLogged = false;
        if (!firstHeaderLogged && totalColumns == 0) {
            LOG_INFO_FMT("OusterStream", "First column header: timestamp=%llu, measurementId=%u, frameId=%u, encoderCount=%u", 
                        header->timestamp, header->measurementId, header->frameId, header->encoderCount);
            firstHeaderLogged = true;
        }
        
        if (header->timestamp == 0) {
            skippedColumns++;
            offset += columnSize;
            continue;
        }
        
        totalColumns++;
        
        uint16_t measurementId = header->measurementId;
        uint32_t encoderCount = header->encoderCount % stream->m_encoderTicksPerRev;
        
        if (!stream->m_fileLoadFrameStarted || header->frameId != stream->m_fileLoadFrameId) {
            if (stream->m_fileLoadFrameStarted && !stream->m_currentFrame.empty()) {
                stream->m_loadedPacketPoints.insert(stream->m_loadedPacketPoints.end(), 
                                                   stream->m_currentFrame.begin(), stream->m_currentFrame.end());
                LOG_INFO_FMT("OusterStream", "Frame %d complete: %zu points (total: %zu)", 
                            stream->m_fileLoadFrameId, stream->m_currentFrame.size(), stream->m_loadedPacketPoints.size());
            }
            
            stream->m_currentFrame.clear();
            stream->m_fileLoadFrameId = header->frameId;
            stream->m_fileLoadFrameStarted = true;
            stream->m_fileLoadLastTimestamp = header->timestamp;
            stream->m_fileLoadMinEncoder = stream->m_fileLoadMaxEncoder = encoderCount;
            stream->m_fileLoadEncoderInitialized = true;
        } else {
            stream->m_fileLoadLastTimestamp = header->timestamp;
            
            if (stream->m_fileLoadEncoderInitialized) {
                stream->m_fileLoadMinEncoder = std::min(stream->m_fileLoadMinEncoder, encoderCount);
                stream->m_fileLoadMaxEncoder = std::max(stream->m_fileLoadMaxEncoder, encoderCount);
            } else {
                stream->m_fileLoadMinEncoder = stream->m_fileLoadMaxEncoder = encoderCount;
                stream->m_fileLoadEncoderInitialized = true;
            }
        }
        
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        const uint8_t* colEnd = colBase + columnSize;
        
        for (int beam = 0; beam < stream->m_beamCount; beam += stream->m_config.downsampleFactor) {
            const uint8_t* p = pxBase + beam * pixelStride;
            
            if (p + pixelStride > colEnd) {
                break;
            }
            
            uint32_t range_counts;
            uint16_t signal;
            std::memcpy(&range_counts, p + 0, sizeof(uint32_t));
            std::memcpy(&signal, p + 4, sizeof(uint16_t));
            
            float range = static_cast<float>(range_counts) * stream->m_rangeUnit;
            
            if (range <= 0.0f || range < stream->m_config.minRange || range > stream->m_config.maxRange) {
                continue;
            }
            
            totalPointsProcessed++;
            stream->m_currentFrame.push_back(stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles));
        }
        
        offset += columnSize;
    }
}

// Manual packet processing for live streaming
void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size) {
    const size_t columnHeaderSize = sizeof(LidarColumnHeader);
    const size_t blockHdrSize = static_cast<size_t>(stream->m_measurementBlockHeaderSize);
    const size_t pixelStride = static_cast<size_t>(stream->m_pixelStride);
    const size_t columnDataSize = static_cast<size_t>(stream->m_beamCount) * pixelStride;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize;
    
    std::vector<float> beamAltitudeAngles = stream->m_beamAltitudeAngles;
    std::vector<float> beamAzimuthAngles = stream->m_beamAzimuthAngles;
    if (beamAltitudeAngles.empty() || beamAzimuthAngles.empty()) {
        beamAltitudeAngles.resize(stream->m_beamCount);
        beamAzimuthAngles.resize(stream->m_beamCount);
        for (int i = 0; i < stream->m_beamCount; i++) {
            beamAltitudeAngles[i] = 45.0f - 90.0f * i / (stream->m_beamCount - 1);
            beamAzimuthAngles[i] = 0.0f;
        }
    }
    
    static thread_local uint16_t currentFrameId = 0;
    static thread_local bool frameStarted = false;
    static thread_local uint64_t lastColumnTimestamp = 0;
    static thread_local uint32_t minEncoder = 0, maxEncoder = 0;
    static thread_local bool encoderInitialized = false;
    
    size_t offset = 0;
    int totalColumns = 0, skippedColumns = 0, totalPointsProcessed = 0;
    
    while (offset + columnSize <= size) {
        totalColumns++;
        if (totalColumns > 2000) {
            LOG_WARNING("OusterStream", "Too many columns, stopping parse");
            break;
        }
        
        const uint8_t* colBase = data + offset;
        const LidarColumnHeader* header = reinterpret_cast<const LidarColumnHeader*>(colBase);
        
        if (header->timestamp == 0) {
            skippedColumns++;
            offset += columnSize;
            continue;
        }
        
        uint32_t encoderCount = header->encoderCount % stream->m_encoderTicksPerRev;
        
        if (!frameStarted || header->frameId != currentFrameId) {
            if (frameStarted && !stream->m_currentFrame.empty()) {
                stream->FlushCurrentFrame(lastColumnTimestamp, minEncoder, maxEncoder, encoderInitialized);
            }
            
            stream->m_currentFrame.clear();
            currentFrameId = header->frameId;
            frameStarted = true;
            lastColumnTimestamp = header->timestamp;
            stream->m_liveLastColumnTimestampNs.store(header->timestamp);
            minEncoder = maxEncoder = encoderCount;
            encoderInitialized = true;
            if (totalColumns == 1) {
                LOG_INFO_FMT("OusterStream", "Starting frame %d (first column, encoder: %u)", 
                             currentFrameId, encoderCount);
            }
        } else {
            lastColumnTimestamp = header->timestamp;
            stream->m_liveLastColumnTimestampNs.store(header->timestamp);
            
            if (encoderInitialized) {
                minEncoder = std::min(minEncoder, encoderCount);
                maxEncoder = std::max(maxEncoder, encoderCount);
            } else {
                minEncoder = maxEncoder = encoderCount;
                encoderInitialized = true;
            }
        }
        
        uint16_t measurementId = header->measurementId;
        
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        
        for (int beam = 0; beam < stream->m_beamCount; beam += stream->m_config.downsampleFactor) {
            const uint8_t* p = pxBase + beam * pixelStride;
            
            uint32_t range_counts;
            uint16_t signal;
            std::memcpy(&range_counts, p + 0, sizeof(uint32_t));
            std::memcpy(&signal, p + 4, sizeof(uint16_t));
            
            float range = static_cast<float>(range_counts) * stream->m_rangeUnit;
            if (range <= 0.0f || range < stream->m_config.minRange || range > stream->m_config.maxRange) {
                continue;
            }
            
            totalPointsProcessed++;
            stream->m_currentFrame.push_back(stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles));
        }
        
        offset += columnSize;
    }
    
    LOG_INFO_FMT("OusterStream", "Packet parsing stats: %d columns processed, %d skipped, %d total points extracted", 
                 totalColumns, skippedColumns, totalPointsProcessed);
}

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
