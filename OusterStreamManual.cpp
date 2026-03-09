// ============================================================================
// OusterStreamManual.cpp - Manual packet parsing (fallback/bridge mode)
// ============================================================================
// This file contains manual packet parsing for:
// 1. Bridge mode (when useBridge=true on macOS)
// 2. Fallback when SDK is not available
// ============================================================================

#include "OusterStream.h"
#include "OusterLogging.h"
#include "platform/Logger.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

#ifdef OUSTER_SDK_AVAILABLE
#include "ouster/xyzlut.h"
#include "ouster/typedefs.h"
#include <Eigen/Core>

using namespace ouster::sdk::core;
#endif

// Ouster UDP packets use big-endian; convert to host for little-endian (e.g. Windows)
namespace {
inline uint32_t be32_to_host(uint32_t x) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return ((x & 0xFFU) << 24) | ((x & 0xFF00U) << 8) | ((x & 0xFF0000U) >> 8) | ((x & 0xFF000000U) >> 24);
#else
    return x;
#endif
}
inline uint16_t be16_to_host(uint16_t x) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return static_cast<uint16_t>(((x & 0xFFU) << 8) | ((x & 0xFF00U) >> 8));
#else
    return x;
#endif
}
inline uint64_t be64_to_host(uint64_t x) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return ((x & 0xFFULL) << 56) | ((x & 0xFF00ULL) << 40) | ((x & 0xFF0000ULL) << 24) | ((x & 0xFF000000ULL) << 8)
         | ((x & 0xFF00000000ULL) >> 8) | ((x & 0xFF0000000000ULL) >> 24) | ((x & 0xFF000000000000ULL) >> 40) | ((x & 0xFF00000000000000ULL) >> 56);
#else
    return x;
#endif
}
}

namespace Engine {
namespace Editors {
namespace CameraSystem {

// ============================================================================
// Packet structures
// ============================================================================

#pragma pack(push, 1)

// LEGACY UDP profile: 16-byte column header, pixel data (12 bytes per pixel), then 4-byte column footer (SDK col_footer_size).
// Column advancement = 16 + beamCount*12 + 4 per SDK parsing.cpp.
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
    return Engine::Editors::CameraSystem::IsOusterVerbose() && (g_packetCount <= 5 || g_packetCount % 100 == 0);
}

static bool shouldLogDebug() {
    return Engine::Editors::CameraSystem::IsOusterVerbose() && (++g_debugCount < 50);
}

#ifdef OUSTER_SDK_AVAILABLE
// Build m_currentFrame from manual range/signal images using SDK XYZLut (same math as SDK path).
static void BuildCurrentFrameFromRangeImage(OusterStream* stream) {
    std::vector<uint32_t>& rangeBuf = stream->GetManualRangeImage();
    std::vector<uint16_t>& signalBuf = stream->GetManualSignalImage();
    const int w = stream->GetColumnsPerFrame();
    const int h = stream->GetBeamCount();
    const size_t expectedSize = static_cast<size_t>(w) * static_cast<size_t>(h);
    if (rangeBuf.size() != expectedSize) return;

    std::shared_ptr<void> rawLut = stream->GetXyzLutPtr();
    auto xyzlutPtr = std::static_pointer_cast<XYZLut>(rawLut);
    if (!xyzlutPtr) return;

    // SDK xyzlut uses i = row*w+col (row-major). We fill rangeBuf with idx = beam*w+col.
    // img_t in Ouster SDK is RowMajor; use it so range.data()[i] matches LUT order.
    Eigen::Map<const img_t<uint32_t>> rangeMap(rangeBuf.data(), h, w);
    PointCloudXYZd xyz = (*xyzlutPtr)(rangeMap);

    const OusterConfig& config = stream->GetConfig();
    std::vector<PointCloudPoint>& frame = stream->GetCurrentFrame();
    frame.clear();
    frame.reserve(expectedSize / 4u);

    for (int col = 0; col < w; col++) {
        for (int row = 0; row < h; row++) {
            const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(w) + static_cast<size_t>(col);
            const uint32_t rangeCounts = rangeBuf[idx];
            if (rangeCounts == 0) continue;
            const double rangeM = static_cast<double>(rangeCounts) * 0.001;
            if (rangeM < static_cast<double>(config.minRange) || rangeM > static_cast<double>(config.maxRange)) continue;
            if (config.downsampleFactor > 1 && (row * w + col) % config.downsampleFactor != 0) continue;

            const double x = xyz(idx, 0), y = xyz(idx, 1), z = xyz(idx, 2);
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

            const float intensity = (idx < signalBuf.size())
                ? std::max(static_cast<float>(signalBuf[idx]) / 65535.0f, 0.1f) : 0.0f;
            frame.push_back(stream->ConvertPoint(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
                intensity, static_cast<float>(rangeM)));
        }
    }
}
#endif

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
    // LEGACY: 4-byte column footer per SDK (col_footer_size); non-LEGACY uses 0
    const size_t columnFooterSize = (pixelStride == 12u && blockHdrSize == 0u) ? 4u : 0u;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize + columnFooterSize;
    
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

#ifdef OUSTER_SDK_AVAILABLE
    const bool useXyzLut = (stream->GetXyzLutPtr() != nullptr);
#else
    const bool useXyzLut = false;
#endif

    // One-time path log: SDK XYZ LUT vs manual beam-angle
    {
        static std::atomic<int> g_pathLogged{0};
        if (g_pathLogged++ == 0) {
            if (useXyzLut)
                LOG_INFO("OusterStream", "Ouster: using SDK XYZ LUT path (manual packets)");
            else
                LOG_INFO("OusterStream", "Ouster: using manual beam-angle path (no SDK LUT)");
        }
    }

    // One scan = one full rotation = exactly ExpectedPacketsPerScan() packets (e.g. 64 for 1024 cols / 16 per packet).
    // Do NOT flush on frame_id change: some sensors increment frame_id more often (e.g. every 16 packets), which would
    // produce 4 partial scans (4 rotated overlays). Only flush when we have a full set of packets for one 360° scan.
    if (useXyzLut && stream->GetPacketsInCurrentScan() == 0) {
        stream->ClearManualRangeImage();
    }
    
    size_t offset = 0;
    int totalColumns = 0;
    int totalPointsProcessed = 0;
    const OusterConfig& config = stream->GetConfig();
    const int w = stream->GetColumnsPerFrame();
    
    while (offset + columnSize <= size && totalColumns < stream->GetColumnsPerPacket()) {
        const uint8_t* colBase = data + offset;
        // Ouster UDP packet format: LITTLE ENDIAN (per sensor docs "Words are 32 bits, little endian")
        uint64_t timestamp = static_cast<uint64_t>(colBase[0]) | (static_cast<uint64_t>(colBase[1]) << 8)
            | (static_cast<uint64_t>(colBase[2]) << 16) | (static_cast<uint64_t>(colBase[3]) << 24)
            | (static_cast<uint64_t>(colBase[4]) << 32) | (static_cast<uint64_t>(colBase[5]) << 40)
            | (static_cast<uint64_t>(colBase[6]) << 48) | (static_cast<uint64_t>(colBase[7]) << 56);
        uint16_t measurementId = static_cast<uint16_t>(colBase[8]) | (static_cast<uint16_t>(colBase[9]) << 8);
        uint16_t frameId = static_cast<uint16_t>(colBase[10]) | (static_cast<uint16_t>(colBase[11]) << 8);
        uint32_t encoderCount = static_cast<uint32_t>(colBase[12]) | (static_cast<uint32_t>(colBase[13]) << 8)
            | (static_cast<uint32_t>(colBase[14]) << 16) | (static_cast<uint32_t>(colBase[15]) << 24);
        encoderCount %= static_cast<uint32_t>(stream->GetEncoderTicksPerRev());
        
        if (timestamp == 0) {
            offset += columnSize;
            continue;
        }
        
        totalColumns++;
        
        // Log first few column headers for debugging
        if (shouldLogDebug()) {
            LOG_INFO_FMT("OusterStream", "Col %d: ts=%llu measId=%u frameId=%u encoder=%u", 
                        totalColumns, (unsigned long long)timestamp, measurementId, frameId, encoderCount);
        }
        
        // Track frame_id/timestamp/encoder for flush stats (no flush on frame_id change;
        // we flush only when we have a full scan = ExpectedPacketsPerScan() packets, like the SDK)
        if (!stream->IsFileLoadFrameStarted() || stream->GetCurrentFrame().empty()) {
            stream->SetFileLoadFrameId(frameId);
            stream->SetFileLoadFrameStarted(true);
        }
        stream->SetFileLoadLastTimestamp(timestamp);
        stream->GetLiveLastColumnTimestampNs().store(timestamp);
        if (stream->IsFileLoadEncoderInitialized()) {
            stream->SetFileLoadMinEncoder(std::min(stream->GetFileLoadMinEncoder(), encoderCount));
            stream->SetFileLoadMaxEncoder(std::max(stream->GetFileLoadMaxEncoder(), encoderCount));
        } else {
            stream->SetFileLoadMinEncoder(encoderCount);
            stream->SetFileLoadMaxEncoder(encoderCount);
            stream->SetFileLoadEncoderInitialized(true);
        }
        
        // Process pixels (pixel data ends before column footer)
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        const uint8_t* pixelEnd = colBase + columnHeaderSize + blockHdrSize + columnDataSize;
        
        if (useXyzLut) {
            // LEGACY pixel: 12 bytes, little-endian. RANGE bits 0-19, SIGNAL bits 48-63 (SDK LEGACY_FIELD_INFO)
            for (int beam = 0; beam < stream->GetBeamCount(); beam++) {
                const uint8_t* p = pxBase + beam * pixelStride;
                if (p + pixelStride > pixelEnd) break;
                uint32_t range_counts = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
                    | ((static_cast<uint32_t>(p[2]) & 0x0Fu) << 16);  // 20 bits LE
                uint16_t signal = static_cast<uint16_t>(p[6]) | (static_cast<uint16_t>(p[7]) << 8);  // bytes 6-7 LE
                const int col = std::min(std::max(static_cast<int>(measurementId), 0), w - 1);
                const size_t idx = static_cast<size_t>(beam) * static_cast<size_t>(w) + static_cast<size_t>(col);
                if (idx < stream->GetManualRangeImage().size()) {
                    stream->GetManualRangeImage()[idx] = range_counts;
                    if (idx < stream->GetManualSignalImage().size())
                        stream->GetManualSignalImage()[idx] = signal;
                }
                totalPointsProcessed++;
            }
        } else {
            for (int beam = 0; beam < stream->GetBeamCount(); beam += config.downsampleFactor) {
                const uint8_t* p = pxBase + beam * pixelStride;
                if (p + pixelStride > pixelEnd) break;
                uint32_t range_counts = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
                    | ((static_cast<uint32_t>(p[2]) & 0x0Fu) << 16);  // 20 bits LE
                uint16_t signal = static_cast<uint16_t>(p[6]) | (static_cast<uint16_t>(p[7]) << 8);
                float range = static_cast<float>(range_counts) * stream->GetRangeUnit();
                if (range <= 0.0f || range < config.minRange || range > config.maxRange) continue;
                totalPointsProcessed++;
                stream->GetCurrentFrame().push_back(
                    stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles)
                );
            }
        }
        
        offset += columnSize;
    }
    
    // Flush only when we have a complete 360° scan (ExpectedPacketsPerScan() packets). Never flush on frame_id
    // change, to avoid outputting multiple partial scans that appear as rotated overlays.
    stream->AddPacketsInCurrentScan(1);
    if (stream->GetPacketsInCurrentScan() >= stream->ExpectedPacketsPerScan()) {
#ifdef OUSTER_SDK_AVAILABLE
        if (useXyzLut) {
            BuildCurrentFrameFromRangeImage(stream);
        }
#endif
        stream->FlushCurrentFrame(stream->GetFileLoadLastTimestamp(),
                                 stream->GetFileLoadMinEncoder(),
                                 stream->GetFileLoadMaxEncoder(),
                                 stream->IsFileLoadEncoderInitialized());
        stream->ResetPacketsInCurrentScan();
        // Refresh range image buffer for the next scan so we don't carry over previous scan data
#ifdef OUSTER_SDK_AVAILABLE
        if (useXyzLut) {
            stream->ClearManualRangeImage();
        }
#endif
    }
    
    // Log stats periodically
    if (shouldLogPacket()) {
        LOG_INFO_FMT("OusterStream", "Packet %d: %d columns, %d points, frame %u (%zu pts, %d/%d packets)", 
                     g_packetCount, totalColumns, totalPointsProcessed, 
                     stream->GetFileLoadFrameId(), stream->GetCurrentFrame().size(),
                     stream->GetPacketsInCurrentScan(), stream->ExpectedPacketsPerScan());
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
    const size_t columnFooterSize = (pixelStride == 12u && blockHdrSize == 0u) ? 4u : 0u;
    const size_t columnSize = columnHeaderSize + blockHdrSize + columnDataSize + columnFooterSize;
    
    if (Engine::Editors::CameraSystem::IsOusterVerbose()) {
        static bool configLogged = false;
        if (!configLogged) {
            LOG_INFO_FMT("OusterStream", "Packet processing config: beamCount=%d, pixelStride=%zu, columnsPerPacket=%d",
                         stream->GetBeamCount(), pixelStride, stream->GetColumnsPerPacket());
            configLogged = true;
        }
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
    
#ifdef OUSTER_SDK_AVAILABLE
    const bool useXyzLut = (stream->GetXyzLutPtr() != nullptr);
#else
    const bool useXyzLut = false;
#endif
    const int w = stream->GetColumnsPerFrame();
    
    // One scan = ExpectedPacketsPerScan() packets. Do NOT flush on frame_id (sensor often increments every 16 packets = 4 overlays).
    if (stream->GetPacketsInCurrentScan() == 0) {
        if (useXyzLut) stream->ClearManualRangeImage();
        else stream->GetCurrentFrame().clear();
    }
    
    size_t offset = 0;
    int totalColumns = 0;
    const OusterConfig& config = stream->GetConfig();
    
    while (offset + columnSize <= size && totalColumns < stream->GetColumnsPerPacket()) {
        const uint8_t* colBase = data + offset;
        // Ouster packet: LITTLE ENDIAN
        uint64_t timestamp = static_cast<uint64_t>(colBase[0]) | (static_cast<uint64_t>(colBase[1]) << 8)
            | (static_cast<uint64_t>(colBase[2]) << 16) | (static_cast<uint64_t>(colBase[3]) << 24)
            | (static_cast<uint64_t>(colBase[4]) << 32) | (static_cast<uint64_t>(colBase[5]) << 40)
            | (static_cast<uint64_t>(colBase[6]) << 48) | (static_cast<uint64_t>(colBase[7]) << 56);
        uint16_t measurementId = static_cast<uint16_t>(colBase[8]) | (static_cast<uint16_t>(colBase[9]) << 8);
        uint16_t frameId = static_cast<uint16_t>(colBase[10]) | (static_cast<uint16_t>(colBase[11]) << 8);
        uint32_t encoderCount = static_cast<uint32_t>(colBase[12]) | (static_cast<uint32_t>(colBase[13]) << 8)
            | (static_cast<uint32_t>(colBase[14]) << 16) | (static_cast<uint32_t>(colBase[15]) << 24);
        encoderCount %= static_cast<uint32_t>(stream->GetEncoderTicksPerRev());
        
        if (timestamp == 0) {
            offset += columnSize;
            continue;
        }
        
        totalColumns++;
        stream->SetFileLoadLastTimestamp(timestamp);
        if (!stream->IsFileLoadFrameStarted()) {
            stream->SetFileLoadFrameId(frameId);
            stream->SetFileLoadFrameStarted(true);
        }
        if (stream->IsFileLoadEncoderInitialized()) {
            stream->SetFileLoadMinEncoder(std::min(stream->GetFileLoadMinEncoder(), encoderCount));
            stream->SetFileLoadMaxEncoder(std::max(stream->GetFileLoadMaxEncoder(), encoderCount));
        } else {
            stream->SetFileLoadMinEncoder(encoderCount);
            stream->SetFileLoadMaxEncoder(encoderCount);
            stream->SetFileLoadEncoderInitialized(true);
        }
        
        const uint8_t* pxBase = colBase + columnHeaderSize + blockHdrSize;
        const uint8_t* pixelEnd = colBase + columnHeaderSize + blockHdrSize + columnDataSize;
        
        if (useXyzLut) {
            for (int beam = 0; beam < stream->GetBeamCount(); beam++) {
                const uint8_t* p = pxBase + beam * pixelStride;
                if (p + pixelStride > pixelEnd) break;
                uint32_t range_counts = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
                    | ((static_cast<uint32_t>(p[2]) & 0x0Fu) << 16);
                uint16_t signal = static_cast<uint16_t>(p[6]) | (static_cast<uint16_t>(p[7]) << 8);
                const int col = std::min(std::max(static_cast<int>(measurementId), 0), w - 1);
                const size_t idx = static_cast<size_t>(beam) * static_cast<size_t>(w) + static_cast<size_t>(col);
                if (idx < stream->GetManualRangeImage().size()) {
                    stream->GetManualRangeImage()[idx] = range_counts;
                    if (idx < stream->GetManualSignalImage().size())
                        stream->GetManualSignalImage()[idx] = signal;
                }
            }
        } else {
            for (int beam = 0; beam < stream->GetBeamCount(); beam += config.downsampleFactor) {
                const uint8_t* p = pxBase + beam * pixelStride;
                if (p + pixelStride > pixelEnd) break;
                uint32_t range_counts = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
                    | ((static_cast<uint32_t>(p[2]) & 0x0Fu) << 16);
                uint16_t signal = static_cast<uint16_t>(p[6]) | (static_cast<uint16_t>(p[7]) << 8);
                float range = static_cast<float>(range_counts) * stream->GetRangeUnit();
                if (range <= 0.0f || range < config.minRange || range > config.maxRange) continue;
                stream->GetCurrentFrame().push_back(
                    stream->CalculatePoint3D(measurementId, beam, range, signal, beamAltitudeAngles, beamAzimuthAngles)
                );
            }
        }
        
        offset += columnSize;
    }
    
    // Flush only when we have a full 360° scan (ExpectedPacketsPerScan() packets), not on frame_id
    stream->AddPacketsInCurrentScan(1);
    if (stream->GetPacketsInCurrentScan() >= stream->ExpectedPacketsPerScan()) {
#ifdef OUSTER_SDK_AVAILABLE
        if (useXyzLut) BuildCurrentFrameFromRangeImage(stream);
#endif
        if (!stream->GetCurrentFrame().empty()) {
            // Replace (don't append) so we display only this scan - avoids overlaying many scans
            stream->GetLoadedPacketPoints() = stream->GetCurrentFrame();
        }
        stream->ResetPacketsInCurrentScan();
        if (useXyzLut) stream->ClearManualRangeImage();
        else stream->GetCurrentFrame().clear();
    }
}

// ============================================================================
// LoadPcapFromFileManual - Manual PCAP loading (fallback when SDK unavailable)
// ============================================================================

bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
#ifdef __APPLE__
    if (Engine::Editors::CameraSystem::IsOusterVerbose()) {
        LOG_INFO_FMT("OusterStream", "Loading PCAP manually: %s", pcapFile.c_str());
        LOG_WARNING("OusterStream", "LoadPcapFromFileManual: SDK parsing is recommended when available");
    }
    
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
    stream->ResetPacketsInCurrentScan();
    
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
    
    // Flush any remaining partial scan
#ifdef OUSTER_SDK_AVAILABLE
    if (stream->GetXyzLutPtr() != nullptr && stream->GetPacketsInCurrentScan() > 0) {
        BuildCurrentFrameFromRangeImage(stream);
    }
#endif
    if (!stream->GetCurrentFrame().empty()) {
        stream->GetLoadedPacketPoints() = stream->GetCurrentFrame();
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
