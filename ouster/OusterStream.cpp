// ============================================================================
// OusterStream.cpp - Main Ouster LiDAR streaming implementation
// ============================================================================

#include "OusterStream.h"
#include "OusterStreamCommon.h"
#include "platform/Logger.h"
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

// Ouster C++ SDK includes (if available)
#ifdef OUSTER_SDK_AVAILABLE
#include "ouster/pcap_scan_source.h"
#include "ouster/lidar_scan.h"
#include "ouster/xyzlut.h"
#include "ouster/cartesian.h"
#include "ouster/field.h"
#include "ouster/types.h"

using namespace ouster::sdk;
using namespace ouster::sdk::core;
using namespace ouster::sdk::pcap;

#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"
#include "ouster/sensor_packet_source.h"
using namespace ouster::sdk::sensor;
#define OUSTER_SENSOR_AVAILABLE 1
#else
#define OUSTER_SENSOR_AVAILABLE 0
#endif
#endif  // OUSTER_SDK_AVAILABLE

// Forward declarations for helper modules
namespace Engine {
namespace Editors {
namespace CameraSystem {

namespace OusterFileLoader {
    bool LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile);
    bool LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
    bool LoadPacketFromFile(OusterStream* stream, const std::string& filename);
    bool ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile);
    void ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size);
}

namespace OusterLiveStreaming {
    bool Connect(OusterStream* stream);
    void Disconnect(OusterStream* stream);
    void StartStreaming(OusterStream* stream);
    void StopStreaming(OusterStream* stream);
    void StreamingThreadImpl(OusterStream* stream);
    void ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size);
}

}}}

namespace Engine {
namespace Editors {
namespace CameraSystem {

// ============================================================================
// Constructor / Destructor
// ============================================================================

OusterStream::OusterStream() {
    m_currentFrame.reserve(m_beamCount * m_columnsPerFrame);
    m_udpProfileLidar = "LEGACY";
    m_columnsPerPacket = 16;
    m_packetHeaderSize = 0;
    m_measurementBlockHeaderSize = 0;  // LEGACY: no block header; 4 bytes are column footer at end
    m_pixelStride = 12;
    m_encoderTicksPerRev = 90112;
    m_rangeUnit = 0.001f;
    
    // Initialize transforms to identity
    std::memset(m_beamToLidarTransform, 0, 16 * sizeof(float));
    m_beamToLidarTransform[0] = 1.0f;
    m_beamToLidarTransform[5] = 1.0f;
    m_beamToLidarTransform[10] = 1.0f;
    m_beamToLidarTransform[15] = 1.0f;
    
    std::memset(m_lidarToSensorTransform, 0, 16 * sizeof(float));
    m_lidarToSensorTransform[0] = 1.0f;
    m_lidarToSensorTransform[5] = 1.0f;
    m_lidarToSensorTransform[10] = 1.0f;
    m_lidarToSensorTransform[15] = 1.0f;
}

OusterStream::~OusterStream() {
    StopStreaming();
    Disconnect();
}

// ============================================================================
// Configuration
// ============================================================================

void OusterStream::SetConfig(const OusterConfig& config) {
    m_config = config;
}

// ============================================================================
// Connection / Streaming delegation
// ============================================================================

bool OusterStream::Connect() {
    return OusterLiveStreaming::Connect(this);
}

void OusterStream::Disconnect() {
    OusterLiveStreaming::Disconnect(this);
}

void OusterStream::StartStreaming() {
    OusterLiveStreaming::StartStreaming(this);
}

void OusterStream::StopStreaming() {
    OusterLiveStreaming::StopStreaming(this);
}

void OusterStream::StreamingThread() {
    OusterLiveStreaming::StreamingThreadImpl(this);
}

// ============================================================================
// File Loading delegation
// ============================================================================

bool OusterStream::LoadMetadataFromFile(const std::string& metadataFile) {
    return OusterFileLoader::LoadMetadataFromFile(this, metadataFile);
}

bool OusterStream::LoadPacketFromFile(const std::string& filename) {
    return OusterFileLoader::LoadPacketFromFile(this, filename);
}

bool OusterStream::LoadPcapFromFile(const std::string& pcapFile, const std::string& metadataFile) {
    return OusterFileLoader::LoadPcapFromFile(this, pcapFile, metadataFile);
}

bool OusterStream::ParseMetadataFile(const std::string& metadataFile) {
    return OusterFileLoader::ParseMetadataFileImpl(this, metadataFile);
}

void OusterStream::ProcessLidarPacketFromFile(const uint8_t* data, size_t size) {
    OusterFileLoader::ProcessLidarPacketFromFileImpl(this, data, size);
}

void OusterStream::ProcessLidarPacket(const uint8_t* data, size_t size) {
    OusterLiveStreaming::ProcessLidarPacketImpl(this, data, size);
}

// ============================================================================
// State Management
// ============================================================================

void OusterStream::ClearFileLoadedState() {
    m_packetLoadedFromFile = false;
    m_loadedPacketPoints.clear();
    m_loadedPacketTimestamp = 0.0;
    
    {
        std::lock_guard<std::mutex> lock(m_pointCloudMutex);
        m_accumulatedFrames.clear();
        m_accumulatedPointCloud.clear();
        m_accumulationDirty = false;
        while (!m_pointCloudQueue.empty()) {
            m_pointCloudQueue.pop();
        }
    }
    
    LOG_INFO("OusterStream", "Cleared file-loaded state and buffers");
}

// ============================================================================
// Point Cloud Processing
// ============================================================================

PointCloudPoint OusterStream::CalculatePoint3D(uint16_t measurementId, int beam, float range, uint16_t signal,
                                                const std::vector<float>& beamAltitudeAngles,
                                                const std::vector<float>& beamAzimuthAngles) {
    // Match SDK xyzlut: encoder from column, azimuth/altitude from beam angles (degrees -> radians)
    int columnIndex = static_cast<int>(measurementId);
    int shift = (!m_pixelShiftByRow.empty() && beam < static_cast<int>(m_pixelShiftByRow.size()))
                ? m_pixelShiftByRow[beam] : 0;
    int correctedColumn = (columnIndex + shift + m_columnsPerFrame) % m_columnsPerFrame;
    float theta_encoder = 2.0f * static_cast<float>(M_PI) - (static_cast<float>(correctedColumn) * 2.0f * static_cast<float>(M_PI) / static_cast<float>(m_columnsPerFrame));
    float theta_azimuth = -(beamAzimuthAngles[beam] * static_cast<float>(M_PI) / 180.0f);
    float altitude_rad = beamAltitudeAngles[beam] * static_cast<float>(M_PI) / 180.0f;
    // Unit direction (same as SDK direction columns before range_unit scale)
    float dir_x = cosf(theta_encoder + theta_azimuth) * cosf(altitude_rad);
    float dir_y = sinf(theta_encoder + theta_azimuth) * cosf(altitude_rad);
    float dir_z = sinf(altitude_rad);
    // Beam origin offset (SDK: beam_to_lidar (0,3) and (2,3) in mm, D = euclidean distance)
    float Bx = m_beamToLidarTransform[3];
    float Bz = m_beamToLidarTransform[11];
    float D = sqrtf(Bx * Bx + Bz * Bz);
    float off_x = cosf(theta_encoder) * Bx - dir_x * D;
    float off_y = sinf(theta_encoder) * Bx - dir_y * D;
    float off_z = -dir_z * D + Bz;
    // point_lidar in mm = range_m * 1000 * direction + offset_mm (SDK uses range_counts * range_unit * dir + offset*range_unit -> meters)
    float range_mm = range * 1000.0f;
    float x_lidar = range_mm * dir_x + off_x;
    float y_lidar = range_mm * dir_y + off_y;
    float z_lidar = range_mm * dir_z + off_z;
    // Apply lidar-to-sensor transform (lidar frame in mm)
    float x_sensor_mm = m_lidarToSensorTransform[0] * x_lidar + m_lidarToSensorTransform[1] * y_lidar + m_lidarToSensorTransform[2] * z_lidar + m_lidarToSensorTransform[3];
    float y_sensor_mm = m_lidarToSensorTransform[4] * x_lidar + m_lidarToSensorTransform[5] * y_lidar + m_lidarToSensorTransform[6] * z_lidar + m_lidarToSensorTransform[7];
    float z_sensor_mm = m_lidarToSensorTransform[8] * x_lidar + m_lidarToSensorTransform[9] * y_lidar + m_lidarToSensorTransform[10] * z_lidar + m_lidarToSensorTransform[11];
    
    // Output in meters to match Python/SDK (setup_ouster.py and xyzlut use meters; browser receives meters)
    float x = x_sensor_mm / 1000.0f;
    float y = y_sensor_mm / 1000.0f;
    float z = z_sensor_mm / 1000.0f;
    
    float intensity = std::max(static_cast<float>(signal) / 65535.0f, 0.1f);
    
    return ConvertPoint(x, y, z, intensity, range);
}

PointCloudPoint OusterStream::ConvertPoint(float x, float y, float z, float intensity, float range) {
    PointCloudPoint point;
    
    glm::vec4 pos(x, y, z, 1.0f);
    pos = m_config.sensorToWorld * pos;
    point.position = glm::vec3(pos);
    point.color = MapToColor(intensity, range);
    point.weight = 1.0f;
    point.segmentId = -1;
    point.cameraId = m_config.cameraId;
    
    return point;
}

glm::vec3 OusterStream::MapToColor(float intensity, float range) {
    if (m_config.useIntensityAsColor) {
        float v = std::clamp(intensity, 0.0f, 1.0f);
        return glm::vec3(v, v, v);
    } else if (m_config.useRangeAsColor) {
        float t = std::clamp((range - m_config.minRange) / (m_config.maxRange - m_config.minRange), 0.0f, 1.0f);
        return glm::vec3(t, 0.2f, 1.0f - t);
    } else {
        return glm::vec3(0.7f, 0.7f, 0.7f);
    }
}

void OusterStream::FlushCurrentFrame(uint64_t lastTimestamp, uint32_t minEncoder, uint32_t maxEncoder, bool encoderInitialized) {
    if (m_currentFrame.empty()) return;
    
    double timestamp = static_cast<double>(lastTimestamp) / 1e9;
    m_lastTimestamp = timestamp;
    m_frameCount++;
    
    if (encoderInitialized) {
        uint32_t encoderSpan = (maxEncoder - minEncoder) % m_encoderTicksPerRev;
        LOG_INFO_FMT("OusterStream", "Frame %d: %zu points, encoder span: %u/%u (%.1f%%)", 
                     m_frameCount.load(), m_currentFrame.size(), 
                     encoderSpan, m_encoderTicksPerRev, (encoderSpan * 100.0f / static_cast<float>(m_encoderTicksPerRev)));
    } else {
        LOG_INFO_FMT("OusterStream", "Frame %d: %zu points", m_frameCount.load(), m_currentFrame.size());
    }
    
    if (m_pointCloudCallback) {
        m_pointCloudCallback(m_currentFrame, timestamp);
    }
    
    {
        std::lock_guard<std::mutex> lock(m_pointCloudMutex);
        if (m_config.enableAccumulation && m_config.accumulateFrames > 0) {
            // Accumulate frames (like web view): append this frame, keep last N frames
            m_accumulatedFrames.push_back(std::move(m_currentFrame));
            while (m_accumulatedFrames.size() > static_cast<size_t>(m_config.accumulateFrames)) {
                m_accumulatedFrames.pop_front();
            }
            m_accumulationDirty = true;
        } else {
            if (m_pointCloudQueue.size() >= MAX_QUEUE_SIZE) {
                m_pointCloudQueue.pop();
                m_droppedFrames++;
            }
            m_pointCloudQueue.push({m_currentFrame, timestamp});
        }
    }
    
    m_currentFrame.clear();
}

// ============================================================================
// Point Cloud Polling
// ============================================================================

bool OusterStream::PollPointCloud(std::vector<PointCloudPoint>& outPoints, double& outTimestamp) {
    // Priority 1: File-loaded data
    if (m_packetLoadedFromFile && !m_loadedPacketPoints.empty()) {
        outPoints = m_loadedPacketPoints;
        outTimestamp = m_loadedPacketTimestamp;
        return true;
    }
    
    std::lock_guard<std::mutex> lock(m_pointCloudMutex);
    
    // Priority 2: Accumulated frames
    if (m_config.enableAccumulation && !m_accumulatedFrames.empty()) {
        if (m_accumulationDirty) {
            m_accumulatedPointCloud.clear();
            size_t totalSize = 0;
            for (const auto& frame : m_accumulatedFrames) {
                totalSize += frame.size();
            }
            m_accumulatedPointCloud.reserve(totalSize);
            
            for (const auto& frame : m_accumulatedFrames) {
                m_accumulatedPointCloud.insert(m_accumulatedPointCloud.end(), frame.begin(), frame.end());
            }
            m_accumulationDirty = false;
            m_lastAccumulationRebuildTime = std::chrono::steady_clock::now();
        }
        
        if (!m_accumulatedPointCloud.empty()) {
            outPoints = m_accumulatedPointCloud;
            outTimestamp = m_lastTimestamp;
            
            static int pollCount = 0;
            pollCount++;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_lastAccumulationRebuildTime).count();
            
            static bool warnedStale = false;
            if (elapsed >= 3 && !warnedStale) {
                LOG_WARNING_FMT("OusterStream", "Accumulated point cloud stale (%lld sec)", static_cast<long long>(elapsed));
                warnedStale = true;
            } else if (elapsed < 3) {
                warnedStale = false;
            }
            
            if (pollCount % 30 == 0) {
                LOG_INFO_FMT("OusterStream", "Poll: %zu accumulated points from %zu frames", outPoints.size(), m_accumulatedFrames.size());
            }
            return true;
        }
    }
    
    // Priority 3: Latest scan from queue
    static int pollCount = 0;
    pollCount++;
    
    if (m_pointCloudQueue.empty()) {
        if (pollCount % 60 == 0) {
            LOG_INFO_FMT("OusterStream", "Poll #%d: queue empty", pollCount);
        }
        return false;
    }
    
    // Drain queue and keep only latest
    size_t skippedScans = 0;
    while (m_pointCloudQueue.size() > 1) {
        m_pointCloudQueue.pop();
        skippedScans++;
    }
    
    auto& latest = m_pointCloudQueue.front();
    outPoints = std::move(latest.first);
    outTimestamp = latest.second;
    m_pointCloudQueue.pop();
    
    if (pollCount <= 5 || pollCount % 30 == 0 || skippedScans > 0) {
        LOG_INFO_FMT("OusterStream", "Poll #%d: %zu points (skipped %zu)", pollCount, outPoints.size(), skippedScans);
    }
    return true;
}

// ============================================================================
// Factory function
// ============================================================================

std::unique_ptr<OusterStream> CreateOusterStream(const std::string& hostname) {
    auto stream = std::make_unique<OusterStream>();
    OusterConfig config;
    config.hostname = hostname;
    stream->SetConfig(config);
    return stream;
}

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
