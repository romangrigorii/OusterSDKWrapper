// ============================================================================
// OusterStream.cpp - Main Ouster LiDAR streaming implementation
// ============================================================================

#include "OusterStream.h"
#include "OusterStreamCommon.h"
#include "OusterLogging.h"
#include "platform/Logger.h"
#if defined(_WIN32)
// Avoid collisions between Windows headers (CHAR/UINT32) and Ouster SDK deprecated chanfield enums.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#endif
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
#include "ouster/lidar_scan.h"
#include "ouster/xyzlut.h"
#include "ouster/cartesian.h"
#include "ouster/field.h"
#include "ouster/types.h"

using namespace ouster::sdk;
namespace ouster_core = ouster::sdk::core;
#if !defined(_WIN32)
#include "ouster/pcap_scan_source.h"
namespace ouster_pcap = ouster::sdk::pcap;
#endif

#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"
#include "ouster/sensor_packet_source.h"
namespace ouster_sensor = ouster::sdk::sensor;
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
    m_packetsInCurrentScan = 0;
    
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
    // Ouster reference: xyz_proj_beam_to_sensor_transform (ouster.sdk.examples.reference)
    // and Sensor Data docs: theta_encoder = 2*pi*(1 - v/w), theta_azimuth = -2*pi*beam_azimuth/360, phi = 2*pi*beam_altitude/360
    // Lidar frame: x = (r-n)*cos(θe+θa)*cos(φ) + Bx*cos(θe), y = (r-n)*sin(θe+θa)*cos(φ) + Bx*sin(θe), z = (r-n)*sin(φ) + Bz
    // n = distance lidar origin to beam origin = sqrt(beam_to_lidar[0,3]^2 + beam_to_lidar[2,3]^2). r, Bx, Bz in mm.
    int v = static_cast<int>(measurementId) % m_columnsPerFrame;
    if (v < 0) v += m_columnsPerFrame;
    int u = beam;
    float theta_encoder = 2.0f * static_cast<float>(M_PI) * (1.0f - static_cast<float>(v) / static_cast<float>(m_columnsPerFrame));
    float theta_azimuth = -2.0f * static_cast<float>(M_PI) * (beamAzimuthAngles[u] / 360.0f);
    float phi = 2.0f * static_cast<float>(M_PI) * (beamAltitudeAngles[u] / 360.0f);
    float Bx_mm = m_beamToLidarTransform[3];
    float Bz_mm = m_beamToLidarTransform[11];
    float n_mm = sqrtf(Bx_mm * Bx_mm + Bz_mm * Bz_mm);
    float r_mm = range * 1000.0f;  // range from sensor is in mm (RANGE field)
    float r_minus_n = r_mm - n_mm;
    // Lidar frame (mm), then transform to sensor frame, then mm -> m
    float x_lidar_mm = r_minus_n * cosf(theta_encoder + theta_azimuth) * cosf(phi) + Bx_mm * cosf(theta_encoder);
    float y_lidar_mm = r_minus_n * sinf(theta_encoder + theta_azimuth) * cosf(phi) + Bx_mm * sinf(theta_encoder);
    float z_lidar_mm = r_minus_n * sinf(phi) + Bz_mm;
    // lidar_to_sensor: row-major 4x4, point_sensor = T * point_lidar (column). Doc: "homogeneous transformation matrix in row-major ordering"
    float x_sensor_mm = m_lidarToSensorTransform[0] * x_lidar_mm + m_lidarToSensorTransform[1] * y_lidar_mm + m_lidarToSensorTransform[2] * z_lidar_mm + m_lidarToSensorTransform[3];
    float y_sensor_mm = m_lidarToSensorTransform[4] * x_lidar_mm + m_lidarToSensorTransform[5] * y_lidar_mm + m_lidarToSensorTransform[6] * z_lidar_mm + m_lidarToSensorTransform[7];
    float z_sensor_mm = m_lidarToSensorTransform[8] * x_lidar_mm + m_lidarToSensorTransform[9] * y_lidar_mm + m_lidarToSensorTransform[10] * z_lidar_mm + m_lidarToSensorTransform[11];
    float x = x_sensor_mm * 0.001f;
    float y = y_sensor_mm * 0.001f;
    float z = z_sensor_mm * 0.001f;
    
    float intensity = std::max(static_cast<float>(signal) / 65535.0f, 0.1f);
    
    return ConvertPoint(x, y, z, intensity, range);
}

PointCloudPoint OusterStream::ConvertPoint(float x, float y, float z, float intensity, float range) {
    PointCloudPoint point;
    
    glm::vec4 pos(x, y, z, 1.0f);
    pos = m_config.sensorToWorld * pos;
    point.position = glm::vec3(pos);
    point.sensorOriginWorld = glm::vec3(m_config.sensorToWorld[3]);
    point.measuredRangeM = glm::length(point.position - point.sensorOriginWorld);
    point.color = MapToColor(intensity, range);
    point.weight = 1.0f;
    point.sampleCount = 1;
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

void OusterStream::ClearManualRangeImage() {
    const size_t n = static_cast<size_t>(m_beamCount) * static_cast<size_t>(m_columnsPerFrame);
    m_manualRangeImage.resize(n, 0);
    m_manualSignalImage.resize(n, 0);
}

// One Ouster "scan" = one full rotation = one complete frame (all columns). We flush when we have a full scan.
// Display buffer: if enableAccumulation we keep last accumulateFrames scans (rolling merge); else we push
// this scan to the queue and PollPointCloud returns the latest scan only (refresh per scan).
void OusterStream::FlushCurrentFrame(uint64_t lastTimestamp, uint32_t minEncoder, uint32_t maxEncoder, bool encoderInitialized) {
    if (m_currentFrame.empty()) return;
    
    double timestamp = static_cast<double>(lastTimestamp) / 1e9;
    m_lastTimestamp = timestamp;
    m_frameCount++;
    
    const bool verbose = Engine::Editors::CameraSystem::IsOusterVerbose();
    if (verbose) {
        if (encoderInitialized) {
            uint32_t encoderSpan = (maxEncoder - minEncoder) % m_encoderTicksPerRev;
            LOG_INFO_FMT("OusterStream", "Frame %d: %zu points, encoder span: %u/%u (%.1f%%)",
                         m_frameCount.load(), m_currentFrame.size(),
                         encoderSpan, m_encoderTicksPerRev, (encoderSpan * 100.0f / static_cast<float>(m_encoderTicksPerRev)));
        } else {
            LOG_INFO_FMT("OusterStream", "Frame %d: %zu points", m_frameCount.load(), m_currentFrame.size());
        }
    } else if (m_frameCount % 60 == 0) {
        LOG_INFO_FMT("OusterStream", "Ouster: frame %d, %zu points", m_frameCount.load(), m_currentFrame.size());
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
            
            if (Engine::Editors::CameraSystem::IsOusterVerbose() && pollCount % 30 == 0) {
                LOG_INFO_FMT("OusterStream", "Poll: %zu accumulated points from %zu frames", outPoints.size(), m_accumulatedFrames.size());
            }
            return true;
        }
    }
    
    // Priority 3: Latest scan from queue
    static int pollCount = 0;
    pollCount++;
    
    if (m_pointCloudQueue.empty()) {
        if (Engine::Editors::CameraSystem::IsOusterVerbose() && pollCount % 60 == 0) {
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
    
    if (Engine::Editors::CameraSystem::IsOusterVerbose() && (pollCount <= 5 || pollCount % 30 == 0 || skippedScans > 0)) {
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
