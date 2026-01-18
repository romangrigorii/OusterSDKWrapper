#include "OusterStream.h"
#include "OusterStreamUtilities.h"
#include "OusterStreamFileLoader.h"
#include "OusterStreamLiveStreaming.h"
#include "../../engine/platform/Logger.h"
#include <cmath>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <cstdio>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <csignal>
#include <unistd.h>
#include <fstream>
#include <sstream>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// Ouster C++ SDK includes (if available)
#ifdef OUSTER_SDK_AVAILABLE
#include "ouster/pcap_scan_source.h"
#include "ouster/lidar_scan.h"
#include "ouster/xyzlut.h"
#include "ouster/cartesian.h"  // For cartesian() function
#include "ouster/field.h"
#include "ouster/types.h"  // Contains SensorInfo definition, RANGE_UNIT, and parse_metadata

using namespace ouster::sdk;
using namespace ouster::sdk::core;
using namespace ouster::sdk::pcap;

// Include sensor headers if BUILD_SENSOR is defined (set by CMakeLists.txt)
#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"  // For live streaming
#include "ouster/sensor_packet_source.h"  // For live packet streaming
using namespace ouster::sdk::sensor;
#define OUSTER_SENSOR_AVAILABLE 1
#else
#define OUSTER_SENSOR_AVAILABLE 0
#endif
#endif  // OUSTER_SDK_AVAILABLE

using EngineLogging::Logger;

#ifdef __APPLE__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#endif

namespace Engine {
namespace Editors {
namespace CameraSystem {

    OusterStream::OusterStream() {
    m_currentFrame.reserve(m_beamCount * m_columnsPerFrame);
    m_udpProfileLidar = "LEGACY";
    m_columnsPerPacket = 16;
    m_packetHeaderSize = 0;
    m_measurementBlockHeaderSize = 4;
    m_pixelStride = 12;
    m_encoderTicksPerRev = 90112;
    m_rangeUnit = 0.001f;
    
    std::memset(m_beamToLidarTransform, 0, 16 * sizeof(float));
    m_beamToLidarTransform[0 * 4 + 0] = 1.0f;
    m_beamToLidarTransform[1 * 4 + 1] = 1.0f;
    m_beamToLidarTransform[2 * 4 + 2] = 1.0f;
    m_beamToLidarTransform[3 * 4 + 3] = 1.0f;
    
    std::memset(m_lidarToSensorTransform, 0, 16 * sizeof(float));
    m_lidarToSensorTransform[0 * 4 + 0] = 1.0f;
    m_lidarToSensorTransform[1 * 4 + 1] = 1.0f;
    m_lidarToSensorTransform[2 * 4 + 2] = 1.0f;
    m_lidarToSensorTransform[3 * 4 + 3] = 1.0f;
}

OusterStream::~OusterStream() {
    StopStreaming();
    Disconnect();
}

void OusterStream::SetConfig(const OusterConfig& config) {
    m_config = config;
}

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

void OusterStream::ClearFileLoadedState() {
    m_packetLoadedFromFile = false;
    m_loadedPacketPoints.clear();
    m_loadedPacketTimestamp = 0.0;
    LOG_INFO("OusterStream", "Cleared file-loaded state");
}

bool OusterStream::LoadMetadataFromFile(const std::string& metadataFile) {
    return OusterFileLoader::LoadMetadataFromFile(this, metadataFile);
}

void OusterStream::StreamingThread() {
    OusterLiveStreaming::StreamingThreadImpl(this);
}

PointCloudPoint OusterStream::CalculatePoint3D(uint16_t measurementId, int beam, float range, uint16_t signal,
                                                const std::vector<float>& beamAltitudeAngles,
                                                const std::vector<float>& beamAzimuthAngles) {
    int columnIndex = static_cast<int>(measurementId);
    int shift = (!m_pixelShiftByRow.empty() && beam < static_cast<int>(m_pixelShiftByRow.size())) 
                ? m_pixelShiftByRow[beam] : 0;
    int correctedColumn = (columnIndex + shift + m_columnsPerFrame) % m_columnsPerFrame;
    
    float theta_encoder = 2.0f * M_PI * (1.0f - static_cast<float>(correctedColumn) / static_cast<float>(m_columnsPerFrame));
    float theta_azimuth = -2.0f * M_PI * (beamAzimuthAngles[beam] / 360.0f);
    float phi = 2.0f * M_PI * (beamAltitudeAngles[beam] / 360.0f);
    
    float n = m_lidarOriginToBeamOriginMm;
    float r_mm = range * 1000.0f;
    float x_lidar = (r_mm - n) * cosf(theta_encoder + theta_azimuth) * cosf(phi) + m_beamToLidarTransform[0 * 4 + 3] * cosf(theta_encoder);
    float y_lidar = (r_mm - n) * sinf(theta_encoder + theta_azimuth) * cosf(phi) + m_beamToLidarTransform[0 * 4 + 3] * sinf(theta_encoder);
    float z_lidar = (r_mm - n) * sinf(phi) + m_beamToLidarTransform[2 * 4 + 3];
    
    float x_sensor_mm = m_lidarToSensorTransform[0 * 4 + 0] * x_lidar + m_lidarToSensorTransform[0 * 4 + 1] * y_lidar + m_lidarToSensorTransform[0 * 4 + 2] * z_lidar + m_lidarToSensorTransform[0 * 4 + 3];
    float y_sensor_mm = m_lidarToSensorTransform[1 * 4 + 0] * x_lidar + m_lidarToSensorTransform[1 * 4 + 1] * y_lidar + m_lidarToSensorTransform[1 * 4 + 2] * z_lidar + m_lidarToSensorTransform[1 * 4 + 3];
    float z_sensor_mm = m_lidarToSensorTransform[2 * 4 + 0] * x_lidar + m_lidarToSensorTransform[2 * 4 + 1] * y_lidar + m_lidarToSensorTransform[2 * 4 + 2] * z_lidar + m_lidarToSensorTransform[2 * 4 + 3];
    
    float x = x_sensor_mm * 0.1f;
    float y = y_sensor_mm * 0.1f;
    float z = z_sensor_mm * 0.1f;
    
    float intensity = std::max(static_cast<float>(signal) / 65535.0f, 0.1f);
    
    return ConvertPoint(x, y, z, intensity, range);
}

void OusterStream::FlushCurrentFrame(uint64_t lastTimestamp, uint32_t minEncoder, uint32_t maxEncoder, bool encoderInitialized) {
    if (m_currentFrame.empty()) return;
    
    double timestamp = static_cast<double>(lastTimestamp) / 1e9;
    m_lastTimestamp = timestamp;
    m_frameCount++;
    
    if (encoderInitialized) {
        uint32_t encoderSpan = (maxEncoder - minEncoder) % m_encoderTicksPerRev;
        LOG_INFO_FMT("OusterStream", "Frame %d complete: %zu points, encoder span: %u/%u (%.1f%%)", 
                     m_frameCount.load(), m_currentFrame.size(), 
                     encoderSpan, m_encoderTicksPerRev, (encoderSpan * 100.0f / static_cast<float>(m_encoderTicksPerRev)));
    } else {
        LOG_INFO_FMT("OusterStream", "Frame %d complete with %zu points", 
                     m_frameCount.load(), m_currentFrame.size());
    }
    
    if (m_pointCloudCallback) {
        m_pointCloudCallback(m_currentFrame, timestamp);
    }
    
    {
        std::lock_guard<std::mutex> lock(m_pointCloudMutex);
        if (m_pointCloudQueue.size() >= MAX_QUEUE_SIZE) {
            m_pointCloudQueue.pop();
            m_droppedFrames++;
        }
        m_pointCloudQueue.push({m_currentFrame, timestamp});
    }
    
    m_currentFrame.clear();
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

bool OusterStream::PollPointCloud(std::vector<PointCloudPoint>& outPoints, double& outTimestamp) {
    if (m_packetLoadedFromFile && !m_loadedPacketPoints.empty()) {
        outPoints = m_loadedPacketPoints;
        outTimestamp = m_loadedPacketTimestamp;
        return true;
    }
    
    std::lock_guard<std::mutex> lock(m_pointCloudMutex);
    if (m_pointCloudQueue.empty()) {
        return false;
    }
    
    auto& front = m_pointCloudQueue.front();
    outPoints = std::move(front.first);
    outTimestamp = front.second;
    m_pointCloudQueue.pop();
    return true;
}


bool OusterStream::LoadPacketFromFile(const std::string& filename) {
    return OusterFileLoader::LoadPacketFromFile(this, filename);
}


bool OusterStream::LoadPcapFromFile(const std::string& pcapFile, const std::string& metadataFile) {
    return OusterFileLoader::LoadPcapFromFile(this, pcapFile, metadataFile);
}


void OusterStream::ProcessLidarPacket(const uint8_t* data, size_t size) {
    OusterLiveStreaming::ProcessLidarPacketImpl(this, data, size);
}

bool OusterStream::ParseMetadataFile(const std::string& metadataFile) {
    return OusterFileLoader::ParseMetadataFileImpl(this, metadataFile);
}


void OusterStream::ProcessLidarPacketFromFile(const uint8_t* data, size_t size) {
    OusterFileLoader::ProcessLidarPacketFromFileImpl(this, data, size);
}

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
