#pragma once

// ============================================================================
// OusterStream.h - Main Ouster LiDAR streaming class
// ============================================================================

#include "robotics/remesh/Remeshing.h"  // For PointCloudPoint
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <deque>
#include <chrono>
#include <cstdint>

// Forward declarations for Ouster SDK types
namespace ouster {
namespace sdk {
namespace sensor {
class SensorScanSource;
}
namespace core {
class SensorInfo;
}
}
}

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Forward declare OusterStream class
class OusterStream;

// ============================================================================
// OusterConfig - Sensor configuration
// ============================================================================
struct OusterConfig {
    std::string hostname = "172.17.1.56";  // Ouster sensor IP
    std::string localIp = "172.17.1.100";  // Local IP to bind to
    int lidarPort = 7510;                   // Port sensor sends lidar data to
    int imuPort = 7513;                     // Port sensor sends IMU data to
    
    // Bridge mode for macOS firewall bypass
    bool useBridge = true;
    int bridgePort = 7506;
    
    // Transform (sensor -> world frame)
    glm::mat4 sensorToWorld = glm::mat4(1.0f);
    
    // Point cloud filtering
    float minRange = 0.3f;
    float maxRange = 50.0f;
    
    // Color mapping
    bool useIntensityAsColor = true;
    bool useRangeAsColor = false;
    
    // Downsampling (skip every N points, 1 = no skip)
    int downsampleFactor = 1;
    
    // Camera ID for all points from this sensor
    int32_t cameraId = 0;
    
    // Live streaming: when true, display = merge of last accumulateFrames scans; when false, display = latest scan only (refresh each scan)
    int accumulateFrames = 10;
    bool enableAccumulation = false;
};

using PointCloudCallback = std::function<void(const std::vector<PointCloudPoint>& points, double timestamp)>;

// ============================================================================
// OusterStream - Main streaming class
// ============================================================================
class OusterStream {
public:
    OusterStream();
    ~OusterStream();
    
    // ========================================================================
    // Public API
    // ========================================================================
    
    // Configuration
    void SetConfig(const OusterConfig& config);
    const OusterConfig& GetConfig() const { return m_config; }
    OusterConfig& GetConfigMutable() { return m_config; }
    
    // Connection
    bool Connect();
    void Disconnect();
    bool IsConnected() const { return m_connected; }
    
    // Streaming control
    void StartStreaming();
    void StopStreaming();
    bool IsStreaming() const { return m_streaming; }
    
    // File loading
    bool LoadPacketFromFile(const std::string& filename);
    bool LoadPcapFromFile(const std::string& pcapFile, const std::string& metadataFile);
    bool LoadMetadataFromFile(const std::string& metadataFile);
    void ClearFileLoadedState();
    
    // Callback and polling
    void SetPointCloudCallback(PointCloudCallback callback) { m_pointCloudCallback = callback; }
    bool PollPointCloud(std::vector<PointCloudPoint>& outPoints, double& outTimestamp);
    
    // Statistics
    int GetFrameCount() const { return m_frameCount; }
    int GetDroppedFrames() const { return m_droppedFrames; }
    double GetLastTimestamp() const { return m_lastTimestamp; }
    int GetBeamCount() const { return m_beamCount; }
    int GetColumnsPerFrame() const { return m_columnsPerFrame; }
    
    // Point conversion utilities (match SDK xyzlut: encoder from column index only, no encoder count / no pixel_shift in LUT)
    PointCloudPoint CalculatePoint3D(uint16_t measurementId, int beam, float range, uint16_t signal,
                                      const std::vector<float>& beamAltitudeAngles,
                                      const std::vector<float>& beamAzimuthAngles);
    PointCloudPoint ConvertPoint(float x, float y, float z, float intensity, float range);
    glm::vec3 MapToColor(float intensity, float range);
    void FlushCurrentFrame(uint64_t lastTimestamp, uint32_t minEncoder, uint32_t maxEncoder, bool encoderInitialized);
    
    // ========================================================================
    // Internal accessors (for use by helper modules)
    // ========================================================================
    
    // State accessors
    void SetConnected(bool connected) { m_connected = connected; }
    void SetStreaming(bool streaming) { m_streaming = streaming; }
    bool ShouldStop() const { return m_shouldStop; }
    void SetShouldStop(bool stop) { m_shouldStop = stop; }
    
    // Thread management
    std::unique_ptr<std::thread>& GetStreamThread() { return m_streamThread; }
    
    // Point cloud queue access
    std::mutex& GetPointCloudMutex() { return m_pointCloudMutex; }
    std::queue<std::pair<std::vector<PointCloudPoint>, double>>& GetPointCloudQueue() { return m_pointCloudQueue; }
    static constexpr size_t MAX_QUEUE_SIZE = 30;
    
    // Accumulated frames access
    std::deque<std::vector<PointCloudPoint>>& GetAccumulatedFrames() { return m_accumulatedFrames; }
    void SetAccumulationDirty(bool dirty) { m_accumulationDirty = dirty; }
    
    // Current frame buffer
    std::vector<PointCloudPoint>& GetCurrentFrame() { return m_currentFrame; }
    
    // File loading state
    bool IsPacketLoadedFromFile() const { return m_packetLoadedFromFile; }
    void SetPacketLoadedFromFile(bool loaded) { m_packetLoadedFromFile = loaded; }
    std::vector<PointCloudPoint>& GetLoadedPacketPoints() { return m_loadedPacketPoints; }
    double GetLoadedPacketTimestamp() const { return m_loadedPacketTimestamp; }
    void SetLoadedPacketTimestamp(double ts) { m_loadedPacketTimestamp = ts; }
    
    // Frame state for file/live loading
    uint16_t GetFileLoadFrameId() const { return m_fileLoadFrameId; }
    void SetFileLoadFrameId(uint16_t id) { m_fileLoadFrameId = id; }
    bool IsFileLoadFrameStarted() const { return m_fileLoadFrameStarted; }
    void SetFileLoadFrameStarted(bool started) { m_fileLoadFrameStarted = started; }
    uint64_t GetFileLoadLastTimestamp() const { return m_fileLoadLastTimestamp; }
    void SetFileLoadLastTimestamp(uint64_t ts) { m_fileLoadLastTimestamp = ts; }
    uint32_t GetFileLoadMinEncoder() const { return m_fileLoadMinEncoder; }
    void SetFileLoadMinEncoder(uint32_t enc) { m_fileLoadMinEncoder = enc; }
    uint32_t GetFileLoadMaxEncoder() const { return m_fileLoadMaxEncoder; }
    void SetFileLoadMaxEncoder(uint32_t enc) { m_fileLoadMaxEncoder = enc; }
    bool IsFileLoadEncoderInitialized() const { return m_fileLoadEncoderInitialized; }
    void SetFileLoadEncoderInitialized(bool init) { m_fileLoadEncoderInitialized = init; }
    int GetPacketsInCurrentScan() const { return m_packetsInCurrentScan; }
    void AddPacketsInCurrentScan(int n = 1) { m_packetsInCurrentScan += n; }
    void ResetPacketsInCurrentScan() { m_packetsInCurrentScan = 0; }
    // Match SDK lidar_packets_per_frame(): ceiling(columns_per_frame / columns_per_packet) for default column_window
    int ExpectedPacketsPerScan() const { return (m_columnsPerPacket > 0) ? static_cast<int>((static_cast<size_t>(m_columnsPerFrame) + m_columnsPerPacket - 1) / m_columnsPerPacket) : 64; }
    
    // Manual path: range/signal images for SDK XYZLut path (row-major, index = beam * w + measurement_id)
    std::vector<uint32_t>& GetManualRangeImage() { return m_manualRangeImage; }
    std::vector<uint16_t>& GetManualSignalImage() { return m_manualSignalImage; }
    void ClearManualRangeImage();
    
    // Beam angles
    std::vector<float>& GetBeamAltitudeAngles() { return m_beamAltitudeAngles; }
    std::vector<float>& GetBeamAzimuthAngles() { return m_beamAzimuthAngles; }
    std::vector<int>& GetPixelShiftByRow() { return m_pixelShiftByRow; }
    
    // Transforms
    float* GetBeamToLidarTransform() { return m_beamToLidarTransform; }
    float GetLidarOriginToBeamOriginMm() const { return m_lidarOriginToBeamOriginMm; }
    void SetLidarOriginToBeamOriginMm(float val) { m_lidarOriginToBeamOriginMm = val; }
    float* GetLidarToSensorTransform() { return m_lidarToSensorTransform; }
    
    // Packet format
    const std::string& GetUdpProfileLidar() const { return m_udpProfileLidar; }
    void SetUdpProfileLidar(const std::string& profile) { m_udpProfileLidar = profile; }
    int GetColumnsPerPacket() const { return m_columnsPerPacket; }
    void SetColumnsPerPacket(int cols) { m_columnsPerPacket = cols; }
    int GetPacketHeaderSize() const { return m_packetHeaderSize; }
    void SetPacketHeaderSize(int size) { m_packetHeaderSize = size; }
    int GetMeasurementBlockHeaderSize() const { return m_measurementBlockHeaderSize; }
    void SetMeasurementBlockHeaderSize(int size) { m_measurementBlockHeaderSize = size; }
    int GetPixelStride() const { return m_pixelStride; }
    void SetPixelStride(int stride) { m_pixelStride = stride; }
    int GetEncoderTicksPerRev() const { return m_encoderTicksPerRev; }
    void SetEncoderTicksPerRev(int ticks) { m_encoderTicksPerRev = ticks; }
    float GetRangeUnit() const { return m_rangeUnit; }
    void SetRangeUnit(float unit) { m_rangeUnit = unit; }
    void SetBeamCount(int count) { m_beamCount = count; }
    void SetColumnsPerFrame(int cols) { m_columnsPerFrame = cols; }
    
    // Statistics (mutable)
    void IncrementFrameCount() { m_frameCount++; }
    void IncrementDroppedFrames() { m_droppedFrames++; }
    std::atomic<int>& GetDroppedFramesAtomic() { return m_droppedFrames; }
    void SetLastTimestamp(double ts) { m_lastTimestamp = ts; }
    std::atomic<uint64_t>& GetLiveLastColumnTimestampNs() { return m_liveLastColumnTimestampNs; }
    
    // Callback access
    PointCloudCallback& GetPointCloudCallback() { return m_pointCloudCallback; }

#ifdef OUSTER_SDK_AVAILABLE
    // SDK objects
    std::shared_ptr<ouster::sdk::sensor::SensorScanSource>& GetSdkScanSource() { return m_sdkScanSource; }
    std::shared_ptr<ouster::sdk::core::SensorInfo>& GetSensorInfo() { return m_sensorInfo; }
    std::shared_ptr<void>& GetXyzLutPtr() { return m_xyzlut_ptr; }
    bool IsSdkStreamingInitialized() const { return m_sdkStreamingInitialized; }
    void SetSdkStreamingInitialized(bool init) { m_sdkStreamingInitialized = init; }
#endif

private:
    void StreamingThread();
    void ProcessLidarPacket(const uint8_t* data, size_t size);
    bool ParseMetadataFile(const std::string& metadataFile);
    void ProcessLidarPacketFromFile(const uint8_t* data, size_t size);

private:
    OusterConfig m_config;
    
    // Connection state
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_streaming{false};
    std::atomic<bool> m_shouldStop{false};
    std::unique_ptr<std::thread> m_streamThread;
    
    // Callback
    PointCloudCallback m_pointCloudCallback;
    
    // Point cloud queue
    std::mutex m_pointCloudMutex;
    std::queue<std::pair<std::vector<PointCloudPoint>, double>> m_pointCloudQueue;
    
    // Accumulated frames
    std::deque<std::vector<PointCloudPoint>> m_accumulatedFrames;
    std::vector<PointCloudPoint> m_accumulatedPointCloud;
    bool m_accumulationDirty = false;
    std::chrono::steady_clock::time_point m_lastAccumulationRebuildTime;
    
    // Statistics
    std::atomic<int> m_frameCount{0};
    std::atomic<int> m_droppedFrames{0};
    std::atomic<double> m_lastTimestamp{0.0};
    std::atomic<uint64_t> m_liveLastColumnTimestampNs{0};
    
    // Sensor metadata
    int m_beamCount = 128;
    int m_columnsPerFrame = 1024;
    
    // Current frame buffer
    std::vector<PointCloudPoint> m_currentFrame;
    
    // File loading state
    bool m_packetLoadedFromFile{false};
    std::vector<PointCloudPoint> m_loadedPacketPoints;
    double m_loadedPacketTimestamp{0.0};
    
    // Frame state
    uint16_t m_fileLoadFrameId{0};
    bool m_fileLoadFrameStarted{false};
    uint64_t m_fileLoadLastTimestamp{0};
    uint32_t m_fileLoadMinEncoder{0};
    uint32_t m_fileLoadMaxEncoder{0};
    bool m_fileLoadEncoderInitialized{false};
    // Scan batching (match SDK: flush only after expected_lidar_packets per scan)
    int m_packetsInCurrentScan{0};
    
    // Manual path: full-scan range/signal images for SDK XYZLut (w * h, row-major)
    std::vector<uint32_t> m_manualRangeImage;
    std::vector<uint16_t> m_manualSignalImage;
    
    // Beam angles
    std::vector<float> m_beamAltitudeAngles;
    std::vector<float> m_beamAzimuthAngles;
    std::vector<int> m_pixelShiftByRow;
    
    // Transforms
    float m_beamToLidarTransform[16];
    float m_lidarOriginToBeamOriginMm = 0.0f;
    float m_lidarToSensorTransform[16];
    
    // Packet format
    std::string m_udpProfileLidar;
    int m_columnsPerPacket = 16;
    int m_packetHeaderSize = 0;
    int m_measurementBlockHeaderSize = 4;
    int m_pixelStride = 12;
    int m_encoderTicksPerRev = 90112;
    float m_rangeUnit = 0.001f;

#ifdef OUSTER_SDK_AVAILABLE
    std::shared_ptr<ouster::sdk::sensor::SensorScanSource> m_sdkScanSource;
    std::shared_ptr<ouster::sdk::core::SensorInfo> m_sensorInfo;
    std::shared_ptr<void> m_xyzlut_ptr;
    bool m_sdkStreamingInitialized{false};
#endif
};

// Manual parsing functions (fallback when SDK not available)
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size);
void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size);

// Convenience function to create a configured stream
std::unique_ptr<OusterStream> CreateOusterStream(const std::string& hostname = "172.17.1.56");

} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
