#pragma once

#include "../remesh/Remeshing.h"  // For PointCloudPoint
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>

// Forward declarations for Ouster SDK types
// These are always declared (even when SDK not available) to avoid compilation errors
// The actual usage is guarded by #ifdef OUSTER_SDK_AVAILABLE
namespace ouster {
namespace sdk {
namespace sensor {
class SensorScanSource;
}
namespace core {
class SensorInfo;
// Note: XYZLut is a typedef (using XYZLut = XYZLutT<double>), not a class
// We can't forward declare typedefs, so we'll handle it differently in the implementation
}
}
}

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Forward declare OusterStream class
class OusterStream;

// Forward declare namespace functions for friend declarations
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

// Ouster sensor configuration
struct OusterConfig {
    std::string hostname = "172.17.1.56";  // Your sensor's IP
    int lidarPort = 7504;  // Changed from 7502 to avoid zombie socket
    
    // Transform to apply to all points (sensor -> world frame)
    glm::mat4 sensorToWorld = glm::mat4(1.0f);
    
    // Point cloud filtering
    float minRange = 0.3f;      // Minimum range in meters
    float maxRange = 50.0f;     // Maximum range in meters
    
    // Color mapping
    bool useIntensityAsColor = true;  // Map signal intensity to grayscale
    bool useRangeAsColor = false;     // Map range to color gradient
    
    // Downsampling (skip every N points, 1 = no skip)
    int downsampleFactor = 1;
    
    // Camera ID to assign to all points from this sensor
    int32_t cameraId = 0;
};

using PointCloudCallback = std::function<void(const std::vector<PointCloudPoint>& points, double timestamp)>;

// Ouster streaming class
// Connects to sensor and provides point cloud data in PointCloudPoint format
class OusterStream {
public:
    OusterStream();
    ~OusterStream();
    
    // Configuration
    void SetConfig(const OusterConfig& config);
    const OusterConfig& GetConfig() const { return m_config; }
    
    // Connection
    bool Connect();
    void Disconnect();
    bool IsConnected() const { return m_connected; }
    
    // Streaming control
    void StartStreaming();
    void StopStreaming();
    bool IsStreaming() const { return m_streaming; }
    
    // Load and parse a saved packet file (for debugging)
    bool LoadPacketFromFile(const std::string& filename);
    
    // Load and parse a PCAP file with metadata JSON
    bool LoadPcapFromFile(const std::string& pcapFile, const std::string& metadataFile);
    
    // Load metadata from JSON file (for use with live streaming)
    // This allows faster initialization and offline operation
    bool LoadMetadataFromFile(const std::string& metadataFile);
    
    // Clear file-loaded state (used when switching from file mode to live mode)
    void ClearFileLoadedState();
    
    void SetPointCloudCallback(PointCloudCallback callback) { m_pointCloudCallback = callback; }
    
    bool PollPointCloud(std::vector<PointCloudPoint>& outPoints, double& outTimestamp);
    
    int GetFrameCount() const { return m_frameCount; }
    int GetDroppedFrames() const { return m_droppedFrames; }
    double GetLastTimestamp() const { return m_lastTimestamp; }
    int GetBeamCount() const { return m_beamCount; }
    int GetColumnsPerFrame() const { return m_columnsPerFrame; }
    
    // Calculate 3D coordinates from lidar measurement (shared by live and file paths)
    PointCloudPoint CalculatePoint3D(uint16_t measurementId, int beam, float range, uint16_t signal,
                                      const std::vector<float>& beamAltitudeAngles,
                                      const std::vector<float>& beamAzimuthAngles);
    
    // Convert Ouster point to PointCloudPoint
    PointCloudPoint ConvertPoint(float x, float y, float z, float intensity, float range);
    
    // Apply color mapping based on config
    glm::vec3 MapToColor(float intensity, float range);
    
    // Flush current frame to callback/queue (called on frame boundary or stream stop)
    void FlushCurrentFrame(uint64_t lastTimestamp, uint32_t minEncoder, uint32_t maxEncoder, bool encoderInitialized);
    
private:
    void StreamingThread();
    void ProcessLidarPacket(const uint8_t* data, size_t size);
    
private:
    OusterConfig m_config;
    
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_streaming{false};
    std::atomic<bool> m_shouldStop{false};
    
    std::unique_ptr<std::thread> m_streamThread;
    
    PointCloudCallback m_pointCloudCallback;
    
    std::mutex m_pointCloudMutex;
    std::queue<std::pair<std::vector<PointCloudPoint>, double>> m_pointCloudQueue;
    static constexpr size_t MAX_QUEUE_SIZE = 10;
    
    // Statistics
    std::atomic<int> m_frameCount{0};
    std::atomic<int> m_droppedFrames{0};
    std::atomic<double> m_lastTimestamp{0.0};
    std::atomic<uint64_t> m_liveLastColumnTimestampNs{0};
    
    // Sensor metadata
    int m_beamCount = 128;  // OS0-128
    int m_columnsPerFrame = 1024;
    
    // Internal buffer
    std::vector<PointCloudPoint> m_currentFrame;
    
    // For loaded packet files: aggregate all frames into one persistent frame
    bool m_packetLoadedFromFile{false};
    std::vector<PointCloudPoint> m_loadedPacketPoints;
    double m_loadedPacketTimestamp{0.0};
    
    // Frame state for file loading (non-thread_local)
    uint16_t m_fileLoadFrameId{0};
    bool m_fileLoadFrameStarted{false};
    uint64_t m_fileLoadLastTimestamp{0};
    uint32_t m_fileLoadMinEncoder{0};
    uint32_t m_fileLoadMaxEncoder{0};
    bool m_fileLoadEncoderInitialized{false};
    
    // Beam angles from metadata
    std::vector<float> m_beamAltitudeAngles;
    std::vector<float> m_beamAzimuthAngles;
    
    // Pixel shift for destaggering (essential for LEGACY profile)
    std::vector<int> m_pixelShiftByRow;  // Shift values for each beam/row
    
    // Beam-to-lidar transform (for accurate coordinate calculation)
    float m_beamToLidarTransform[16];  // 4x4 matrix in row-major order
    float m_lidarOriginToBeamOriginMm = 0.0f;  // Distance in mm
    
    // Lidar-to-sensor transform (for sensor frame coordinates)
    float m_lidarToSensorTransform[16];  // 4x4 matrix in row-major order
    
    // Packet format from metadata
    std::string m_udpProfileLidar;  // "LEGACY", "RNG19_RFL8_SIG16_NIR16", etc.
    int m_columnsPerPacket = 16;
    int m_packetHeaderSize = 0;  // Packet-level header size (0 for LEGACY)
    int m_measurementBlockHeaderSize = 4;  // Status field size per measurement block
    int m_pixelStride = 12;  // Bytes per pixel (depends on profile)
    int m_encoderTicksPerRev = 90112;  // Default, should be read from metadata
    float m_rangeUnit = 0.001f;  // Range unit in meters per count (from metadata, default 0.001)
    
    // Helper to parse metadata JSON and extract beam angles, packet format, etc.
    bool ParseMetadataFile(const std::string& metadataFile);
    
    // Process lidar packet when loading from file (uses member variables)
    void ProcessLidarPacketFromFile(const uint8_t* data, size_t size);
    
    // Make manual parsing functions friends so they can access private members
    friend bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
    friend void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size);
    friend void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size);
    
    // Make file loader functions friends so they can access private members
    // Forward declarations are outside the class (before class definition)
    friend bool OusterFileLoader::LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile);
    friend bool OusterFileLoader::LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
    friend bool OusterFileLoader::LoadPacketFromFile(OusterStream* stream, const std::string& filename);
    friend bool OusterFileLoader::ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile);
    friend void OusterFileLoader::ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size);
    
    // Make live streaming functions friends so they can access private members
    // Forward declarations are outside the class (before class definition)
    friend bool OusterLiveStreaming::Connect(OusterStream* stream);
    friend void OusterLiveStreaming::Disconnect(OusterStream* stream);
    friend void OusterLiveStreaming::StartStreaming(OusterStream* stream);
    friend void OusterLiveStreaming::StopStreaming(OusterStream* stream);
    friend void OusterLiveStreaming::StreamingThreadImpl(OusterStream* stream);
    friend void OusterLiveStreaming::ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size);
    
#ifdef OUSTER_SDK_AVAILABLE
    // SDK objects for live streaming
    std::shared_ptr<ouster::sdk::sensor::SensorScanSource> m_sdkScanSource;
    std::shared_ptr<ouster::sdk::core::SensorInfo> m_sensorInfo;
    // XYZLut is a typedef, so we need to include the header to use it
    // We'll use a type alias in the .cpp file
    std::shared_ptr<void> m_xyzlut_ptr;  // Will be cast to std::shared_ptr<XYZLut> in .cpp
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
