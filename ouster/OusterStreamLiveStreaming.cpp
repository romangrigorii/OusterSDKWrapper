// ============================================================================
// OusterStreamLiveStreaming.cpp - Live sensor streaming
// ============================================================================

#include "OusterStreamLiveStreaming.h"
#include "OusterStream.h"
#include "OusterStreamUtilities.h"
#include "OusterStreamCommon.h"
#include "platform/Logger.h"
#include <filesystem>

// Ouster C++ SDK includes (if available)
#ifdef OUSTER_SDK_AVAILABLE
#include "ouster/lidar_scan.h"
#include "ouster/xyzlut.h"
#include "ouster/field.h"
#include "ouster/types.h"
#include "ouster/packet.h"

using namespace ouster::sdk;
using namespace ouster::sdk::core;

#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"
#include "ouster/sensor_packet_source.h"
using namespace ouster::sdk::sensor;
#define OUSTER_SENSOR_AVAILABLE 1
#else
#define OUSTER_SENSOR_AVAILABLE 0
#endif
#endif

// Forward declare manual parsing
namespace Engine {
namespace Editors {
namespace CameraSystem {
void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size);
}}}

namespace Engine {
namespace Editors {
namespace CameraSystem {
namespace OusterLiveStreaming {

// ============================================================================
// Helper functions
// ============================================================================

static bool CheckSensorConnectivity(const std::string& hostname) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "curl -s --max-time 10 http://%s/api/v1/sensor/metadata/sensor_info > /dev/null 2>&1", hostname.c_str());
    return (system(cmd) == 0);
}

static bool EnsurePortsFree(int lidarPort, int imuPort) {
    bool lidarFree = OusterUtilities::KillProcessesOnPort(lidarPort);
    bool imuFree = OusterUtilities::KillProcessesOnPort(imuPort);
    
    if (!lidarFree || !imuFree) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        lidarFree = OusterUtilities::KillProcessesOnPort(lidarPort);
        imuFree = OusterUtilities::KillProcessesOnPort(imuPort);
    }
    
    return lidarFree && imuFree;
}

#ifdef OUSTER_SDK_AVAILABLE
static void ProcessScanToPointCloud(OusterStream* stream, const LidarScan& scan, 
                                     std::shared_ptr<XYZLut> xyzlut,
                                     std::vector<PointCloudPoint>& outPoints) {
    auto ranges = scan.field<uint32_t>(ChanField::RANGE);
    auto xyz = (*xyzlut)(ranges);
    
    const OusterConfig& config = stream->GetConfig();
    size_t w = scan.w;
    size_t h = scan.h;
    outPoints.reserve(w * h / 4);
    
    bool hasSignal = scan.has_field(ChanField::SIGNAL);
    
    for (size_t col = 0; col < w; col++) {
        for (size_t row = 0; row < h; row++) {
            uint32_t rangeCounts = ranges(row, col);
            if (rangeCounts == 0) continue;
            
            double rangeM = static_cast<double>(rangeCounts) * 0.001;
            if (rangeM < config.minRange || rangeM > config.maxRange) continue;
            
            if (config.downsampleFactor > 1 && (row * w + col) % config.downsampleFactor != 0) continue;
            
            size_t idx = row * w + col;
            double x = xyz(idx, 0);
            double y = xyz(idx, 1);
            double z = xyz(idx, 2);
            
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
            float intensity = 0.0f;
            if (hasSignal) {
                auto signalField = scan.field<uint16_t>(ChanField::SIGNAL);
                intensity = static_cast<float>(signalField(row, col));
            }
            
            PointCloudPoint point;
            point.position = glm::vec3(
                static_cast<float>(x * 100.0),
                static_cast<float>(y * 100.0),
                static_cast<float>(z * 100.0)
            );
            point.color = stream->MapToColor(intensity, static_cast<float>(rangeM));
            point.cameraId = config.cameraId;
            
            outPoints.push_back(point);
        }
    }
}

static void AddPointsToStream(OusterStream* stream, std::vector<PointCloudPoint>&& points, double timestamp) {
    if (points.empty()) return;
    
    const OusterConfig& config = stream->GetConfig();
    size_t pointCount = points.size();
    
    std::lock_guard<std::mutex> lock(stream->GetPointCloudMutex());
    
    if (config.enableAccumulation) {
        stream->GetAccumulatedFrames().push_back(std::move(points));
        
        while (stream->GetAccumulatedFrames().size() > static_cast<size_t>(config.accumulateFrames)) {
            stream->GetAccumulatedFrames().pop_front();
        }
        stream->SetAccumulationDirty(true);
    } else {
        auto& queue = stream->GetPointCloudQueue();
        queue.push({std::move(points), timestamp});
        
        while (queue.size() > OusterStream::MAX_QUEUE_SIZE) {
            queue.pop();
            stream->IncrementDroppedFrames();
        }
    }
    
    stream->IncrementFrameCount();
    stream->SetLastTimestamp(timestamp);
    (void)pointCount;  // Suppress unused warning in release builds
}
#endif

// ============================================================================
// Connect
// ============================================================================

bool Connect(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return false;
    }
    
    const OusterConfig& config = stream->GetConfig();
    
#ifdef __APPLE__
    LOG_INFO_FMT("OusterLiveStreaming", "Connecting to sensor at %s", config.hostname.c_str());
    
    // Configure sensor UDP destination
    char curlCmd[512];
    snprintf(curlCmd, sizeof(curlCmd), 
             "curl -s -X POST -H 'Content-Type: application/json' "
             "-d '{\"udp_dest\": \"%s\", \"udp_port_lidar\": %d, \"udp_port_imu\": %d}' "
             "http://%s/api/v1/sensor/config --max-time 5 2>/dev/null",
             config.localIp.c_str(), config.lidarPort, config.imuPort, config.hostname.c_str());
    
    int ret = system(curlCmd);
    if (ret == 0) {
        LOG_INFO_FMT("OusterLiveStreaming", "Configured: udp_dest=%s, ports=%d/%d", 
                     config.localIp.c_str(), config.lidarPort, config.imuPort);
    } else {
        LOG_WARNING_FMT("OusterLiveStreaming", "Sensor config failed (curl=%d)", ret);
    }
#endif
    
    stream->SetConnected(true);
    LOG_INFO("OusterLiveStreaming", "Connect completed");
    return true;
}

// ============================================================================
// Disconnect
// ============================================================================

void Disconnect(OusterStream* stream) {
    if (!stream) return;
    StopStreaming(stream);
    stream->SetConnected(false);
}

// ============================================================================
// StartStreaming
// ============================================================================

void StartStreaming(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    
    // Stop existing stream first
    if (stream->IsStreaming() || stream->GetStreamThread()) {
        LOG_INFO("OusterLiveStreaming", "Stopping existing stream...");
        StopStreaming(stream);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Clean up existing thread
    auto& thread = stream->GetStreamThread();
    if (thread) {
        if (thread->joinable()) thread->detach();
        thread.reset();
    }
    
    // Reset state
    stream->SetShouldStop(false);
    stream->SetStreaming(true);
    stream->GetLiveLastColumnTimestampNs().store(0);
    
    // Create streaming thread
    try {
        thread = std::make_unique<std::thread>(&StreamingThreadImpl, stream);
        LOG_INFO("OusterLiveStreaming", "Streaming thread created");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create thread: %s", e.what());
        stream->SetStreaming(false);
        thread.reset();
        throw;
    }
}

// ============================================================================
// StopStreaming
// ============================================================================

void StopStreaming(OusterStream* stream) {
    if (!stream) return;
    if (!stream->IsStreaming() && !stream->GetStreamThread()) return;
    
    stream->SetShouldStop(true);
    
#ifdef OUSTER_SDK_AVAILABLE
    if (stream->GetSdkScanSource()) {
        LOG_INFO("OusterLiveStreaming", "Stopping SDK connection...");
        stream->GetSdkScanSource().reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
#endif
    
    auto& thread = stream->GetStreamThread();
    if (thread && thread->joinable()) {
        auto start = std::chrono::steady_clock::now();
        bool joined = false;
        
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
            if (!stream->IsStreaming()) {
                thread->join();
                joined = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!joined) {
            LOG_WARNING("OusterLiveStreaming", "Thread not responding - detaching");
            thread->detach();
        }
    }
    thread.reset();
    
    // Flush remaining frame
    if (!stream->GetCurrentFrame().empty()) {
        uint64_t ts = stream->GetLiveLastColumnTimestampNs().load();
        if (ts == 0) {
            ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }
        stream->FlushCurrentFrame(ts, 0, 0, false);
    }
    
#ifdef OUSTER_SDK_AVAILABLE
    stream->GetSensorInfo().reset();
    stream->GetXyzLutPtr().reset();
    stream->SetSdkStreamingInitialized(false);
#endif
    
    stream->SetStreaming(false);
    LOG_INFO("OusterLiveStreaming", "Streaming stopped");
}

// ============================================================================
// StreamingThreadImpl - Main streaming logic
// ============================================================================

void StreamingThreadImpl(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    
    const OusterConfig& config = stream->GetConfig();
    
#ifdef __APPLE__
    try {
        LOG_INFO("OusterLiveStreaming", "StreamingThread started");

        // Check if we should use bridge/hybrid mode or direct SDK mode
        bool useBridgeMode = config.useBridge;

#ifdef OUSTER_SDK_AVAILABLE
#if OUSTER_SENSOR_AVAILABLE
        // Direct SDK mode (without bridge)
        if (!useBridgeMode) {
            // Ensure ports are free
            if (!EnsurePortsFree(config.lidarPort, config.imuPort)) {
                LOG_WARNING("OusterLiveStreaming", "Ports may still be in use");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            
            // Check sensor connectivity
            LOG_INFO_FMT("OusterLiveStreaming", "Checking sensor at %s...", config.hostname.c_str());
            
            if (!CheckSensorConnectivity(config.hostname)) {
                if (stream->GetSensorInfo() && stream->GetXyzLutPtr()) {
                    LOG_WARNING("OusterLiveStreaming", "Sensor unreachable but have pre-loaded metadata");
                } else {
                    LOG_ERROR("OusterLiveStreaming", "Sensor unreachable and no metadata - load metadata first");
                    throw std::runtime_error("Sensor unreachable");
                }
            } else {
                LOG_INFO("OusterLiveStreaming", "Sensor is reachable");
            }
            
            // Initialize SDK with retries
            LOG_INFO("OusterLiveStreaming", "Creating SensorScanSource...");
            
            for (int retry = 0; retry < 3 && !stream->ShouldStop(); retry++) {
                if (retry > 0) {
                    LOG_WARNING_FMT("OusterLiveStreaming", "Retry %d/3...", retry + 1);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    EnsurePortsFree(config.lidarPort, config.imuPort);
                }
                
                try {
                    stream->GetSdkScanSource() = std::make_shared<SensorScanSource>(
                        config.hostname,
                        [&config](SensorScanSourceOptions& opts) {
                            opts.lidar_port = config.lidarPort;
                            opts.imu_port = config.imuPort;
                            opts.timeout = 15.0f;
                            opts.do_not_reinitialize = true;
                            opts.queue_size = 2;
                            opts.config_timeout = 20.0f;
                        }
                    );
                    LOG_INFO("OusterLiveStreaming", "SensorScanSource created");
                    break;
                } catch (const std::exception& e) {
                    LOG_ERROR_FMT("OusterLiveStreaming", "SDK init failed: %s", e.what());
                    if (retry >= 2) throw;
                }
            }
            
            if (!stream->GetSdkScanSource() || !stream->GetSdkScanSource()->is_live()) {
                throw std::runtime_error("Failed to create live SensorScanSource");
            }
            
            // Get sensor info
            if (!stream->GetSensorInfo()) {
                const auto& infos = stream->GetSdkScanSource()->sensor_info();
                if (infos.empty() || !infos[0]) {
                    throw std::runtime_error("No sensor info from SDK");
                }
                stream->GetSensorInfo() = infos[0];
                
                auto xyzlutPtr = std::make_shared<XYZLut>(*stream->GetSensorInfo(), false);
                stream->GetXyzLutPtr() = std::static_pointer_cast<void>(xyzlutPtr);
            }
            
            stream->SetSdkStreamingInitialized(true);
            LOG_INFO_FMT("OusterLiveStreaming", "SDK initialized: SN=%llu", 
                        static_cast<unsigned long long>(stream->GetSensorInfo()->sn));
            
            // Stream scans
            auto xyzlut = std::static_pointer_cast<XYZLut>(stream->GetXyzLutPtr());
            auto scanIt = stream->GetSdkScanSource()->begin(0);
            auto scanEnd = stream->GetSdkScanSource()->end();
            int scanCount = 0;
            
            while (!stream->ShouldStop()) {
                if (scanIt == scanEnd) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    scanIt = stream->GetSdkScanSource()->begin(0);
                    scanEnd = stream->GetSdkScanSource()->end();
                    continue;
                }
                
                LidarScanSet scanSet = *scanIt;
                ++scanIt;
                
                for (const auto& scanPtr : scanSet) {
                    if (!scanPtr) continue;
                    
                    scanCount++;
                    if (scanCount % 10 == 0) {
                        LOG_INFO_FMT("OusterLiveStreaming", "Scan %d via SDK", scanCount);
                    }
                    
                    std::vector<PointCloudPoint> points;
                    ProcessScanToPointCloud(stream, *scanPtr, xyzlut, points);
                    
                    double timestamp = 0.0;
                    auto tsField = scanPtr->timestamp();
                    timestamp = (tsField.size() > 0) ? static_cast<double>(tsField(0)) * 1e-9 :
                        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
                    
                    AddPointsToStream(stream, std::move(points), timestamp);
                }
            }
            
            LOG_INFO("OusterLiveStreaming", "SDK streaming ended");
            stream->GetSdkScanSource().reset();
            stream->SetSdkStreamingInitialized(false);
            stream->SetStreaming(false);
            return;
        }  // End if (!useBridgeMode)
        
#endif  // OUSTER_SENSOR_AVAILABLE
#endif  // OUSTER_SDK_AVAILABLE

        // Bridge/Hybrid mode: Unix socket to Python bridge + SDK parsing
        if (config.useBridge) {
            LOG_INFO("OusterLiveStreaming", "Bridge mode - connecting to Python bridge");
            LOG_INFO_FMT("OusterLiveStreaming", "Run: python3 udp_bridge.py --src-ip %s --src-port %d",
                         config.localIp.c_str(), config.lidarPort);
            
            int sock = socket(AF_UNIX, SOCK_STREAM, 0);
            if (sock < 0) {
                LOG_ERROR_FMT("OusterLiveStreaming", "Socket create failed: %s", strerror(errno));
                stream->SetStreaming(false);
                return;
            }
            
            struct sockaddr_un serverAddr;
            memset(&serverAddr, 0, sizeof(serverAddr));
            serverAddr.sun_family = AF_UNIX;
            strncpy(serverAddr.sun_path, "/tmp/ouster_bridge.sock", sizeof(serverAddr.sun_path) - 1);
            
            // Connect with retries
            for (int i = 0; i < 10 && !stream->ShouldStop(); i++) {
                if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == 0) {
                    LOG_INFO("OusterLiveStreaming", "Connected to bridge");
                    break;
                }
                if (i == 0) LOG_INFO("OusterLiveStreaming", "Waiting for bridge...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                if (i >= 9) {
                    LOG_ERROR("OusterLiveStreaming", "Bridge connect failed - is udp_bridge.py running?");
                    close(sock);
                    stream->SetStreaming(false);
                    return;
                }
            }
            
            int bufSize = 4 * 1024 * 1024;
            setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
            
#ifdef OUSTER_SDK_AVAILABLE
            // Hybrid mode with SDK parsing
            if (stream->GetSensorInfo()) {
                LOG_INFO("OusterLiveStreaming", "Hybrid mode with SDK parsing");
                
                auto packetFormat = std::make_shared<PacketFormat>(*stream->GetSensorInfo());
                ScanBatcher batcher(stream->GetSensorInfo());
                XYZLut xyzlut(*stream->GetSensorInfo(), false);
                LidarScan lidarScan(*stream->GetSensorInfo());
                
                std::vector<uint8_t> buffer(65536);
                struct pollfd pfd = {sock, POLLIN, 0};
                int totalPackets = 0, totalScans = 0;
                
                auto readExact = [&](void* buf, size_t n) -> bool {
                    size_t total = 0;
                    uint8_t* ptr = static_cast<uint8_t*>(buf);
                    while (total < n && !stream->ShouldStop()) {
                        ssize_t r = recv(sock, ptr + total, n - total, 0);
                        if (r <= 0) return false;
                        total += r;
                    }
                    return total == n;
                };
                
                while (!stream->ShouldStop()) {
                    int ret = poll(&pfd, 1, 100);
                    if (ret <= 0 || !(pfd.revents & POLLIN)) continue;
                    
                    uint32_t length;
                    if (!readExact(&length, 4)) break;
                    if (length > 65536) break;
                    if (!readExact(buffer.data(), length)) break;
                    
                    totalPackets++;
                    if (totalPackets % 100 == 0) {
                        LOG_INFO_FMT("OusterLiveStreaming", "Hybrid: packet %d, size=%u", totalPackets, length);
                    }
                    
                    LidarPacket packet(static_cast<int>(length));
                    std::memcpy(packet.buf.data(), buffer.data(), length);
                    packet.format = packetFormat;
                    packet.host_timestamp = static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());
                    
                    if (batcher(packet, lidarScan)) {
                        totalScans++;
                        
                        std::vector<PointCloudPoint> points;
                        auto xyzlutPtr = std::make_shared<XYZLut>(xyzlut);
                        ProcessScanToPointCloud(stream, lidarScan, xyzlutPtr, points);
                        
                        double timestamp = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
                        
                        if (!points.empty() && (totalScans == 1 || totalScans % 10 == 0)) {
                            LOG_INFO_FMT("OusterLiveStreaming", "Hybrid scan %d: %zu pts", totalScans, points.size());
                        }
                        
                        AddPointsToStream(stream, std::move(points), timestamp);
                        lidarScan = LidarScan(*stream->GetSensorInfo());
                    }
                }
                
                close(sock);
                LOG_INFO_FMT("OusterLiveStreaming", "Hybrid ended: %d packets, %d scans", totalPackets, totalScans);
                stream->SetStreaming(false);
                return;
            }
#endif
            // Fallback: manual parsing
            LOG_WARNING("OusterLiveStreaming", "No SDK metadata - using manual parsing");
            
            std::vector<uint8_t> buffer(65536);
            struct pollfd pfd = {sock, POLLIN, 0};
            
            auto readExact = [&](void* buf, size_t n) -> bool {
                size_t total = 0;
                uint8_t* ptr = static_cast<uint8_t*>(buf);
                while (total < n && !stream->ShouldStop()) {
                    ssize_t r = recv(sock, ptr + total, n - total, 0);
                    if (r <= 0) return false;
                    total += r;
                }
                return total == n;
            };
            
            while (!stream->ShouldStop()) {
                int ret = poll(&pfd, 1, 100);
                if (ret <= 0 || !(pfd.revents & POLLIN)) continue;
                
                uint32_t length;
                if (!readExact(&length, 4)) break;
                if (length > 65536) break;
                if (!readExact(buffer.data(), length)) break;
                
                ProcessLidarPacketManual(stream, buffer.data(), length);
            }
            
            close(sock);
            stream->SetStreaming(false);
            return;
        }
        
        // Direct UDP mode (may not work on macOS due to firewall)
        LOG_INFO("OusterLiveStreaming", "Direct UDP mode");
        
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Socket create failed: %s", strerror(errno));
            stream->SetStreaming(false);
            return;
        }
        
        int reuse = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
        
        int bufSize = 4 * 1024 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.lidarPort);
        
        if (!config.localIp.empty()) {
            inet_pton(AF_INET, config.localIp.c_str(), &addr.sin_addr);
        } else {
            addr.sin_addr.s_addr = INADDR_ANY;
        }
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Bind failed on port %d: %s", config.lidarPort, strerror(errno));
            close(sock);
            stream->SetStreaming(false);
            return;
        }
        
        LOG_INFO_FMT("OusterLiveStreaming", "Bound to %s:%d", 
                     config.localIp.empty() ? "0.0.0.0" : config.localIp.c_str(), config.lidarPort);
        
        std::vector<uint8_t> buffer(65536);
        struct pollfd pfd = {sock, POLLIN, 0};
        
        while (!stream->ShouldStop()) {
            int ret = poll(&pfd, 1, 100);
            if (ret <= 0 || !(pfd.revents & POLLIN)) continue;
            
            struct sockaddr_in sender;
            socklen_t senderLen = sizeof(sender);
            ssize_t n = recvfrom(sock, buffer.data(), buffer.size(), 0, 
                                 (struct sockaddr*)&sender, &senderLen);
            
            if (n > 0) {
                ProcessLidarPacketManual(stream, buffer.data(), static_cast<size_t>(n));
            }
        }
        
        close(sock);
        stream->SetStreaming(false);
        return;
        
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Streaming failed: %s", e.what());
#ifdef OUSTER_SDK_AVAILABLE
        stream->GetSdkScanSource().reset();
        stream->SetSdkStreamingInitialized(false);
#endif
        stream->SetStreaming(false);
        return;
    }
#else
    LOG_ERROR("OusterLiveStreaming", "Only macOS is currently supported");
    stream->SetStreaming(false);
#endif  // __APPLE__
}

// ============================================================================
// ProcessLidarPacketImpl - Delegate to manual parsing
// ============================================================================

void ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size) {
    ProcessLidarPacketManual(stream, data, size);
}

} // namespace OusterLiveStreaming
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
