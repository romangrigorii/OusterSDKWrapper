#include "OusterStreamLiveStreaming.h"
#include "OusterStream.h"
#include "OusterStreamUtilities.h"
#include "platform/Logger.h"
#include <thread>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#ifdef __APPLE__
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#endif

// Ouster C++ SDK includes (if available)
#ifdef OUSTER_SDK_AVAILABLE
#include "ouster/lidar_scan.h"
#include "ouster/xyzlut.h"
#include "ouster/field.h"
#include "ouster/types.h"
#include "ouster/packet.h"

using namespace ouster::sdk;
using namespace ouster::sdk::core;

// Include sensor headers if BUILD_SENSOR is defined
#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"
#include "ouster/sensor_packet_source.h"
using namespace ouster::sdk::sensor;
#define OUSTER_SENSOR_AVAILABLE 1
#else
#define OUSTER_SENSOR_AVAILABLE 0
#endif
#endif

// SDK-only mode: Use Ouster SDK's SensorScanSource with XYZLut for correct point cloud generation
// Manual UDP streaming has been removed to simplify the code

// Forward declare manual parsing function
namespace Engine {
namespace Editors {
namespace CameraSystem {
void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size);
}
}
}

namespace Engine {
namespace Editors {
namespace CameraSystem {
namespace OusterLiveStreaming {

bool Connect(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return false;
    }
#ifdef __APPLE__
    LOG_INFO_FMT("OusterLiveStreaming", "Connect() called - configuring sensor at %s", stream->m_config.hostname.c_str());
    
    char curlCmd[512];
    
    // Set UDP destination to our specific local IP and configure ports
    LOG_INFO_FMT("OusterLiveStreaming", "Setting UDP destination to %s, lidar port %d, IMU port %d...", 
                 stream->m_config.localIp.c_str(), stream->m_config.lidarPort, stream->m_config.imuPort);
    snprintf(curlCmd, sizeof(curlCmd), 
             "curl -s -X POST -H 'Content-Type: application/json' "
             "-d '{\"udp_dest\": \"%s\", \"udp_port_lidar\": %d, \"udp_port_imu\": %d}' "
             "http://%s/api/v1/sensor/config --max-time 5 2>/dev/null",
             stream->m_config.localIp.c_str(), 
             stream->m_config.lidarPort,
             stream->m_config.imuPort,
             stream->m_config.hostname.c_str());
    int ret = system(curlCmd);
    if (ret == 0) {
        LOG_INFO_FMT("OusterLiveStreaming", "Configured sensor: udp_dest=%s, lidar_port=%d, imu_port=%d", 
                     stream->m_config.localIp.c_str(), stream->m_config.lidarPort, stream->m_config.imuPort);
    } else {
        LOG_WARNING_FMT("OusterLiveStreaming", "Failed to configure sensor (curl returned %d)", ret);
    }
    
    LOG_INFO("OusterLiveStreaming", "Skipping sensor reinitialization (assuming sensor is already configured)");
    LOG_INFO("OusterLiveStreaming", "If no data is received, run setup_ouster.py first to configure the sensor");
    
    stream->m_connected = true;
    LOG_INFO("OusterLiveStreaming", "Connect() completed successfully");
    return true;
#else
    stream->m_connected = true;
    return true;
#endif
}

void Disconnect(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    StopStreaming(stream);
    stream->m_connected = false;
}

void StartStreaming(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    // Always stop any existing stream first to ensure clean state
    if (stream->m_streaming || stream->m_streamThread) {
        LOG_INFO("OusterLiveStreaming", "Stopping existing stream before starting new one...");
        StopStreaming(stream);
        // Wait a bit for thread to fully exit
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    LOG_INFO("OusterLiveStreaming", "StartStreaming() called - creating streaming thread");
    
    // Clean up any existing thread object
    if (stream->m_streamThread) {
        if (stream->m_streamThread->joinable()) {
            stream->m_streamThread->detach();  // Detach if still running (shouldn't happen, but be safe)
        }
        stream->m_streamThread.reset();
    }
    
    // Reset state
    stream->m_shouldStop = false;
    stream->m_streaming = true;
    stream->m_liveLastColumnTimestampNs.store(0);
    
    // Create new thread with exception handling
    try {
        stream->m_streamThread = std::make_unique<std::thread>(&OusterLiveStreaming::StreamingThreadImpl, stream);
        LOG_INFO("OusterLiveStreaming", "Streaming thread created, waiting for it to initialize...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create streaming thread: %s", e.what());
        stream->m_streaming = false;
        stream->m_streamThread.reset();
        throw;  // Re-throw to let caller know it failed
    } catch (...) {
        LOG_ERROR("OusterLiveStreaming", "Failed to create streaming thread: unknown exception");
        stream->m_streaming = false;
        stream->m_streamThread.reset();
        throw;
    }
}

void StopStreaming(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    if (!stream->m_streaming && !stream->m_streamThread) return;
    
    stream->m_shouldStop = true;
    
    // Clean up SDK connection first to release ports
#ifdef OUSTER_SDK_AVAILABLE
    if (stream->m_sdkScanSource) {
        LOG_INFO("OusterLiveStreaming", "Stopping SDK connection to release ports...");
        stream->m_sdkScanSource.reset();
        // Wait longer for SDK to fully release ports (destructor may take time)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Verify ports are actually released
        bool lidarPortFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
        bool imuPortFree = OusterUtilities::VerifyPortIsFree(stream->m_config.imuPort);
        
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports still in use after SDK reset - waiting longer...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // Try cleanup again
            OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
            OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
#endif
    
    if (stream->m_streamThread && stream->m_streamThread->joinable()) {
        // Use a timeout for joining - SDK initialization can block for a long time
        // If the thread doesn't respond in 2 seconds, detach it and let it die with the process
        auto start = std::chrono::steady_clock::now();
        bool joined = false;
        
        // Busy-wait with short sleeps to check if thread finished
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
            // Check if thread is still running by trying to get its native handle
            // Unfortunately C++ doesn't have timed_join, so we poll
            if (!stream->m_streaming) {
                // Thread has finished its work
                stream->m_streamThread->join();
                joined = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!joined) {
            LOG_WARNING("OusterLiveStreaming", "Streaming thread not responding - detaching (SDK may still be initializing)");
            stream->m_streamThread->detach();
        }
    }
    stream->m_streamThread.reset();
    
    if (!stream->m_currentFrame.empty()) {
        uint64_t ts = stream->m_liveLastColumnTimestampNs.load();
        if (ts == 0) {
            ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }
        stream->FlushCurrentFrame(ts, 0, 0, false);
    }
    
    // Clean up SDK resources
#ifdef OUSTER_SDK_AVAILABLE
    if (stream->m_sensorInfo) {
        stream->m_sensorInfo.reset();
    }
    if (stream->m_xyzlut_ptr) {
        stream->m_xyzlut_ptr.reset();
    }
    stream->m_sdkStreamingInitialized = false;
#endif
    
    stream->m_streaming = false;
    LOG_INFO("OusterLiveStreaming", "Streaming stopped and ports released");
}

void StreamingThreadImpl(OusterStream* stream) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
#ifdef __APPLE__
    // Wrap entire thread function in try-catch to prevent uncaught exceptions
    try {
        LOG_INFO("OusterLiveStreaming", "StreamingThread started");
        
        // Set up a terminate handler to catch any uncaught exceptions that cause std::terminate
        std::set_terminate([]() {
            LOG_ERROR("OusterLiveStreaming", "std::terminate() called - uncaught exception in SDK");
            LOG_ERROR("OusterLiveStreaming", "This usually means an exception was thrown but not caught");
            try {
                auto exc = std::current_exception();
                if (exc) {
                    std::rethrow_exception(exc);
                }
            } catch (const std::exception& e) {
                LOG_ERROR_FMT("OusterLiveStreaming", "Uncaught exception: %s", e.what());
            } catch (...) {
                LOG_ERROR("OusterLiveStreaming", "Uncaught exception of unknown type");
            }
            std::abort();
        });
    
#ifdef OUSTER_SDK_AVAILABLE
#if OUSTER_SENSOR_AVAILABLE
    // Skip SDK if bridge mode is enabled (SDK can't use the bridge, it needs direct UDP)
    if (stream->m_config.useBridge) {
        LOG_INFO("OusterLiveStreaming", "Bridge mode enabled - skipping SDK (using manual parsing with Python bridge)");
        goto manual_streaming;
    }
    
    // Use Ouster SDK for live streaming (only works if macOS allows direct UDP reception)
    try {
        LOG_INFO("OusterLiveStreaming", "Attempting to initialize Ouster SDK live streaming...");
        LOG_INFO_FMT("OusterLiveStreaming", "Connecting to sensor at %s, lidar_port=%d", 
                     stream->m_config.hostname.c_str(), stream->m_config.lidarPort);
        
        // Ensure any previous SDK connection is fully cleaned up
        if (stream->m_sdkScanSource) {
            LOG_INFO("OusterLiveStreaming", "Cleaning up previous SDK connection...");
            stream->m_sdkScanSource.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Clean up any processes using the ports before SDK initialization
        LOG_INFO("OusterLiveStreaming", "Cleaning up processes using lidar and IMU ports...");
        bool lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
        bool imuPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
        
        // If ports are still in use, wait longer and try again
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports may still be in use, waiting longer and retrying cleanup...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
            imuPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // If ports are still in use, try more aggressive cleanup
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports may still be in use - attempting aggressive cleanup...");
            
            // Check if current process is holding the ports
            pid_t current_pid = getpid();
            char checkCurrentLidar[256], checkCurrentImu[256];
            snprintf(checkCurrentLidar, sizeof(checkCurrentLidar), "lsof -ti :%d 2>/dev/null | grep '^%d$' | wc -l", stream->m_config.lidarPort, current_pid);
            snprintf(checkCurrentImu, sizeof(checkCurrentImu), "lsof -ti :%d 2>/dev/null | grep '^%d$' | wc -l", stream->m_config.imuPort, current_pid);
            
            FILE* lidarFp = popen(checkCurrentLidar, "r");
            FILE* imuFp = popen(checkCurrentImu, "r");
            bool currentHoldingLidar = false, currentHoldingImu = false;
            
            if (lidarFp) {
                char buf[128];
                if (fgets(buf, sizeof(buf), lidarFp) != nullptr) {
                    currentHoldingLidar = (atoi(buf) > 0);
                }
                pclose(lidarFp);
            }
            if (imuFp) {
                char buf[128];
                if (fgets(buf, sizeof(buf), imuFp) != nullptr) {
                    currentHoldingImu = (atoi(buf) > 0);
                }
                pclose(imuFp);
            }
            
            if (currentHoldingLidar || currentHoldingImu) {
                LOG_WARNING("OusterLiveStreaming", "Current process is holding ports from previous SDK connection");
                LOG_WARNING("OusterLiveStreaming", "Waiting for SDK to fully release ports...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                if (stream->m_sdkScanSource) {
                    LOG_INFO("OusterLiveStreaming", "Force-resetting SDK connection to release ports...");
                    stream->m_sdkScanSource.reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                
                lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                imuPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
            }
            
            // If still not free, try using kill_port_auto.sh script
            if (!lidarPortFree || !imuPortFree) {
                std::string scriptPath = "./ouster/kill_port_auto.sh";
                if (!std::filesystem::exists(scriptPath)) {
                    scriptPath = "camera_system/ouster/kill_port_auto.sh";
                }
                
                if (std::filesystem::exists(scriptPath)) {
                    char killScriptCmd[512];
                    snprintf(killScriptCmd, sizeof(killScriptCmd),
                        "%s %d %d 2>/dev/null || true",
                        scriptPath.c_str(), stream->m_config.lidarPort, stream->m_config.imuPort);
                    LOG_INFO("OusterLiveStreaming", "Running aggressive port cleanup script...");
                    system(killScriptCmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    
                    lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                    imuPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
                } else {
                    LOG_WARNING("OusterLiveStreaming", "kill_port_auto.sh not found, using built-in cleanup");
                }
            }
            
            if (!lidarPortFree || !imuPortFree) {
                LOG_ERROR("OusterLiveStreaming", "Ports are still in use after aggressive cleanup!");
                LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (lidar) free: %s", stream->m_config.lidarPort, lidarPortFree ? "YES" : "NO");
                LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (imu) free: %s", stream->m_config.imuPort, imuPortFree ? "YES" : "NO");
                LOG_ERROR("OusterLiveStreaming", "Please manually run: ./ouster/kill_port.sh");
                LOG_ERROR("OusterLiveStreaming", "Or wait 30-60 seconds for TIME_WAIT sockets to expire");
                LOG_WARNING("OusterLiveStreaming", "Waiting 3 seconds for TIME_WAIT sockets...");
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            } else {
                LOG_INFO("OusterLiveStreaming", "Ports freed after aggressive cleanup");
            }
        } else {
            LOG_INFO("OusterLiveStreaming", "Ports verified free by process check - verifying with actual bind test...");
            bool lidarPortActuallyFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
            bool imuPortActuallyFree = OusterUtilities::VerifyPortIsFree(stream->m_config.imuPort);
            
            if (!lidarPortActuallyFree || !imuPortActuallyFree) {
                LOG_WARNING("OusterLiveStreaming", "Ports reported as free but bind test failed - waiting longer...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                imuPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                lidarPortActuallyFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
                imuPortActuallyFree = OusterUtilities::VerifyPortIsFree(stream->m_config.imuPort);
                
                if (!lidarPortActuallyFree || !imuPortActuallyFree) {
                    LOG_ERROR("OusterLiveStreaming", "Ports still not free after extended wait!");
                    LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (lidar) bind test: %s", stream->m_config.lidarPort, lidarPortActuallyFree ? "PASS" : "FAIL");
                    LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (imu) bind test: %s", stream->m_config.imuPort, imuPortActuallyFree ? "PASS" : "FAIL");
                    throw std::runtime_error("Ports are not available for binding - please wait 30-60 seconds or manually kill processes");
                }
            }
            
            LOG_INFO("OusterLiveStreaming", "Ports verified free with bind test - ready for SDK initialization");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Check sensor connectivity before attempting SDK initialization
        LOG_INFO_FMT("OusterLiveStreaming", "Checking sensor connectivity at %s...", stream->m_config.hostname.c_str());
        
        bool systemInfoReachable = false;
        bool metadataReachable = false;
        
        char curlCmd[512];
        snprintf(curlCmd, sizeof(curlCmd), 
                 "curl -s --max-time 10 http://%s/api/v1/system/info > /dev/null 2>&1",
                 stream->m_config.hostname.c_str());
        int connectivityCheck = system(curlCmd);
        systemInfoReachable = (connectivityCheck == 0);
        
        if (!systemInfoReachable) {
            LOG_WARNING_FMT("OusterLiveStreaming", "Sensor system/info endpoint not reachable (curl returned %d)", connectivityCheck);
        } else {
            LOG_INFO("OusterLiveStreaming", "Sensor system/info endpoint is reachable");
        }
        
        snprintf(curlCmd, sizeof(curlCmd), 
                 "curl -s --max-time 10 http://%s/api/v1/sensor/metadata/sensor_info > /dev/null 2>&1",
                 stream->m_config.hostname.c_str());
        int metadataCheck = system(curlCmd);
        metadataReachable = (metadataCheck == 0);
        
        if (!metadataReachable) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Sensor metadata endpoint not reachable (curl returned %d)", metadataCheck);
            LOG_ERROR("OusterLiveStreaming", "This is the endpoint the SDK will use - SDK initialization will likely fail");
            LOG_ERROR("OusterLiveStreaming", "Possible causes:");
            LOG_ERROR("OusterLiveStreaming", "  1. Sensor is not powered on");
            LOG_ERROR_FMT("OusterLiveStreaming", "  2. Sensor IP address is incorrect (current: %s)", stream->m_config.hostname.c_str());
            LOG_ERROR("OusterLiveStreaming", "  3. Network connectivity issues");
            LOG_ERROR("OusterLiveStreaming", "  4. Firewall blocking HTTP requests");
            LOG_ERROR("OusterLiveStreaming", "  5. Sensor is busy or unresponsive");
            LOG_ERROR("OusterLiveStreaming", "");
            LOG_ERROR("OusterLiveStreaming", "Troubleshooting steps:");
            LOG_ERROR_FMT("OusterLiveStreaming", "  - Ping the sensor: ping %s", stream->m_config.hostname.c_str());
            LOG_ERROR_FMT("OusterLiveStreaming", "  - Test manually: curl -v http://%s/api/v1/system/info", stream->m_config.hostname.c_str());
            LOG_ERROR_FMT("OusterLiveStreaming", "  - Check sensor web interface: http://%s", stream->m_config.hostname.c_str());
            LOG_ERROR("OusterLiveStreaming", "  - Verify sensor is powered on and network cable is connected");
        } else {
            LOG_INFO("OusterLiveStreaming", "Sensor metadata endpoint is reachable");
        }
        
        // If metadata endpoint is not reachable, check if we have pre-loaded metadata
        if (!metadataReachable) {
            if (stream->m_sensorInfo && stream->m_xyzlut_ptr) {
                LOG_WARNING("OusterLiveStreaming", "Sensor metadata endpoint unreachable, but using pre-loaded metadata");
                LOG_INFO("OusterLiveStreaming", "Proceeding with SDK initialization using pre-loaded metadata");
            } else {
                LOG_ERROR("OusterLiveStreaming", "Cannot proceed with SDK initialization - metadata endpoint unreachable and no pre-loaded metadata");
                LOG_ERROR("OusterLiveStreaming", "Load a metadata file first using --ouster-metadata or LoadMetadataFromFile()");
                throw std::runtime_error("Sensor metadata endpoint unreachable - check sensor connectivity or load metadata file first");
            }
        } else {
            if (stream->m_sensorInfo && stream->m_xyzlut_ptr) {
                LOG_INFO("OusterLiveStreaming", "Metadata endpoint is reachable, but pre-loaded metadata is available");
                LOG_INFO("OusterLiveStreaming", "Will use pre-loaded metadata for parsing (faster, no dependency on sensor response time)");
            }
        }
        
        // Note: USE_SENSOR_PACKET_SOURCE now jumps directly to manual_streaming at the top
        LOG_INFO("OusterLiveStreaming", "Creating SensorScanSource (modern SDK API - handles everything automatically)...");
        LOG_INFO("OusterLiveStreaming", "This may take up to 60 seconds if sensor needs configuration...");
        LOG_INFO("OusterLiveStreaming", "SensorScanSource will automatically:");
        LOG_INFO("OusterLiveStreaming", "  - Connect to sensor and retrieve metadata");
        LOG_INFO("OusterLiveStreaming", "  - Configure sensor if needed (firmware 2.5.3+)");
        LOG_INFO("OusterLiveStreaming", "  - Bind to UDP ports and receive packets");
        LOG_INFO("OusterLiveStreaming", "  - Batch packets into complete LidarScans");
        
        // Retry SDK initialization up to 3 times if bind fails
        int maxRetries = 3;
        int retryCount = 0;
        bool sdkInitialized = false;
        
        while (retryCount < maxRetries && !sdkInitialized && !stream->m_shouldStop) {
            // Check if we should stop before each attempt
            if (stream->m_shouldStop) {
                LOG_INFO("OusterLiveStreaming", "Stop requested, aborting SDK initialization");
                stream->m_streaming = false;
                return;
            }
            
            if (retryCount > 0) {
                LOG_WARNING_FMT("OusterLiveStreaming", "Retrying SDK initialization (attempt %d/%d)...", retryCount + 1, maxRetries);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                if (stream->m_shouldStop) {
                    LOG_INFO("OusterLiveStreaming", "Stop requested during retry wait");
                    stream->m_streaming = false;
                    return;
                }
                
                bool lidarPortFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
                bool imuPortFree = OusterUtilities::VerifyPortIsFree(stream->m_config.imuPort);
                
                if (!lidarPortFree || !imuPortFree) {
                    LOG_WARNING("OusterLiveStreaming", "Ports not free on retry - cleaning up again...");
                    OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                    OusterUtilities::KillProcessesOnPort(stream->m_config.imuPort);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
            
            try {
                // Final check before blocking SDK call
                if (stream->m_shouldStop) {
                    LOG_INFO("OusterLiveStreaming", "Stop requested before SDK initialization");
                    stream->m_streaming = false;
                    return;
                }
                
                LOG_INFO("OusterLiveStreaming", "Creating SensorScanSource (this may take up to 15 seconds)...");
                stream->m_sdkScanSource = std::make_shared<SensorScanSource>(
                    stream->m_config.hostname,
                    [stream](SensorScanSourceOptions& opts) {
                        opts.lidar_port = stream->m_config.lidarPort;
                        opts.imu_port = stream->m_config.imuPort;
                        opts.timeout = 15.0f;  // Reduced from 60s - fail faster if sensor not responding
                        opts.do_not_reinitialize = true;  // Don't reinit sensor - we already configured it
                        opts.queue_size = 2;
                        opts.config_timeout = 20.0f;  // Reduced from 90s
                        LOG_INFO_FMT("OusterLiveStreaming", "SensorScanSourceOptions: lidar_port=%d, imu_port=%d, timeout=15s", 
                                stream->m_config.lidarPort, stream->m_config.imuPort);
                    }
                );
                LOG_INFO("OusterLiveStreaming", "SensorScanSource created successfully");
                sdkInitialized = true;
            } catch (const std::exception& e) {
                std::string errorMsg = e.what();
                LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create SensorScanSource (attempt %d/%d): %s", 
                             retryCount + 1, maxRetries, errorMsg.c_str());
                LOG_ERROR_FMT("OusterLiveStreaming", "Exception type: %s", typeid(e).name());
                
                bool isBindError = (errorMsg.find("bind") != std::string::npos || 
                                   errorMsg.find("UDP socket") != std::string::npos ||
                                   errorMsg.find("Address already in use") != std::string::npos);
                
                if (isBindError && retryCount < maxRetries - 1) {
                    LOG_WARNING("OusterLiveStreaming", "Bind error detected - will retry after cleanup");
                    if (stream->m_sdkScanSource) {
                        stream->m_sdkScanSource.reset();
                    }
                } else {
                    try {
                        std::rethrow_if_nested(e);
                    } catch (const std::exception& nested) {
                        LOG_ERROR_FMT("OusterLiveStreaming", "Nested exception: %s", nested.what());
                    } catch (...) {
                        LOG_ERROR("OusterLiveStreaming", "Nested exception of unknown type");
                    }
                    
                    if (retryCount >= maxRetries - 1) {
                        LOG_ERROR("OusterLiveStreaming", "Max retries reached - giving up");
                        throw;
                    }
                }
            } catch (...) {
                LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create SensorScanSource (attempt %d/%d): unknown exception", 
                             retryCount + 1, maxRetries);
                if (retryCount >= maxRetries - 1) {
                    throw std::runtime_error("Unknown exception creating SensorScanSource");
                }
            }
            
            retryCount++;
        }
        
        if (!sdkInitialized) {
            throw std::runtime_error("Failed to initialize SDK after multiple retries");
        }
        
        // Check if the scan source is live
        try {
            if (!stream->m_sdkScanSource->is_live()) {
                throw std::runtime_error("SensorScanSource is not live - sensor may not be configured or accessible");
            }
            LOG_INFO("OusterLiveStreaming", "SensorScanSource is live");
        } catch (const std::exception& e) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Error checking is_live(): %s", e.what());
            throw;
        } catch (...) {
            LOG_ERROR("OusterLiveStreaming", "Unknown error checking is_live()");
            throw std::runtime_error("Unknown error checking SensorScanSource live status");
        }
#endif
        
        // Get sensor info from the scan source (or use pre-loaded metadata)
        try {
            if (stream->m_sensorInfo && stream->m_xyzlut_ptr) {
                LOG_INFO("OusterLiveStreaming", "Using pre-loaded metadata for parsing");
                LOG_INFO_FMT("OusterLiveStreaming", "Pre-loaded sensor info: SN=%llu", 
                            static_cast<unsigned long long>(stream->m_sensorInfo->sn));
                
                const auto& sensorInfos = stream->m_sdkScanSource->sensor_info();
                if (!sensorInfos.empty() && sensorInfos[0]) {
                    if (sensorInfos[0]->sn != stream->m_sensorInfo->sn) {
                        LOG_WARNING_FMT("OusterLiveStreaming", "Sensor SN mismatch: pre-loaded=%llu, sensor=%llu (using pre-loaded)",
                                      static_cast<unsigned long long>(stream->m_sensorInfo->sn),
                                      static_cast<unsigned long long>(sensorInfos[0]->sn));
                    } else {
                        LOG_INFO("OusterLiveStreaming", "Pre-loaded metadata matches sensor metadata");
                    }
                }
            } else {
                const auto& sensorInfos = stream->m_sdkScanSource->sensor_info();
                if (sensorInfos.empty() || !sensorInfos[0]) {
                    throw std::runtime_error("No sensor info available from SensorScanSource");
                }
                stream->m_sensorInfo = sensorInfos[0];
                LOG_INFO_FMT("OusterLiveStreaming", "Retrieved sensor info from sensor: SN=%llu", 
                            static_cast<unsigned long long>(stream->m_sensorInfo->sn));
                
                auto xyzlut = std::make_shared<XYZLut>(*stream->m_sensorInfo, false);
                stream->m_xyzlut_ptr = std::static_pointer_cast<void>(xyzlut);
                LOG_INFO("OusterLiveStreaming", "XYZ lookup table created from sensor metadata");
            }
        } catch (const std::exception& e) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Error getting/using sensor_info(): %s", e.what());
            throw;
        } catch (...) {
            LOG_ERROR("OusterLiveStreaming", "Unknown error getting sensor info");
            throw std::runtime_error("Unknown error getting sensor info");
        }
        
        stream->m_sdkStreamingInitialized = true;
        LOG_INFO("OusterLiveStreaming", "Ouster SDK live streaming initialized successfully");
        LOG_INFO_FMT("OusterLiveStreaming", "Sensor SN: %llu", static_cast<unsigned long long>(stream->m_sensorInfo->sn));
        LOG_INFO_FMT("OusterLiveStreaming", "Min range: %.2fm, Max range: %.2fm", stream->m_config.minRange, stream->m_config.maxRange);
        
        // Stream using SDK - iterate through scans
        int scanCount = 0;
        auto scanIt = stream->m_sdkScanSource->begin(0);
        auto scanEnd = stream->m_sdkScanSource->end();
        
        LOG_INFO("OusterLiveStreaming", "Starting SDK scan iteration...");
        
        while (!stream->m_shouldStop) {
            if (scanIt == scanEnd) {
                LOG_WARNING("OusterLiveStreaming", "SDK iterator reached end - this shouldn't happen for live streaming");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                scanIt = stream->m_sdkScanSource->begin(0);
                scanEnd = stream->m_sdkScanSource->end();
                continue;
            }
            
            LidarScanSet scanSet = *scanIt;
            ++scanIt;
            
            for (const auto& scanPtr : scanSet) {
                if (!scanPtr) {
                    continue;
                }
                
                const LidarScan& scan = *scanPtr;
                
                scanCount++;
                if (scanCount == 1 || scanCount % 10 == 0) {
                    LOG_INFO_FMT("OusterLiveStreaming", "Received scan %d via SDK", scanCount);
                }
                
                auto ranges = scan.field<uint32_t>(ChanField::RANGE);
                
                auto xyzlut = std::static_pointer_cast<XYZLut>(stream->m_xyzlut_ptr);
                auto xyz = (*xyzlut)(ranges);
                
                double rangeUnit = RANGE_UNIT;
                
                std::vector<PointCloudPoint> points;
                size_t w = scan.w;
                size_t h = scan.h;
                points.reserve(w * h);
                
                bool hasSignal = scan.has_field(ChanField::SIGNAL);
                bool hasReflectivity = scan.has_field(ChanField::REFLECTIVITY);
                
                for (size_t col = 0; col < w; col++) {
                    for (size_t row = 0; row < h; row++) {
                        uint32_t rangeCounts = ranges(row, col);
                        
                        if (rangeCounts == 0) {
                            continue;
                        }
                        
                        double rangeM = static_cast<double>(rangeCounts) * rangeUnit;
                        
                        if (rangeM < stream->m_config.minRange || rangeM > stream->m_config.maxRange) {
                            continue;
                        }
                        
                        size_t idx = row * w + col;
                        double x = xyz(idx, 0);
                        double y = xyz(idx, 1);
                        double z = xyz(idx, 2);
                        
                        float intensity = 0.0f;
                        if (hasSignal) {
                            auto signalField = scan.field<uint16_t>(ChanField::SIGNAL);
                            intensity = static_cast<float>(signalField(row, col));
                        } else if (hasReflectivity) {
                            auto reflectivityField = scan.field<uint16_t>(ChanField::REFLECTIVITY);
                            intensity = static_cast<float>(reflectivityField(row, col));
                        }
                        
                        if (stream->m_config.downsampleFactor > 1 && (row * w + col) % stream->m_config.downsampleFactor != 0) {
                            continue;
                        }
                        
                        PointCloudPoint point;
                        point.position = glm::vec3(
                            static_cast<float>(x * 100.0),
                            static_cast<float>(y * 100.0),
                            static_cast<float>(z * 100.0)
                        );
                        point.color = stream->MapToColor(intensity, static_cast<float>(rangeM));
                        point.cameraId = stream->m_config.cameraId;
                        
                        points.push_back(point);
                    }
                }
                
                double timestamp = 0.0;
                auto timestampField = scan.timestamp();
                if (timestampField.size() > 0) {
                    timestamp = static_cast<double>(timestampField(0)) * 1e-9;
                } else {
                    timestamp = std::chrono::duration<double>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                }
                
                {
                    std::lock_guard<std::mutex> lock(stream->m_pointCloudMutex);
                    if (stream->m_pointCloudQueue.size() >= OusterStream::MAX_QUEUE_SIZE) {
                        stream->m_pointCloudQueue.pop();
                        stream->m_droppedFrames++;
                    }
                    stream->m_pointCloudQueue.push({std::move(points), timestamp});
                }
                
                stream->m_frameCount++;
                stream->m_lastTimestamp = timestamp;
            }
        }
        
        LOG_INFO("OusterLiveStreaming", "SDK streaming thread exiting");
        stream->m_sdkScanSource.reset();
        stream->m_sensorInfo.reset();
        stream->m_xyzlut_ptr.reset();
        stream->m_sdkStreamingInitialized = false;
        
        return;
        
    } catch (const std::exception& e) {
        std::string errorMsg = e.what();
        LOG_ERROR_FMT("OusterLiveStreaming", "SDK live streaming failed: %s", errorMsg.c_str());
        LOG_ERROR_FMT("OusterLiveStreaming", "Exception type: %s", typeid(e).name());
        
        try {
            std::rethrow_if_nested(e);
        } catch (const std::exception& nested) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Nested exception: %s", nested.what());
        } catch (...) {}
        
        stream->m_sdkStreamingInitialized = false;
        stream->m_streaming = false;
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        LOG_ERROR("OusterLiveStreaming", "Check: 1) sensor is on and reachable, 2) ports are not in use by other processes");
        return;
    } catch (...) {
        LOG_ERROR("OusterLiveStreaming", "SDK live streaming failed: unknown exception");
        stream->m_sdkStreamingInitialized = false;
        stream->m_streaming = false;
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        return;
    }
    
#else  // OUSTER_SENSOR_AVAILABLE
    LOG_ERROR("OusterLiveStreaming", "Ouster sensor library not available - rebuild with BUILD_SENSOR=ON");
    stream->m_streaming = false;
    return;
#endif  // OUSTER_SENSOR_AVAILABLE
#else  // OUSTER_SDK_AVAILABLE
    LOG_ERROR("OusterLiveStreaming", "Ouster SDK not available - rebuild with SDK support");
    stream->m_streaming = false;
    return;
#endif  // OUSTER_SDK_AVAILABLE

    // This label kept for compatibility but should not be reached with SDK enabled
    (void)0;  // Suppress unused label warning
manual_streaming:
    // Manual UDP socket streaming (fallback or when SDK not available)
    {
    LOG_INFO("OusterLiveStreaming", "Using manual UDP socket streaming");
    
    // Check if metadata is loaded
    if (stream->m_beamAltitudeAngles.empty() || stream->m_beamAzimuthAngles.empty()) {
        LOG_WARNING("OusterLiveStreaming", "No metadata loaded - using default beam angles");
        LOG_WARNING("OusterLiveStreaming", "For accurate point positions, load metadata first via LoadMetadataFromFile()");
    } else {
        LOG_INFO_FMT("OusterLiveStreaming", "Using pre-loaded metadata: %d beams, %d columns/frame",
                    stream->m_beamCount, stream->m_columnsPerFrame);
    }
    
    int sock = -1;
    bool bound = false;
    int actualPort = stream->m_config.lidarPort;
    std::string bindIp = stream->m_config.localIp;
    
    // Check if using Python bridge (bypasses macOS firewall)
    if (stream->m_config.useBridge) {
        LOG_INFO("OusterLiveStreaming", "Bridge mode enabled - using Unix stream socket");
        LOG_INFO_FMT("OusterLiveStreaming", "Run: python3 udp_bridge.py --src-ip %s --src-port %d",
                     stream->m_config.localIp.c_str(), stream->m_config.lidarPort);
        
        const char* serverPath = "/tmp/ouster_bridge.sock";
        
        // Create Unix STREAM socket and connect to bridge
        sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create Unix socket: %s", strerror(errno));
            stream->m_streaming = false;
            return;
        }
        
        struct sockaddr_un serverAddr;
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sun_family = AF_UNIX;
        strncpy(serverAddr.sun_path, serverPath, sizeof(serverAddr.sun_path) - 1);
        
        // Try to connect with retries
        int connectAttempts = 0;
        while (connectAttempts < 10 && !stream->m_shouldStop) {
            if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == 0) {
                LOG_INFO("OusterLiveStreaming", "Connected to bridge");
                break;
            }
            connectAttempts++;
            if (connectAttempts == 1) {
                LOG_INFO_FMT("OusterLiveStreaming", "Waiting for bridge at %s...", serverPath);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        if (connectAttempts >= 10) {
            LOG_ERROR("OusterLiveStreaming", "Failed to connect - is udp_bridge.py running?");
            close(sock);
            stream->m_streaming = false;
            return;
        }
        
        // Set receive buffer size
        int bufSize = 4 * 1024 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
        
        bound = true;
        
#ifdef OUSTER_SDK_AVAILABLE
        // Hybrid mode: Use SDK's ScanBatcher and XYZLut for correct parsing
        LOG_INFO("OusterLiveStreaming", "Setting up SDK components for hybrid mode...");
        
        // Use existing SensorInfo (must be pre-loaded via metadata file)
        std::shared_ptr<SensorInfo> sensorInfo = stream->m_sensorInfo;
        
        if (sensorInfo) {
            LOG_INFO_FMT("OusterLiveStreaming", "Hybrid mode: Sensor=%s SN=%llu Resolution=%dx%d", 
                        sensorInfo->prod_line.c_str(), 
                        static_cast<unsigned long long>(sensorInfo->sn),
                        static_cast<int>(sensorInfo->format.columns_per_frame),
                        static_cast<int>(sensorInfo->format.pixels_per_column));
            
            // Create SDK components
            auto packetFormat = std::make_shared<PacketFormat>(*sensorInfo);
            ScanBatcher batcher(sensorInfo);
            auto xyzlut = XYZLut(*sensorInfo, false);
            
            // Create a LidarScan using SensorInfo constructor - handles all dimensions correctly
            LidarScan lidarScan(*sensorInfo);
            
            LOG_INFO("OusterLiveStreaming", "SDK hybrid mode ready - using ScanBatcher + XYZLut");
            LOG_INFO_FMT("OusterLiveStreaming", "Range filter: [%.2f, %.2f] m, downsample: %d, accumulation: %s (%d frames)",
                        stream->m_config.minRange, stream->m_config.maxRange,
                        stream->m_config.downsampleFactor,
                        stream->m_config.enableAccumulation ? "enabled" : "disabled",
                        stream->m_config.accumulateFrames);

            
            // Run the hybrid receive loop - stream socket with 4-byte length prefix
            std::vector<uint8_t> buffer(65536);
            struct pollfd pfd;
            pfd.fd = sock;
            pfd.events = POLLIN;
            int totalPackets = 0;
            int totalScans = 0;
            int pollTimeouts = 0;
            auto lastDataTime = std::chrono::steady_clock::now();
            
            // Helper to read exactly N bytes
            auto readExact = [&](void* buf, size_t n) -> bool {
                size_t total = 0;
                uint8_t* ptr = static_cast<uint8_t*>(buf);
                while (total < n && !stream->m_shouldStop) {
                    ssize_t r = recv(sock, ptr + total, n - total, 0);
                    if (r <= 0) {
                        if (r == 0) {
                            LOG_WARNING("OusterLiveStreaming", "Bridge disconnected");
                            return false;
                        }
                        if (errno == EINTR) continue;
                        LOG_ERROR_FMT("OusterLiveStreaming", "recv error: %s", strerror(errno));
                        return false;
                    }
                    total += r;
                }
                return total == n;
            };
            
            while (!stream->m_shouldStop) {
                int ret = poll(&pfd, 1, 100);
                if (ret < 0) {
                    if (errno == EINTR) continue;
                    LOG_ERROR_FMT("OusterLiveStreaming", "poll() error: %s", strerror(errno));
                    break;
                }
                if (ret == 0) {
                    pollTimeouts++;
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastDataTime).count();
                    if (elapsed >= 5 && pollTimeouts % 50 == 0) {
                        LOG_WARNING_FMT("OusterLiveStreaming", "No data for %lld seconds", 
                                       static_cast<long long>(elapsed));
                    }
                    continue;
                }
                if (!(pfd.revents & POLLIN)) continue;
                
                // Read 4-byte length prefix
                uint32_t length;
                if (!readExact(&length, 4)) {
                    goto hybrid_exit;
                }
                
                if (length > 65536) {
                    LOG_ERROR_FMT("OusterLiveStreaming", "Packet too large: %u", length);
                    goto hybrid_exit;
                }
                
                // Read packet data
                if (!readExact(buffer.data(), length)) {
                    goto hybrid_exit;
                }
                
                // Reset stale detection
                lastDataTime = std::chrono::steady_clock::now();
                pollTimeouts = 0;
                
                totalPackets++;
                if (totalPackets == 1 || totalPackets % 100 == 0) {
                    LOG_INFO_FMT("OusterLiveStreaming", "Hybrid: received packet %d, size=%u", 
                                totalPackets, length);
                }
                
                // Create LidarPacket from raw bytes
                LidarPacket packet(static_cast<int>(length));
                std::memcpy(packet.buf.data(), buffer.data(), length);
                packet.format = packetFormat;
                packet.host_timestamp = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
                
                // Pass to ScanBatcher
                bool scanComplete = batcher(packet, lidarScan);
                
                if (scanComplete) {
                    totalScans++;
                    
                    // Get RANGE field and convert to XYZ using XYZLut
                    auto ranges = lidarScan.field<uint32_t>(ChanField::RANGE);
                    auto xyz = xyzlut(ranges);
                    
                    // Build point cloud
                    std::vector<PointCloudPoint> points;
                    size_t w = lidarScan.w;
                    size_t h = lidarScan.h;
                    points.reserve(w * h / 4);  // Most points will be filtered
                    
                    bool hasSignal = lidarScan.has_field(ChanField::SIGNAL);
                    
                    for (size_t col = 0; col < w; col++) {
                        for (size_t row = 0; row < h; row++) {
                            uint32_t rangeCounts = ranges(row, col);
                            if (rangeCounts == 0) continue;
                            
                            double rangeM = static_cast<double>(rangeCounts) * 0.001;  // mm to m
                            if (rangeM < stream->m_config.minRange || rangeM > stream->m_config.maxRange) continue;
                            
                            // Downsample
                            if (stream->m_config.downsampleFactor > 1 && 
                                (row * w + col) % stream->m_config.downsampleFactor != 0) continue;
                            
                            size_t idx = row * w + col;
                            double x = xyz(idx, 0);
                            double y = xyz(idx, 1);
                            double z = xyz(idx, 2);
                            
                            // Skip invalid points
                            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
                            
                            float intensity = 0.0f;
                            if (hasSignal) {
                                auto signalField = lidarScan.field<uint16_t>(ChanField::SIGNAL);
                                intensity = static_cast<float>(signalField(row, col));
                            }
                            
                            PointCloudPoint point;
                            // Convert to centimeters (1 unit = 1 cm)
                            point.position = glm::vec3(
                                static_cast<float>(x * 100.0),
                                static_cast<float>(y * 100.0),
                                static_cast<float>(z * 100.0)
                            );
                            point.color = stream->MapToColor(intensity, static_cast<float>(rangeM));
                            point.cameraId = stream->m_config.cameraId;
                            
                            points.push_back(point);
                        }
                    }
                    
                    // Track range statistics for debugging
                    static float minRawRange = 999999.0f;
                    static float maxRawRange = 0.0f;
                    static float minXYZDist = 999999.0f;
                    static float maxXYZDist = 0.0f;
                    
                    // Scan raw range values to see actual sensor data
                    for (size_t col = 0; col < w; col++) {
                        for (size_t row = 0; row < h; row++) {
                            uint32_t rangeCounts = ranges(row, col);
                            if (rangeCounts == 0) continue;
                            float rangeM = static_cast<float>(rangeCounts) * 0.001f;
                            if (rangeM < minRawRange) minRawRange = rangeM;
                            if (rangeM > maxRawRange) maxRawRange = rangeM;
                        }
                    }
                    
                    if (!points.empty()) {
                        // Update XYZ distance stats
                        for (const auto& pt : points) {
                            float dist = glm::length(pt.position) / 100.0f;  // Convert cm back to m
                            if (dist < minXYZDist) minXYZDist = dist;
                            if (dist > maxXYZDist) maxXYZDist = dist;
                        }
                        
                        if (totalScans == 1 || totalScans % 10 == 0) {
                            LOG_INFO_FMT("OusterLiveStreaming", "Hybrid scan %d: %zu pts, RAW range: [%.1f, %.1f] m, XYZ dist: [%.1f, %.1f] m", 
                                        totalScans, points.size(), minRawRange, maxRawRange, minXYZDist, maxXYZDist);
                        }
                        
                        // Add to accumulated point cloud (ring buffer of recent frames)
                        double timestamp = static_cast<double>(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count()) / 1e9;
                        
                        size_t pointCount = points.size();
                        {
                            std::lock_guard<std::mutex> lock(stream->m_pointCloudMutex);
                            
                            if (stream->m_config.enableAccumulation) {
                                // Add to ring buffer of frames
                                stream->m_accumulatedFrames.push_back(std::move(points));
                                
                                // Remove oldest frames if we exceed the limit
                                while (stream->m_accumulatedFrames.size() > 
                                       static_cast<size_t>(stream->m_config.accumulateFrames)) {
                                    stream->m_accumulatedFrames.pop_front();
                                }
                                stream->m_accumulationDirty = true;
                                
                                if (totalScans <= 5 || totalScans % 10 == 0) {
                                    size_t totalPoints = 0;
                                    for (const auto& frame : stream->m_accumulatedFrames) {
                                        totalPoints += frame.size();
                                    }
                                    LOG_INFO_FMT("OusterLiveStreaming", "Hybrid: added %zu pts, %zu frames accumulated (%zu total pts)", 
                                                pointCount, stream->m_accumulatedFrames.size(), totalPoints);
                                }
                            } else {
                                // Legacy mode: simple queue
                                stream->m_pointCloudQueue.push({std::move(points), timestamp});
                                if (totalScans <= 5 || totalScans % 10 == 0) {
                                    LOG_INFO_FMT("OusterLiveStreaming", "Hybrid: pushed %zu pts to queue (queue size now: %zu)", 
                                                pointCount, stream->m_pointCloudQueue.size());
                                }
                                while (stream->m_pointCloudQueue.size() > stream->MAX_QUEUE_SIZE) {
                                    stream->m_pointCloudQueue.pop();
                                    stream->m_droppedFrames++;
                                }
                            }
                        }
                        stream->m_frameCount++;
                        stream->m_lastTimestamp = timestamp;
                    }
                    
                    // Reset scan for next frame using SensorInfo constructor
                    lidarScan = LidarScan(*sensorInfo);
                }
            }
            
hybrid_exit:
            close(sock);
            stream->m_streaming = false;
            LOG_INFO_FMT("OusterLiveStreaming", "Hybrid mode ended: %d packets, %d scans", totalPackets, totalScans);
            return;
        } else {
            LOG_WARNING("OusterLiveStreaming", "sensorInfo is NULL - metadata not loaded!");
            LOG_WARNING("OusterLiveStreaming", "Run with: --ouster-metadata path/to/metadata.json");
            LOG_WARNING("OusterLiveStreaming", "Falling back to legacy manual parsing (may have issues)");
        }
#endif
        // Fall through to legacy manual parsing if SDK not available or metadata not loaded
    }
    
    // Try ports starting from configured port, then try alternatives if blocked by zombie sockets
    // (Skip this section if we already bound via Unix socket bridge)
    std::vector<int> portsToTry;
    if (!bound) {
        portsToTry = {
            stream->m_config.lidarPort,      // Try configured port first
            stream->m_config.lidarPort + 1,  // Then try +1
            stream->m_config.lidarPort + 2,  // Then +2
        };
    }
    
    for (int portIdx = 0; portIdx < (int)portsToTry.size() && !bound; portIdx++) {
        actualPort = portsToTry[portIdx];
        
        if (portIdx > 0 && !stream->m_config.useBridge) {
            LOG_WARNING_FMT("OusterLiveStreaming", "Port %d blocked by zombie socket, trying port %d...", 
                           portsToTry[portIdx - 1], actualPort);
            
            // Reconfigure sensor to use new port
            char curlCmd[512];
            snprintf(curlCmd, sizeof(curlCmd),
                     "curl -s -X POST -H 'Content-Type: application/json' "
                     "-d '{\"udp_port_lidar\": %d}' "
                     "http://%s/api/v1/sensor/config 2>/dev/null",
                     actualPort, stream->m_config.hostname.c_str());
            LOG_INFO_FMT("OusterLiveStreaming", "Reconfiguring sensor to use port %d...", actualPort);
            int result = system(curlCmd);
            if (result != 0) {
                LOG_WARNING_FMT("OusterLiveStreaming", "Failed to reconfigure sensor port (curl returned %d)", result);
            }
            // Wait for sensor to apply new configuration
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        // Kill any processes using this port (skip in bridge mode - the bridge handles it)
        if (!stream->m_config.useBridge) {
            LOG_INFO_FMT("OusterLiveStreaming", "Cleaning up port %d before binding...", actualPort);
            OusterUtilities::KillProcessesOnPort(actualPort);
        }
        
        // Try to bind to this port (2 attempts per port)
        for (int attempt = 0; attempt < 2 && !bound; attempt++) {
            if (attempt > 0) {
                LOG_INFO_FMT("OusterLiveStreaming", "Bind attempt %d/2 for port %d - waiting before retry...", 
                            attempt + 1, actualPort);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                OusterUtilities::KillProcessesOnPort(actualPort);
            }
        
        // Use IPv4 socket and bind to specific local IP
        // This avoids issues with zombie sockets bound to INADDR_ANY (*)
        bool useIPv6 = false;  // Disabled - causes issues on macOS
        
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create socket (errno: %d)", errno);
            continue;
        }
        
        LOG_INFO("OusterLiveStreaming", "Created IPv4 socket");
        
        // Set socket options BEFORE binding
        int reuse = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            LOG_WARNING_FMT("OusterLiveStreaming", "Failed to set SO_REUSEADDR: %s", strerror(errno));
        }
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
            LOG_WARNING_FMT("OusterLiveStreaming", "Failed to set SO_REUSEPORT: %s", strerror(errno));
        }
        
        int bufSize = 4 * 1024 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
        
        // Set up address based on socket type
        struct sockaddr_storage addr_storage;
        struct sockaddr* addr_ptr;
        socklen_t addr_len;
        
        // Always use IPv4 and bind to specific local IP (not INADDR_ANY)
        // This avoids interference from zombie sockets bound to *
        // IPv6 dual-stack causes issues on macOS, so we skip it
        (void)useIPv6;  // Suppress unused variable warning
        
        struct sockaddr_in* addr4 = (struct sockaddr_in*)&addr_storage;
        memset(addr4, 0, sizeof(*addr4));
        addr4->sin_family = AF_INET;
        addr4->sin_port = htons(actualPort);
        
        // Bind to specific local IP instead of INADDR_ANY
        // This ensures we receive packets even if there are zombie sockets on *
        if (!bindIp.empty()) {
            if (inet_pton(AF_INET, bindIp.c_str(), &addr4->sin_addr) != 1) {
                LOG_WARNING_FMT("OusterLiveStreaming", "Invalid local IP '%s', falling back to INADDR_ANY", 
                               bindIp.c_str());
                addr4->sin_addr.s_addr = INADDR_ANY;
            } else {
                LOG_INFO_FMT("OusterLiveStreaming", "Binding to specific local IP: %s", bindIp.c_str());
            }
        } else {
            addr4->sin_addr.s_addr = INADDR_ANY;
        }
        addr_ptr = (struct sockaddr*)addr4;
        addr_len = sizeof(*addr4);
        
        LOG_INFO_FMT("OusterLiveStreaming", "Attempting to bind to %s:%d...", 
                     bindIp.empty() ? "0.0.0.0" : bindIp.c_str(), actualPort);
        
        if (bind(sock, addr_ptr, addr_len) == 0) {
            bound = true;
            LOG_INFO_FMT("OusterLiveStreaming", "Successfully bound to port %d", actualPort);
            // Update the config to reflect the actual port we're using
            stream->m_config.lidarPort = actualPort;
        } else {
            LOG_WARNING_FMT("OusterLiveStreaming", "Bind to port %d failed (errno: %d - %s)", 
                          actualPort, errno, strerror(errno));
            close(sock);
            sock = -1;
        }
        }  // end inner attempt loop
    }  // end port loop
    
    if (!bound || sock < 0) {
        LOG_ERROR("OusterLiveStreaming", "Failed to bind to any port after trying multiple alternatives");
        LOG_ERROR("OusterLiveStreaming", "All ports appear blocked by zombie sockets");
        LOG_ERROR("OusterLiveStreaming", "Please REBOOT your machine to clear the zombie sockets");
        LOG_ERROR("OusterLiveStreaming", "Or wait 5-10 minutes for them to expire naturally");
        stream->m_streaming = false;
        return;
    }
    
    // Set non-blocking mode after successful bind
    {
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }
    
    LOG_INFO_FMT("OusterLiveStreaming", "Successfully bound to UDP port %d", actualPort);
    LOG_INFO_FMT("OusterLiveStreaming", "Waiting for lidar data from %s...", stream->m_config.hostname.c_str());
    LOG_INFO_FMT("OusterLiveStreaming", "Min range: %.2fm, Max range: %.2fm", stream->m_config.minRange, stream->m_config.maxRange);
    
    int totalPackets = 0;
    std::vector<uint8_t> buffer(65536);
    
    struct pollfd pfd;
    pfd.fd = sock;
    pfd.events = POLLIN;
    
    int pollIterations = 0;
    int recvFailures = 0;
    
    while (!stream->m_shouldStop) {
        int ret = poll(&pfd, 1, 100);
        pollIterations++;
        
        if (pollIterations <= 20 || pollIterations % 50 == 0) {
            LOG_INFO_FMT("OusterLiveStreaming", "Poll iteration %d: ret=%d, revents=0x%x, shouldStop=%d", 
                         pollIterations, ret, pfd.revents, stream->m_shouldStop.load() ? 1 : 0);
        }
        
        if (ret < 0) {
            if (errno != EINTR) {
                LOG_ERROR_FMT("OusterLiveStreaming", "poll() error: %s", strerror(errno));
            }
            continue;
        }
        
        if (ret == 0) {
            continue;
        }
        
        if (!(pfd.revents & POLLIN)) {
            if (pfd.revents & POLLERR) {
                LOG_WARNING("OusterLiveStreaming", "Socket error detected in poll");
            }
            continue;
        }
        
        ssize_t received = 0;
        
        if (stream->m_config.useBridge) {
            // Stream socket with magic + length-prefixed messages
            // Header: 8-byte magic + 4-byte length = 12 bytes
            struct {
                uint64_t magic;
                uint32_t length;
            } __attribute__((packed)) header;
            
            ssize_t headerRead = recv(sock, &header, sizeof(header), MSG_WAITALL);
            if (headerRead != sizeof(header)) {
                if (headerRead == 0) {
                    LOG_WARNING("OusterLiveStreaming", "Bridge disconnected");
                    break;
                }
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    recvFailures++;
                    if (recvFailures <= 10) {
                        LOG_WARNING_FMT("OusterLiveStreaming", "Header recv failed: %zd (errno: %d)", 
                                       headerRead, errno);
                    }
                }
                continue;
            }
            
            const uint64_t MAGIC = 0x4F55535445524C44ULL;  // "OUSTERLID"
            
            // Check magic
            if (header.magic != MAGIC) {
                // Out of sync - skip this read and try again
                recvFailures++;
                if (recvFailures <= 5) {
                    LOG_WARNING_FMT("OusterLiveStreaming", "Bad magic: 0x%llx (expected 0x%llx)", 
                                   (unsigned long long)header.magic, (unsigned long long)MAGIC);
                }
                // Drain socket to resync
                char drain[65536];
                while (recv(sock, drain, sizeof(drain), MSG_DONTWAIT) > 0) {}
                continue;
            }
            
            uint32_t msgLen = header.length;
            
            // Validate length (Ouster packets are ~25KB max)
            if (msgLen > 32768) {
                LOG_ERROR_FMT("OusterLiveStreaming", "Message too large: %u bytes", msgLen);
                continue;
            }
            
            // Read the packet data
            size_t totalRead = 0;
            while (totalRead < msgLen && !stream->m_shouldStop) {
                ssize_t n = recv(sock, buffer.data() + totalRead, msgLen - totalRead, 0);
                if (n <= 0) {
                    if (n == 0) {
                        LOG_WARNING("OusterLiveStreaming", "Bridge disconnected during read");
                        break;
                    }
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // Wait for more data
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                        continue;
                    }
                    LOG_ERROR_FMT("OusterLiveStreaming", "Data recv failed: %s", strerror(errno));
                    break;
                }
                totalRead += n;
            }
            
            if (totalRead != msgLen) {
                continue;
            }
            received = msgLen;
        } else {
            // Regular UDP datagram
            received = recv(sock, buffer.data(), buffer.size(), 0);
            if (received <= 0) {
                recvFailures++;
                if (recvFailures <= 10) {
                    LOG_WARNING_FMT("OusterLiveStreaming", "recv failed: %zd (errno: %d - %s)", 
                                   received, errno, strerror(errno));
                }
                continue;
            }
        }
        
        totalPackets++;
        if (totalPackets == 1) {
            LOG_INFO_FMT("OusterLiveStreaming", "First packet received! Size: %zd bytes", received);
            std::string hexPreview;
            size_t previewLen = std::min(static_cast<size_t>(received), size_t(64));
            for (size_t i = 0; i < previewLen; i++) {
                char hex[4];
                snprintf(hex, sizeof(hex), "%02x ", buffer[i]);
                hexPreview += hex;
            }
            LOG_INFO_FMT("OusterLiveStreaming", "First packet hex dump (first 64 bytes): %s", hexPreview.c_str());
        }
        if (totalPackets <= 5 || totalPackets % 100 == 0) {
            LOG_INFO_FMT("OusterLiveStreaming", "Packet %d received, size: %zd bytes", totalPackets, received);
        }
        
        ProcessLidarPacketImpl(stream, buffer.data(), received);
    }
    
    if (!stream->m_currentFrame.empty()) {
        uint64_t ts = stream->m_liveLastColumnTimestampNs.load();
        if (ts == 0) {
            ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }
        stream->FlushCurrentFrame(ts, 0, 0, false);
    }
    
    close(sock);
    }  // End of manual streaming block
    
    stream->m_streaming = false;
    
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Uncaught exception in StreamingThread: %s", e.what());
        LOG_ERROR_FMT("OusterLiveStreaming", "Exception type: %s", typeid(e).name());
        try {
            std::rethrow_if_nested(e);
        } catch (const std::exception& nested) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Nested exception: %s", nested.what());
        } catch (...) {
            LOG_ERROR("OusterLiveStreaming", "Nested exception of unknown type");
        }
        stream->m_streaming = false;
#ifdef OUSTER_SDK_AVAILABLE
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
#endif
    } catch (...) {
        LOG_ERROR("OusterLiveStreaming", "Unknown exception in StreamingThread (not std::exception)");
        stream->m_streaming = false;
#ifdef OUSTER_SDK_AVAILABLE
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
#endif
    }
}

void ProcessLidarPacketImpl(OusterStream* stream, const uint8_t* data, size_t size) {
    if (!stream) {
        LOG_ERROR("OusterLiveStreaming", "Stream is null");
        return;
    }
    // Use manual parsing for live streaming (SDK doesn't provide live packet source in this version)
    ProcessLidarPacketManual(stream, data, size);
}

} // namespace OusterLiveStreaming
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
