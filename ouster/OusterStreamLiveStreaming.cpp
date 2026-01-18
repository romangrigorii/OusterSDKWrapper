#include "OusterStreamLiveStreaming.h"
#include "OusterStream.h"
#include "OusterStreamUtilities.h"
#include "../../engine/platform/Logger.h"
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

using namespace ouster::sdk;
using namespace ouster::sdk::core;

// Include sensor headers if BUILD_SENSOR is defined
#ifdef BUILD_SENSOR
#include "ouster/sensor_scan_source.h"
using namespace ouster::sdk::sensor;
#define OUSTER_SENSOR_AVAILABLE 1
#else
#define OUSTER_SENSOR_AVAILABLE 0
#endif
#endif

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
    
    LOG_INFO("OusterLiveStreaming", "Setting UDP destination to auto...");
    snprintf(curlCmd, sizeof(curlCmd), 
             "curl -s -X POST http://%s/api/v1/sensor/cmd/set_udp_dest_auto --max-time 5 > /dev/null 2>&1",
             stream->m_config.hostname.c_str());
    int ret = system(curlCmd);
    if (ret == 0) {
        LOG_INFO("OusterLiveStreaming", "Configured sensor UDP destination to auto");
    } else {
        LOG_WARNING_FMT("OusterLiveStreaming", "Failed to set UDP destination (curl returned %d) - sensor may already be configured", ret);
    }
    
    LOG_INFO_FMT("OusterLiveStreaming", "Setting lidar port to %d...", stream->m_config.lidarPort);
    snprintf(curlCmd, sizeof(curlCmd),
             "ouster-cli source %s config udp_port_lidar %d > /dev/null 2>&1",
             stream->m_config.hostname.c_str(), stream->m_config.lidarPort);
    ret = system(curlCmd);
    if (ret == 0) {
        LOG_INFO_FMT("OusterLiveStreaming", "Set sensor lidar port to %d", stream->m_config.lidarPort);
    } else {
        LOG_INFO("OusterLiveStreaming", "Could not set lidar port via ouster-cli (may already be configured)");
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
        bool imuPortFree = OusterUtilities::VerifyPortIsFree(7503);
        
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports still in use after SDK reset - waiting longer...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // Try cleanup again
            OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
            OusterUtilities::KillProcessesOnPort(7503);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
#endif
    
    if (stream->m_streamThread && stream->m_streamThread->joinable()) {
        stream->m_streamThread->join();
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
    // Try to use SDK for live streaming if available
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
        bool imuPortFree = OusterUtilities::KillProcessesOnPort(7503);
        
        // If ports are still in use, wait longer and try again
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports may still be in use, waiting longer and retrying cleanup...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
            imuPortFree = OusterUtilities::KillProcessesOnPort(7503);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // If ports are still in use, try more aggressive cleanup
        if (!lidarPortFree || !imuPortFree) {
            LOG_WARNING("OusterLiveStreaming", "Ports may still be in use - attempting aggressive cleanup...");
            
            // Check if current process is holding the ports
            pid_t current_pid = getpid();
            char checkCurrentLidar[256], checkCurrentImu[256];
            snprintf(checkCurrentLidar, sizeof(checkCurrentLidar), "lsof -ti :%d 2>/dev/null | grep '^%d$' | wc -l", stream->m_config.lidarPort, current_pid);
            snprintf(checkCurrentImu, sizeof(checkCurrentImu), "lsof -ti :7503 2>/dev/null | grep '^%d$' | wc -l", current_pid);
            
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
                imuPortFree = OusterUtilities::KillProcessesOnPort(7503);
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
                        "%s %d 7503 2>/dev/null || true",
                        scriptPath.c_str(), stream->m_config.lidarPort);
                    LOG_INFO("OusterLiveStreaming", "Running aggressive port cleanup script...");
                    system(killScriptCmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    
                    lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                    imuPortFree = OusterUtilities::KillProcessesOnPort(7503);
                } else {
                    LOG_WARNING("OusterLiveStreaming", "kill_port_auto.sh not found, using built-in cleanup");
                }
            }
            
            if (!lidarPortFree || !imuPortFree) {
                LOG_ERROR("OusterLiveStreaming", "Ports are still in use after aggressive cleanup!");
                LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (lidar) free: %s", stream->m_config.lidarPort, lidarPortFree ? "YES" : "NO");
                LOG_ERROR_FMT("OusterLiveStreaming", "Port 7503 (imu) free: %s", imuPortFree ? "YES" : "NO");
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
            bool imuPortActuallyFree = OusterUtilities::VerifyPortIsFree(7503);
            
            if (!lidarPortActuallyFree || !imuPortActuallyFree) {
                LOG_WARNING("OusterLiveStreaming", "Ports reported as free but bind test failed - waiting longer...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                lidarPortFree = OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                imuPortFree = OusterUtilities::KillProcessesOnPort(7503);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                lidarPortActuallyFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
                imuPortActuallyFree = OusterUtilities::VerifyPortIsFree(7503);
                
                if (!lidarPortActuallyFree || !imuPortActuallyFree) {
                    LOG_ERROR("OusterLiveStreaming", "Ports still not free after extended wait!");
                    LOG_ERROR_FMT("OusterLiveStreaming", "Port %d (lidar) bind test: %s", stream->m_config.lidarPort, lidarPortActuallyFree ? "PASS" : "FAIL");
                    LOG_ERROR_FMT("OusterLiveStreaming", "Port 7503 (imu) bind test: %s", imuPortActuallyFree ? "PASS" : "FAIL");
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
        
        while (retryCount < maxRetries && !sdkInitialized) {
            if (retryCount > 0) {
                LOG_WARNING_FMT("OusterLiveStreaming", "Retrying SDK initialization (attempt %d/%d)...", retryCount + 1, maxRetries);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                bool lidarPortFree = OusterUtilities::VerifyPortIsFree(stream->m_config.lidarPort);
                bool imuPortFree = OusterUtilities::VerifyPortIsFree(7503);
                
                if (!lidarPortFree || !imuPortFree) {
                    LOG_WARNING("OusterLiveStreaming", "Ports not free on retry - cleaning up again...");
                    OusterUtilities::KillProcessesOnPort(stream->m_config.lidarPort);
                    OusterUtilities::KillProcessesOnPort(7503);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
            
            try {
                stream->m_sdkScanSource = std::make_shared<SensorScanSource>(
                    stream->m_config.hostname,
                    [stream](SensorScanSourceOptions& opts) {
                        opts.lidar_port = stream->m_config.lidarPort;
                        opts.imu_port = 7503;
                        opts.timeout = 60.0f;
                        opts.do_not_reinitialize = false;
                        opts.queue_size = 2;
                        opts.config_timeout = 90.0f;
                        LOG_INFO_FMT("OusterLiveStreaming", "SensorScanSourceOptions configured: lidar_port=%d, imu_port=7503, timeout=60.0s, config_timeout=90.0s", 
                                stream->m_config.lidarPort);
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
        LOG_ERROR_FMT("OusterLiveStreaming", "SDK live streaming failed: %s", e.what());
        LOG_ERROR_FMT("OusterLiveStreaming", "Exception type: %s", typeid(e).name());
        try {
            std::rethrow_if_nested(e);
        } catch (const std::exception& nested) {
            LOG_ERROR_FMT("OusterLiveStreaming", "Nested exception: %s", nested.what());
        } catch (...) {
            LOG_ERROR("OusterLiveStreaming", "Nested exception of unknown type");
        }
        LOG_ERROR("OusterLiveStreaming", "SDK initialization failed - stopping streaming (no fallback to manual mode)");
        stream->m_sdkStreamingInitialized = false;
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
        stream->m_streaming = false;
        return;
    } catch (...) {
        LOG_ERROR("OusterLiveStreaming", "SDK live streaming failed: unknown exception (not std::exception)");
        LOG_ERROR("OusterLiveStreaming", "SDK initialization failed - stopping streaming (no fallback to manual mode)");
        stream->m_sdkStreamingInitialized = false;
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
        stream->m_streaming = false;
        return;
    }
    
#else  // OUSTER_SENSOR_AVAILABLE
    LOG_INFO("OusterLiveStreaming", "Ouster sensor library not available, using manual UDP socket streaming");
#endif  // OUSTER_SENSOR_AVAILABLE
#endif  // OUSTER_SDK_AVAILABLE
    
    // Manual UDP socket streaming (fallback or when SDK not available)
    {
    LOG_INFO("OusterLiveStreaming", "Using manual UDP socket streaming");
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Failed to create socket (errno: %d)", errno);
        stream->m_streaming = false;
        return;
    }
    LOG_INFO("OusterLiveStreaming", "Socket created successfully");
    
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    int bufSize = 4 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
    
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(stream->m_config.lidarPort);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    LOG_INFO_FMT("OusterLiveStreaming", "Attempting to bind to port %d...", stream->m_config.lidarPort);
    
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERROR_FMT("OusterLiveStreaming", "Failed to bind to port %d (errno: %d - %s)", 
                      stream->m_config.lidarPort, errno, strerror(errno));
        close(sock);
        stream->m_streaming = false;
        return;
    }
    
    LOG_INFO_FMT("OusterLiveStreaming", "Successfully bound to UDP port %d", stream->m_config.lidarPort);
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
        
        ssize_t received = recv(sock, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            recvFailures++;
            if (recvFailures <= 10) {
                LOG_WARNING_FMT("OusterLiveStreaming", "recv failed: %zd (errno: %d - %s)", 
                               received, errno, strerror(errno));
            }
            continue;
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
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
    } catch (...) {
        LOG_ERROR("OusterLiveStreaming", "Unknown exception in StreamingThread (not std::exception)");
        stream->m_streaming = false;
        if (stream->m_sdkScanSource) {
            stream->m_sdkScanSource.reset();
        }
        if (stream->m_sensorInfo) {
            stream->m_sensorInfo.reset();
        }
        if (stream->m_xyzlut_ptr) {
            stream->m_xyzlut_ptr.reset();
        }
    }
#else
    stream->m_streaming = false;
#endif
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
