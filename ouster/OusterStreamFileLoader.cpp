#include "OusterStreamFileLoader.h"
#include "OusterStream.h"
#include "../../engine/platform/Logger.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <vector>
#include <mutex>
#include <optional>
#include <glm/glm.hpp>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

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
#endif

// Forward declare manual parsing functions from OusterStreamManual.cpp (in global namespace)
namespace Engine {
namespace Editors {
namespace CameraSystem {
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size);
void ProcessLidarPacketManual(OusterStream* stream, const uint8_t* data, size_t size);
}
}
}

namespace Engine {
namespace Editors {
namespace CameraSystem {
namespace OusterFileLoader {

bool LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
#ifdef OUSTER_SDK_AVAILABLE
    try {
        LOG_INFO_FMT("OusterFileLoader", "Loading metadata from file: %s", metadataFile.c_str());
        
        // Resolve path relative to executable or use absolute path
        std::string actualMetadataPath = metadataFile;
        if (!std::filesystem::path(metadataFile).is_absolute()) {
            // Try relative to current working directory first
            if (!std::filesystem::exists(metadataFile)) {
                // Try relative to executable directory
                char exePath[1024];
                uint32_t size = sizeof(exePath);
#ifdef __APPLE__
                if (_NSGetExecutablePath(exePath, &size) == 0) {
                    std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
                    actualMetadataPath = (exeDir / metadataFile).string();
                }
#else
                ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
                if (len != -1) {
                    exePath[len] = '\0';
                    std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
                    actualMetadataPath = (exeDir / metadataFile).string();
                }
#endif
            }
        }
        
        if (!std::filesystem::exists(actualMetadataPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "Metadata file not found: %s", metadataFile.c_str());
            return false;
        }
        
        // Read metadata file
        std::ifstream file(actualMetadataPath);
        if (!file.is_open()) {
            LOG_ERROR_FMT("OusterFileLoader", "Failed to open metadata file: %s", actualMetadataPath.c_str());
            return false;
        }
        
        std::string metadataJson((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        
        if (metadataJson.empty()) {
            LOG_ERROR_FMT("OusterFileLoader", "Metadata file is empty: %s", actualMetadataPath.c_str());
            return false;
        }
        
        // Parse metadata using SDK
        SensorInfo sensorInfoObj(metadataJson);
        
        // Store as shared_ptr - m_sensorInfo expects ouster::sdk::core::SensorInfo
        stream->m_sensorInfo = std::make_shared<SensorInfo>(sensorInfoObj);
        
        LOG_INFO_FMT("OusterFileLoader", "Metadata loaded successfully: %s (SN: %llu)", 
                    stream->m_sensorInfo->prod_line.c_str(), 
                    static_cast<unsigned long long>(stream->m_sensorInfo->sn));
        
        // Create XYZ lookup table from loaded metadata
        auto xyzlut = std::make_shared<XYZLut>(*stream->m_sensorInfo, false);
        stream->m_xyzlut_ptr = std::static_pointer_cast<void>(xyzlut);
        
        LOG_INFO("OusterFileLoader", "XYZ lookup table created from loaded metadata");
        LOG_INFO("OusterFileLoader", "Metadata will be used for parsing live stream data");
        
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to load metadata: %s", e.what());
        return false;
    } catch (...) {
        LOG_ERROR("OusterFileLoader", "Unknown error loading metadata");
        return false;
    }
#else
    LOG_ERROR("OusterFileLoader", "Ouster SDK not available - cannot load metadata");
    return false;
#endif
}

bool LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
#ifdef __APPLE__
    LOG_INFO_FMT("OusterFileLoader", "Loading PCAP from file: %s with metadata: %s", pcapFile.c_str(), metadataFile.c_str());
    
#ifdef OUSTER_SDK_AVAILABLE
    // Use Ouster C++ SDK for robust PCAP parsing
    try {
        // Resolve file paths
        std::string actualPcapPath = pcapFile;
        std::string actualMetadataPath = metadataFile;
        
        // Try to find files if paths are relative
        auto findFile = [](const std::string& filename) -> std::string {
            if (std::filesystem::exists(filename)) return filename;
            
            std::vector<std::string> pathsToTry = {
                filename,
                (std::filesystem::current_path() / filename).string(),
                "../" + filename,
                "../../" + filename
            };
            
            char exePath[1024];
            uint32_t size = sizeof(exePath);
            if (_NSGetExecutablePath(exePath, &size) == 0) {
                try {
                    std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
                    pathsToTry.push_back((exeDir / filename).string());
                    pathsToTry.push_back((exeDir / "../" / filename).string());
                    pathsToTry.push_back((exeDir / "../../" / filename).string());
                } catch (...) {}
            }
            
            for (const auto& path : pathsToTry) {
                if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
                    return path;
                }
            }
            return filename; // Return original if not found
        };
        
        actualPcapPath = findFile(pcapFile);
        actualMetadataPath = findFile(metadataFile);
        
        if (!std::filesystem::exists(actualPcapPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "PCAP file not found: %s", pcapFile.c_str());
            return false;
        }
        
        if (!std::filesystem::exists(actualMetadataPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "Metadata file not found: %s", metadataFile.c_str());
            return false;
        }
        
        LOG_INFO_FMT("OusterFileLoader", "Using PCAP: %s, Metadata: %s", actualPcapPath.c_str(), actualMetadataPath.c_str());
        
        // Configure PcapScanSource options
        PcapScanSourceOptions opts;
        opts.meta = {actualMetadataPath};
        opts.index = false; // Don't index - process scans on-the-fly (allows partial scans)
        opts.soft_id_check = true; // Allow soft ID checking for better compatibility
        // Don't restrict field_names - decode all available fields (SIGNAL may not be available)
        // opts.field_names = nonstd::make_optional(std::vector<std::string>{"RANGE", "SIGNAL"});
        
        LOG_INFO("OusterFileLoader", "Creating PcapScanSource with options (index=false, soft_id_check=true)...");
        
        // Create PcapScanSource
        PcapScanSource scanSource(actualPcapPath, [&opts](PcapScanSourceOptions& o) { o = opts; });
        
        LOG_INFO("OusterFileLoader", "PcapScanSource created successfully");
        
        // Get sensor info
        const auto& sensorInfos = scanSource.sensor_info();
        if (sensorInfos.empty()) {
            LOG_ERROR("OusterFileLoader", "No sensor info found in PCAP");
            return false;
        }
        
        if (!sensorInfos[0]) {
            LOG_ERROR("OusterFileLoader", "Sensor info is null");
            return false;
        }
        const SensorInfo& sensorInfo = *sensorInfos[0];
        LOG_INFO_FMT("OusterFileLoader", "Sensor: %s (SN: %llu)", sensorInfo.prod_line.c_str(), static_cast<unsigned long long>(sensorInfo.sn));
        
        // Create XYZ lookup table
        XYZLut lut(sensorInfo, false); // false = don't use extrinsics
        
        // Clear previous data
        stream->m_loadedPacketPoints.clear();
        stream->m_packetLoadedFromFile = true;
        stream->m_connected = true;
        stream->m_streaming = false;
        
        // Iterate through scans
        size_t scanCount = 0;
        size_t totalPoints = 0;
        
        LOG_INFO("OusterFileLoader", "Starting scan iteration...");
        
        // Get sensor count first
        size_t sensorCount = scanSource.sensor_info().size();
        LOG_INFO_FMT("OusterFileLoader", "Number of sensors in PCAP: %zu", sensorCount);
        
        // Try using begin() without sensor index first (for multi-sensor sources)
        // If that doesn't work, try begin(0) for single sensor
        auto scanIt = scanSource.begin();
        auto scanEnd = scanSource.end();
        
        LOG_INFO_FMT("OusterFileLoader", "Iterator created with begin(): begin == end? %s", (scanIt == scanEnd ? "YES (empty)" : "NO (has scans)"));
        
        // If empty, try with explicit sensor index 0
        if (scanIt == scanEnd && sensorCount > 0) {
            LOG_INFO("OusterFileLoader", "Trying begin(0) for single sensor...");
            scanIt = scanSource.begin(0);
            scanEnd = scanSource.end();
            LOG_INFO_FMT("OusterFileLoader", "After begin(0): begin == end? %s", (scanIt == scanEnd ? "YES (empty)" : "NO (has scans)"));
        }
        
        if (scanIt == scanEnd) {
            LOG_ERROR("OusterFileLoader", "PCAP scan source is empty - no scans found");
            LOG_ERROR("OusterFileLoader", "This could mean:");
            LOG_ERROR("OusterFileLoader", "  1. PCAP file is empty or corrupted");
            LOG_ERROR("OusterFileLoader", "  2. PCAP file doesn't contain complete lidar scans");
            LOG_ERROR("OusterFileLoader", "  3. Metadata file doesn't match PCAP file");
            LOG_ERROR("OusterFileLoader", "  4. PCAP file format is not supported");
            LOG_ERROR("OusterFileLoader", "  5. PCAP requires complete scans but only partial scans are available");
            LOG_ERROR("OusterFileLoader", "NOT using manual parsing - SDK must work");
            return false;
        }
        
        LOG_INFO("OusterFileLoader", "Scan iterator is valid - starting to process scans...");
        
        for (; scanIt != scanEnd; ++scanIt) {
            LidarScanSet scanSet = *scanIt;
            
            // Iterate through scans in the set (usually just one for single sensor)
            for (const auto& scanPtr : scanSet) {
                if (!scanPtr) {
                    continue;
                }
                
                const LidarScan& scan = *scanPtr;
                scanCount++;
                
                // Get range field (required)
                if (!scan.has_field(ChanField::RANGE)) {
                    LOG_WARNING_FMT("OusterFileLoader", "Scan %zu missing RANGE field, skipping", scanCount);
                    continue;
                }
                auto ranges = scan.field<uint32_t>(ChanField::RANGE);
                
                // Get signal field (optional - may not be available)
                bool hasSignal = scan.has_field(ChanField::SIGNAL);
                
                // Convert to XYZ point cloud
                auto xyz = lut(ranges);
                
                // Use constant range unit (0.001 meters per count)
                constexpr double rangeUnit = RANGE_UNIT;
                
                // Process points
                size_t w = scan.w;
                size_t h = scan.h;
                
                for (size_t col = 0; col < w; col++) {
                    for (size_t row = 0; row < h; row++) {
                        uint32_t rangeCounts = ranges(row, col);
                        
                        // Skip zero range (invalid measurements)
                        if (rangeCounts == 0) {
                            continue;
                        }
                        
                        // Convert range counts to meters
                        double rangeM = static_cast<double>(rangeCounts) * rangeUnit;
                        
                        // Apply range filtering
                        if (rangeM <= 0.0 || rangeM < stream->m_config.minRange || rangeM > stream->m_config.maxRange) {
                            continue;
                        }
                        
                        // Get XYZ coordinates (xyz is h*w x 3 array, row-major)
                        size_t idx = row * w + col;
                        double x = xyz(idx, 0);
                        double y = xyz(idx, 1);
                        double z = xyz(idx, 2);
                        
                        // Get signal/intensity (use 0 if not available)
                        uint16_t signal = 0;
                        if (hasSignal) {
                            auto signals = scan.field<uint16_t>(ChanField::SIGNAL);
                            signal = signals(row, col);
                        }
                        
                        // Convert to PointCloudPoint (position in cm, color from intensity/range)
                        PointCloudPoint point;
                        point.position = glm::vec3(
                            static_cast<float>(x * 100.0), // Convert to cm
                            static_cast<float>(y * 100.0),
                            static_cast<float>(z * 100.0)
                        );
                        point.cameraId = stream->m_config.cameraId;
                        
                        // Apply color mapping (using signal as intensity, rangeM as range)
                        float intensity = static_cast<float>(signal);
                        float range = static_cast<float>(rangeM);
                        glm::vec3 color = stream->MapToColor(intensity, range);
                        point.color = color;
                        
                        stream->m_loadedPacketPoints.push_back(point);
                        totalPoints++;
                    }
                }
                
                if (scanCount % 10 == 0 || scanCount == 1) {
                    LOG_INFO_FMT("OusterFileLoader", "Processed scan %zu: %zu points (total: %zu)", 
                                scanCount, totalPoints, stream->m_loadedPacketPoints.size());
                }
            }
        }
        
        LOG_INFO_FMT("OusterFileLoader", "PCAP loaded: %zu scans, %zu total points extracted", 
                    scanCount, totalPoints);
        
        if (stream->m_loadedPacketPoints.empty()) {
            LOG_WARNING("OusterFileLoader", "No points extracted from PCAP file");
            return false;
        }
        
        // Set timestamp from last scan
        stream->m_loadedPacketTimestamp = static_cast<double>(scanCount) * 0.1; // Approximate
        
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterFileLoader", "Exception loading PCAP with SDK: %s", e.what());
        LOG_WARNING("OusterFileLoader", "Falling back to manual PCAP parsing");
        return ::Engine::Editors::CameraSystem::LoadPcapFromFileManual(stream, pcapFile, metadataFile);
    }
#else
    // Fallback to manual parsing if SDK not available
    LOG_WARNING("OusterFileLoader", "Ouster SDK not available, using manual PCAP parsing");
    return ::Engine::Editors::CameraSystem::LoadPcapFromFileManual(stream, pcapFile, metadataFile);
#endif
#else
    LOG_ERROR("OusterFileLoader", "LoadPcapFromFile: Only macOS is currently supported");
    return false;
#endif
}

bool LoadPacketFromFile(OusterStream* stream, const std::string& filename) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
#ifdef __APPLE__
    LOG_INFO_FMT("OusterFileLoader", "Loading packet from file: %s", filename.c_str());
    
    std::vector<std::string> pathsToTry;
    pathsToTry.push_back(filename);
    
    try {
        pathsToTry.push_back((std::filesystem::current_path() / filename).string());
    } catch (...) {}
    
    char exePath[1024];
    uint32_t size = sizeof(exePath);
    if (_NSGetExecutablePath(exePath, &size) == 0) {
        try {
            std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
            pathsToTry.push_back((exeDir / "../" / filename).string());
            pathsToTry.push_back((exeDir / "../../" / filename).string());
            pathsToTry.push_back((exeDir / filename).string());
        } catch (...) {}
    }
    
    pathsToTry.push_back("../" + filename);
    pathsToTry.push_back("../../" + filename);
    pathsToTry.push_back("../../../" + filename);
    
    try {
        std::filesystem::path currentPath = std::filesystem::current_path();
        if (currentPath.filename() == "bin" || currentPath.filename() == "Release" || currentPath.filename() == "Debug") {
            std::filesystem::path thegamePath = currentPath.parent_path().parent_path();
            pathsToTry.push_back((thegamePath / filename).string());
        }
        if (currentPath.filename() == "thegame") {
            pathsToTry.push_back((currentPath / filename).string());
        }
    } catch (...) {}
    
    FILE* file = nullptr;
    std::string actualPath;
    
    // Try each path
    for (const auto& path : pathsToTry) {
        try {
            if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
                file = fopen(path.c_str(), "rb");
                if (file) {
                    actualPath = path;
                    LOG_INFO_FMT("OusterFileLoader", "Found packet file at: %s", path.c_str());
                    break;
                }
            }
        } catch (...) {
            // Try next path
            continue;
        }
    }
    
    if (!file) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to open packet file: %s", filename.c_str());
        LOG_INFO("OusterFileLoader", "Tried the following paths:");
        for (const auto& path : pathsToTry) {
            try {
                bool exists = std::filesystem::exists(path);
                LOG_INFO_FMT("OusterFileLoader", "  - %s %s", path.c_str(), exists ? "(exists)" : "(not found)");
            } catch (...) {
                LOG_INFO_FMT("OusterFileLoader", "  - %s (error checking)", path.c_str());
            }
        }
        return false;
    }
    
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (fileSize <= 0 || fileSize > 2 * 1024 * 1024) {
        LOG_ERROR_FMT("OusterFileLoader", "Invalid packet file size: %ld bytes (expected up to 2MB for full frame)", fileSize);
        fclose(file);
        return false;
    }
    
    std::vector<uint8_t> buffer(fileSize);
    size_t read = fread(buffer.data(), 1, fileSize, file);
    fclose(file);
    
    if (read != static_cast<size_t>(fileSize)) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to read packet file: read %zu bytes, expected %ld", read, fileSize);
        return false;
    }
    
    LOG_INFO_FMT("OusterFileLoader", "Loaded packet file: %zu bytes", read);
    
    stream->m_loadedPacketPoints.clear();
    stream->m_packetLoadedFromFile = true;
    
    // Process packet using live packet processing (which delegates to manual parsing)
    stream->ProcessLidarPacket(buffer.data(), read);
    
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
    
    LOG_INFO_FMT("OusterFileLoader", "Aggregated %zu points from loaded packet into persistent frame", 
                 stream->m_loadedPacketPoints.size());
    
    if (!stream->m_loadedPacketPoints.empty()) {
        float minX = stream->m_loadedPacketPoints[0].position.x, maxX = stream->m_loadedPacketPoints[0].position.x;
        float minY = stream->m_loadedPacketPoints[0].position.y, maxY = stream->m_loadedPacketPoints[0].position.y;
        float minZ = stream->m_loadedPacketPoints[0].position.z, maxZ = stream->m_loadedPacketPoints[0].position.z;
        float minRange = 0.0f, maxRange = 0.0f;
        bool firstRange = true;
        
        for (const auto& point : stream->m_loadedPacketPoints) {
            minX = std::min(minX, point.position.x);
            maxX = std::max(maxX, point.position.x);
            minY = std::min(minY, point.position.y);
            maxY = std::max(maxY, point.position.y);
            minZ = std::min(minZ, point.position.z);
            maxZ = std::max(maxZ, point.position.z);
            
            float range = std::sqrt(point.position.x * point.position.x + 
                                   point.position.y * point.position.y + 
                                   point.position.z * point.position.z) / 100.0f;
            if (firstRange) {
                minRange = maxRange = range;
                firstRange = false;
            } else {
                minRange = std::min(minRange, range);
                maxRange = std::max(maxRange, range);
            }
        }
        
        LOG_INFO("OusterFileLoader", "Point cloud statistics:");
        LOG_INFO_FMT("OusterFileLoader", "  X: [%.3f, %.3f] cm (%.3f to %.3f m)", minX, maxX, minX/100.0f, maxX/100.0f);
        LOG_INFO_FMT("OusterFileLoader", "  Y: [%.3f, %.3f] cm (%.3f to %.3f m)", minY, maxY, minY/100.0f, maxY/100.0f);
        LOG_INFO_FMT("OusterFileLoader", "  Z: [%.3f, %.3f] cm (%.3f to %.3f m)", minZ, maxZ, minZ/100.0f, maxZ/100.0f);
        LOG_INFO_FMT("OusterFileLoader", "  Range: [%.3f, %.3f] m", minRange, maxRange);
        LOG_INFO_FMT("OusterFileLoader", "  Total points: %zu", stream->m_loadedPacketPoints.size());
        
        size_t sampleSize = std::min(size_t(5), stream->m_loadedPacketPoints.size());
        LOG_INFO_FMT("OusterFileLoader", "  Sample points (first %zu):", sampleSize);
        for (size_t i = 0; i < sampleSize; i++) {
            const auto& p = stream->m_loadedPacketPoints[i];
            float range = std::sqrt(p.position.x * p.position.x + 
                                   p.position.y * p.position.y + 
                                   p.position.z * p.position.z) / 100.0f;
            LOG_INFO_FMT("OusterFileLoader", "    [%zu] x=%.3f, y=%.3f, z=%.3f cm, range=%.3f m, color=(%.3f,%.3f,%.3f)", 
                         i, p.position.x, p.position.y, p.position.z, range, 
                         p.color.r, p.color.g, p.color.b);
        }
    }
    
    stream->m_connected = true;
    stream->m_streaming = false;
    LOG_INFO("OusterFileLoader", "Packet loaded and processed, Ouster stream is now active.");
    
    return true;
#else
    LOG_ERROR("OusterFileLoader", "LoadPacketFromFile: Only macOS is currently supported");
    return false;
#endif
}

bool ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
    FILE* file = fopen(metadataFile.c_str(), "r");
    if (!file) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to open metadata file: %s", metadataFile.c_str());
        return false;
    }
    
    // Read entire file
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (fileSize <= 0 || fileSize > 1024 * 1024) { // Max 1MB
        fclose(file);
        return false;
    }
    
    std::vector<char> buffer(fileSize + 1);
    size_t read = fread(buffer.data(), 1, fileSize, file);
    fclose(file);
    
    if (read != static_cast<size_t>(fileSize)) {
        return false;
    }
    buffer[fileSize] = '\0';
    
    std::string jsonStr(buffer.data());
    
    // Parse beam_altitude_angles
    size_t altPos = jsonStr.find("\"beam_altitude_angles\"");
    if (altPos != std::string::npos) {
        size_t arrayStart = jsonStr.find('[', altPos);
        if (arrayStart != std::string::npos) {
            size_t arrayEnd = jsonStr.find(']', arrayStart);
            if (arrayEnd != std::string::npos) {
                std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                stream->m_beamAltitudeAngles.clear();
                
                size_t pos = 0;
                while (pos < arrayStr.length()) {
                    while (pos < arrayStr.length() && (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                        pos++;
                    }
                    if (pos >= arrayStr.length()) break;
                    
                    size_t end = pos;
                    while (end < arrayStr.length() && arrayStr[end] != ',' && arrayStr[end] != ' ' && arrayStr[end] != '\n' && arrayStr[end] != '\t') {
                        end++;
                    }
                    
                    if (end > pos) {
                        std::string numStr = arrayStr.substr(pos, end - pos);
                        try {
                            float angle = std::stof(numStr);
                            stream->m_beamAltitudeAngles.push_back(angle);
                        } catch (...) {
                        }
                    }
                    pos = end;
                }
                
                LOG_INFO_FMT("OusterFileLoader", "Parsed %zu beam altitude angles from metadata", stream->m_beamAltitudeAngles.size());
            }
        }
    }
    
    // Parse beam_azimuth_angles
    size_t azPos = jsonStr.find("\"beam_azimuth_angles\"");
    if (azPos != std::string::npos) {
        size_t arrayStart = jsonStr.find('[', azPos);
        if (arrayStart != std::string::npos) {
            size_t arrayEnd = jsonStr.find(']', arrayStart);
            if (arrayEnd != std::string::npos) {
                std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                stream->m_beamAzimuthAngles.clear();
                
                size_t pos = 0;
                while (pos < arrayStr.length()) {
                    while (pos < arrayStr.length() && (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                        pos++;
                    }
                    if (pos >= arrayStr.length()) break;
                    
                    size_t end = pos;
                    while (end < arrayStr.length() && arrayStr[end] != ',' && arrayStr[end] != ' ' && arrayStr[end] != '\n' && arrayStr[end] != '\t') {
                        end++;
                    }
                    
                    if (end > pos) {
                        std::string numStr = arrayStr.substr(pos, end - pos);
                        try {
                            float angle = std::stof(numStr);
                            stream->m_beamAzimuthAngles.push_back(angle);
                        } catch (...) {
                        }
                    }
                    pos = end;
                }
                
                LOG_INFO_FMT("OusterFileLoader", "Parsed %zu beam azimuth angles from metadata", stream->m_beamAzimuthAngles.size());
            }
        }
    }
    
    auto FindNestedField = [&jsonStr](const std::string& parent, const std::string& field) -> size_t {
        size_t parentPos = jsonStr.find("\"" + parent + "\"");
        if (parentPos == std::string::npos) return std::string::npos;
        
        size_t braceStart = jsonStr.find('{', parentPos);
        if (braceStart == std::string::npos) return std::string::npos;
        
        int braceDepth = 1;
        size_t pos = braceStart + 1;
        while (pos < jsonStr.length() && braceDepth > 0) {
            if (jsonStr[pos] == '{') braceDepth++;
            else if (jsonStr[pos] == '}') braceDepth--;
            pos++;
        }
        size_t braceEnd = pos - 1;
        
        size_t fieldPos = jsonStr.find("\"" + field + "\"", braceStart);
        if (fieldPos != std::string::npos && fieldPos < braceEnd) {
            return fieldPos;
        }
        return std::string::npos;
    };
    
    auto ExtractStringValue = [&jsonStr](size_t fieldPos) -> std::string {
        size_t colonPos = jsonStr.find(':', fieldPos);
        if (colonPos == std::string::npos) return "";
        
        size_t quoteStart = jsonStr.find('"', colonPos);
        if (quoteStart == std::string::npos) return "";
        
        size_t quoteEnd = jsonStr.find('"', quoteStart + 1);
        if (quoteEnd == std::string::npos) return "";
        
        return jsonStr.substr(quoteStart + 1, quoteEnd - quoteStart - 1);
    };
    
    auto ExtractNumberValue = [&jsonStr](size_t fieldPos) -> float {
        size_t colonPos = jsonStr.find(':', fieldPos);
        if (colonPos == std::string::npos) return 0.0f;
        
        size_t numStart = jsonStr.find_first_of("0123456789.-", colonPos);
        if (numStart == std::string::npos) return 0.0f;
        
        size_t numEnd = numStart;
        while (numEnd < jsonStr.length() && 
               (std::isdigit(jsonStr[numEnd]) || jsonStr[numEnd] == '.' || jsonStr[numEnd] == 'e' || 
                jsonStr[numEnd] == 'E' || jsonStr[numEnd] == '+' || jsonStr[numEnd] == '-')) {
            numEnd++;
        }
        
        try {
            return std::stof(jsonStr.substr(numStart, numEnd - numStart));
        } catch (...) {
            return 0.0f;
        }
    };
    
    // Parse UDP profile and related settings
    size_t profilePos = FindNestedField("data_format", "udp_profile_lidar");
    if (profilePos == std::string::npos) {
        profilePos = FindNestedField("lidar_data_format", "udp_profile_lidar");
    }
    if (profilePos == std::string::npos) {
        profilePos = jsonStr.find("\"udp_profile_lidar\"");
    }
    
    if (profilePos != std::string::npos) {
        stream->m_udpProfileLidar = ExtractStringValue(profilePos);
        LOG_INFO_FMT("OusterFileLoader", "Parsed udp_profile_lidar: '%s'", stream->m_udpProfileLidar.c_str());
        
        if (stream->m_udpProfileLidar == "LEGACY") {
            stream->m_pixelStride = 12;
            stream->m_packetHeaderSize = 0;
            stream->m_measurementBlockHeaderSize = 4;
        } else if (stream->m_udpProfileLidar == "RNG19_RFL8_SIG16_NIR16") {
            stream->m_pixelStride = 8;
            stream->m_packetHeaderSize = 0;
            stream->m_measurementBlockHeaderSize = 4;
        } else if (stream->m_udpProfileLidar == "RNG15_RFL8_NIR8") {
            stream->m_pixelStride = 4;
            stream->m_packetHeaderSize = 0;
            stream->m_measurementBlockHeaderSize = 4;
        } else if (stream->m_udpProfileLidar == "RNG19_RFL8_SIG16_NIR16_DUAL") {
            stream->m_pixelStride = 16;
            stream->m_packetHeaderSize = 0;
            stream->m_measurementBlockHeaderSize = 4;
        } else {
            LOG_WARNING_FMT("OusterFileLoader", "Unknown UDP profile: '%s', using LEGACY defaults", stream->m_udpProfileLidar.c_str());
            stream->m_pixelStride = 12;
            stream->m_packetHeaderSize = 0;
            stream->m_measurementBlockHeaderSize = 4;
        }
        LOG_INFO_FMT("OusterFileLoader", "Profile '%s': pixelStride=%d, blockHeaderSize=%d", 
                    stream->m_udpProfileLidar.c_str(), stream->m_pixelStride, stream->m_measurementBlockHeaderSize);
    } else {
        LOG_WARNING("OusterFileLoader", "udp_profile_lidar not found, using LEGACY defaults");
        stream->m_udpProfileLidar = "LEGACY";
        stream->m_pixelStride = 12;
        stream->m_packetHeaderSize = 0;
        stream->m_measurementBlockHeaderSize = 4;
    }
    
    // Parse range_unit
    size_t rangeUnitPos = FindNestedField("data_format", "range_unit");
    if (rangeUnitPos == std::string::npos) {
        rangeUnitPos = FindNestedField("lidar_data_format", "range_unit");
    }
    if (rangeUnitPos == std::string::npos) {
        rangeUnitPos = jsonStr.find("\"range_unit\"");
    }
    
    if (rangeUnitPos != std::string::npos) {
        stream->m_rangeUnit = ExtractNumberValue(rangeUnitPos);
        if (stream->m_rangeUnit > 0.0f) {
            LOG_INFO_FMT("OusterFileLoader", "Parsed range_unit: %.6f m/count", stream->m_rangeUnit);
        } else {
            LOG_WARNING("OusterFileLoader", "Failed to parse range_unit, using default 0.001");
            stream->m_rangeUnit = 0.001f;
        }
    } else {
        LOG_INFO("OusterFileLoader", "range_unit not found in metadata, using default 0.001 m/count");
        stream->m_rangeUnit = 0.001f;
    }
    
    auto FindNestedNumber = [&](const std::string& parent, const std::string& field) -> int {
        size_t fieldPos = FindNestedField(parent, field);
        if (fieldPos == std::string::npos) return -1;
        return static_cast<int>(ExtractNumberValue(fieldPos));
    };
    
    // Parse columns_per_packet
    int colsPerPacket = FindNestedNumber("data_format", "columns_per_packet");
    if (colsPerPacket <= 0) {
        colsPerPacket = FindNestedNumber("lidar_data_format", "columns_per_packet");
    }
    if (colsPerPacket <= 0) {
        size_t colsPerPacketPos = jsonStr.find("\"columns_per_packet\"");
        if (colsPerPacketPos != std::string::npos) {
            colsPerPacket = static_cast<int>(ExtractNumberValue(colsPerPacketPos));
        }
    }
    if (colsPerPacket > 0) {
        stream->m_columnsPerPacket = colsPerPacket;
        LOG_INFO_FMT("OusterFileLoader", "Parsed columns_per_packet: %d", stream->m_columnsPerPacket);
    }
    
    // Parse columns_per_frame
    int colsPerFrame = FindNestedNumber("data_format", "columns_per_frame");
    if (colsPerFrame <= 0) {
        colsPerFrame = FindNestedNumber("lidar_data_format", "columns_per_frame");
    }
    if (colsPerFrame <= 0) {
        size_t colsPos = jsonStr.find("\"columns_per_frame\"");
        if (colsPos != std::string::npos) {
            colsPerFrame = static_cast<int>(ExtractNumberValue(colsPos));
        }
    }
    if (colsPerFrame > 0) {
        stream->m_columnsPerFrame = colsPerFrame;
        LOG_INFO_FMT("OusterFileLoader", "Parsed columns_per_frame: %d", stream->m_columnsPerFrame);
        
        if (stream->m_columnsPerFrame == 1024) {
            stream->m_encoderTicksPerRev = 90112;
        } else {
            stream->m_encoderTicksPerRev = stream->m_columnsPerFrame * 88;
            LOG_INFO_FMT("OusterFileLoader", "Computed encoder_ticks_per_rev: %d (from %d columns * 88)", 
                        stream->m_encoderTicksPerRev, stream->m_columnsPerFrame);
        }
    }
    
    // Parse pixels_per_column
    int pixelsPerColumn = FindNestedNumber("data_format", "pixels_per_column");
    if (pixelsPerColumn <= 0) {
        pixelsPerColumn = FindNestedNumber("lidar_data_format", "pixels_per_column");
    }
    if (pixelsPerColumn <= 0) {
        size_t pixelsPos = jsonStr.find("\"pixels_per_column\"");
        if (pixelsPos != std::string::npos) {
            pixelsPerColumn = static_cast<int>(ExtractNumberValue(pixelsPos));
        }
    }
    if (pixelsPerColumn > 0) {
        stream->m_beamCount = pixelsPerColumn;
        LOG_INFO_FMT("OusterFileLoader", "Parsed pixels_per_column: %d (beam count set to %d)", pixelsPerColumn, stream->m_beamCount);
    }
    
    // Parse pixel_shift_by_row
    auto FindNestedArray = [&FindNestedField](const std::string& parent, const std::string& field) -> size_t {
        size_t fieldPos = FindNestedField(parent, field);
        if (fieldPos == std::string::npos) return std::string::npos;
        return fieldPos;
    };
    
    size_t shiftPos = FindNestedArray("data_format", "pixel_shift_by_row");
    if (shiftPos == std::string::npos) {
        shiftPos = FindNestedArray("lidar_data_format", "pixel_shift_by_row");
    }
    if (shiftPos == std::string::npos) {
        shiftPos = jsonStr.find("\"pixel_shift_by_row\"");
    }
    
    if (shiftPos != std::string::npos) {
        size_t arrayStart = jsonStr.find('[', shiftPos);
        if (arrayStart != std::string::npos) {
            size_t arrayEnd = jsonStr.find(']', arrayStart);
            if (arrayEnd != std::string::npos) {
                std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                stream->m_pixelShiftByRow.clear();
                
                size_t pos = 0;
                while (pos < arrayStr.length()) {
                    while (pos < arrayStr.length() && (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                        pos++;
                    }
                    if (pos >= arrayStr.length()) break;
                    
                    bool negative = false;
                    if (arrayStr[pos] == '-') {
                        negative = true;
                        pos++;
                    }
                    
                    size_t end = pos;
                    while (end < arrayStr.length() && std::isdigit(arrayStr[end])) {
                        end++;
                    }
                    
                    if (end > pos) {
                        std::string numStr = arrayStr.substr(pos, end - pos);
                        try {
                            int shift = std::stoi(numStr);
                            if (negative) shift = -shift;
                            stream->m_pixelShiftByRow.push_back(shift);
                        } catch (...) {
                        }
                    }
                    pos = end;
                }
                
                LOG_INFO_FMT("OusterFileLoader", "Parsed pixel_shift_by_row: %zu values", stream->m_pixelShiftByRow.size());
            }
        }
    } else {
        LOG_WARNING("OusterFileLoader", "pixel_shift_by_row not found in metadata - azimuth calculation may be incorrect");
    }
    
    // Validate beam angles
    if (stream->m_beamAltitudeAngles.size() != static_cast<size_t>(stream->m_beamCount) || 
        stream->m_beamAzimuthAngles.size() != static_cast<size_t>(stream->m_beamCount)) {
        LOG_WARNING_FMT("OusterFileLoader", "Beam angle count mismatch: altitude=%zu, azimuth=%zu, beamCount=%d. Using defaults.", 
                       stream->m_beamAltitudeAngles.size(), stream->m_beamAzimuthAngles.size(), stream->m_beamCount);
        stream->m_beamAltitudeAngles.clear();
        stream->m_beamAzimuthAngles.clear();
        return false;
    }
    
    if (!stream->m_pixelShiftByRow.empty() && stream->m_pixelShiftByRow.size() != static_cast<size_t>(stream->m_beamCount)) {
        LOG_WARNING_FMT("OusterFileLoader", "pixel_shift_by_row count mismatch: %zu vs beamCount=%d", 
                       stream->m_pixelShiftByRow.size(), stream->m_beamCount);
        stream->m_pixelShiftByRow.clear();
    }
    
    // Parse lidar_origin_to_beam_origin_mm
    size_t originPos = jsonStr.find("\"lidar_origin_to_beam_origin_mm\"");
    if (originPos != std::string::npos) {
        size_t numStart = jsonStr.find_first_of("0123456789.", originPos);
        if (numStart != std::string::npos) {
            size_t numEnd = numStart;
            while (numEnd < jsonStr.length() && (std::isdigit(jsonStr[numEnd]) || jsonStr[numEnd] == '.' || jsonStr[numEnd] == 'e' || jsonStr[numEnd] == 'E' || jsonStr[numEnd] == '+' || jsonStr[numEnd] == '-')) {
                numEnd++;
            }
            try {
                stream->m_lidarOriginToBeamOriginMm = std::stof(jsonStr.substr(numStart, numEnd - numStart));
                LOG_INFO_FMT("OusterFileLoader", "Parsed lidar_origin_to_beam_origin_mm: %.3f", stream->m_lidarOriginToBeamOriginMm);
            } catch (...) {
                LOG_WARNING("OusterFileLoader", "Failed to parse lidar_origin_to_beam_origin_mm");
            }
        }
    }
    
    // Parse beam_to_lidar_transform
    size_t beamTransformPos = jsonStr.find("\"beam_to_lidar_transform\"");
    if (beamTransformPos != std::string::npos) {
        size_t arrayStart = jsonStr.find('[', beamTransformPos);
        if (arrayStart != std::string::npos) {
            size_t arrayEnd = jsonStr.find(']', arrayStart);
            if (arrayEnd != std::string::npos) {
                std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                std::vector<float> values;
                
                size_t pos = 0;
                while (pos < arrayStr.length() && values.size() < 16) {
                    while (pos < arrayStr.length() && (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                        pos++;
                    }
                    if (pos >= arrayStr.length()) break;
                    
                    size_t end = pos;
                    while (end < arrayStr.length() && (std::isdigit(arrayStr[end]) || arrayStr[end] == '.' || arrayStr[end] == '-' || arrayStr[end] == '+' || arrayStr[end] == 'e' || arrayStr[end] == 'E')) {
                        end++;
                    }
                    
                    if (end > pos) {
                        try {
                            values.push_back(std::stof(arrayStr.substr(pos, end - pos)));
                        } catch (...) {}
                    }
                    pos = end;
                }
                
                if (values.size() == 16) {
                    std::memcpy(stream->m_beamToLidarTransform, values.data(), 16 * sizeof(float));
                    LOG_INFO("OusterFileLoader", "Parsed beam_to_lidar_transform");
                }
            }
        }
    }
    
    // Parse lidar_to_sensor_transform
    size_t lidarTransformPos = jsonStr.find("\"lidar_to_sensor_transform\"");
    if (lidarTransformPos != std::string::npos) {
        size_t arrayStart = jsonStr.find('[', lidarTransformPos);
        if (arrayStart != std::string::npos) {
            size_t arrayEnd = jsonStr.find(']', arrayStart);
            if (arrayEnd != std::string::npos) {
                std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                std::vector<float> values;
                
                size_t pos = 0;
                while (pos < arrayStr.length() && values.size() < 16) {
                    while (pos < arrayStr.length() && (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                        pos++;
                    }
                    if (pos >= arrayStr.length()) break;
                    
                    size_t end = pos;
                    while (end < arrayStr.length() && (std::isdigit(arrayStr[end]) || arrayStr[end] == '.' || arrayStr[end] == '-' || arrayStr[end] == '+' || arrayStr[end] == 'e' || arrayStr[end] == 'E')) {
                        end++;
                    }
                    
                    if (end > pos) {
                        try {
                            values.push_back(std::stof(arrayStr.substr(pos, end - pos)));
                        } catch (...) {}
                    }
                    pos = end;
                }
                
                if (values.size() == 16) {
                    std::memcpy(stream->m_lidarToSensorTransform, values.data(), 16 * sizeof(float));
                    LOG_INFO("OusterFileLoader", "Parsed lidar_to_sensor_transform");
                }
            }
        }
    }
    
    return !stream->m_beamAltitudeAngles.empty() && !stream->m_beamAzimuthAngles.empty();
}

void ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return;
    }
    stream->ProcessLidarPacketFromFile(data, size);
}

// Re-export manual parsing functions (call global namespace versions)
namespace OusterFileLoader {
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
    return ::Engine::Editors::CameraSystem::LoadPcapFromFileManual(stream, pcapFile, metadataFile);
}

void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size) {
    ::Engine::Editors::CameraSystem::ProcessLidarPacketFromFileManual(stream, data, size);
}
} // namespace OusterFileLoader

} // namespace OusterFileLoader
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
