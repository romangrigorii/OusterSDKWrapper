// ============================================================================
// OusterStreamFileLoader.cpp - File loading for Ouster streams
// ============================================================================

#include "OusterStreamFileLoader.h"
#include "OusterStream.h"
#include "OusterStreamCommon.h"
#include "platform/Logger.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <algorithm>
#include <cmath>

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

namespace Engine {
namespace Editors {
namespace CameraSystem {

// Forward declare manual parsing functions
bool LoadPcapFromFileManual(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile);
void ProcessLidarPacketFromFileManual(OusterStream* stream, const uint8_t* data, size_t size);

namespace OusterFileLoader {

// ============================================================================
// Path resolution helper
// ============================================================================

static std::string FindFile(const std::string& filename) {
    if (std::filesystem::exists(filename)) return filename;
    
    std::vector<std::string> paths;
    paths.push_back(filename);
    
    try {
        paths.push_back((std::filesystem::current_path() / filename).string());
    } catch (...) {}
    
#ifdef __APPLE__
    char exePath[1024];
    uint32_t size = sizeof(exePath);
    if (_NSGetExecutablePath(exePath, &size) == 0) {
        try {
            std::filesystem::path exeDir = std::filesystem::path(exePath).parent_path();
            paths.push_back((exeDir / filename).string());
            paths.push_back((exeDir / "../" / filename).string());
            paths.push_back((exeDir / "../../" / filename).string());
        } catch (...) {}
    }
#endif
    
    paths.push_back("../" + filename);
    paths.push_back("../../" + filename);
    
    for (const auto& path : paths) {
        if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
            return path;
        }
    }
    return filename;
}

// ============================================================================
// LoadMetadataFromFile
// ============================================================================

bool LoadMetadataFromFile(OusterStream* stream, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
    
#ifdef OUSTER_SDK_AVAILABLE
    try {
        LOG_INFO_FMT("OusterFileLoader", "Loading metadata: %s", metadataFile.c_str());
        
        std::string actualPath = FindFile(metadataFile);
        if (!std::filesystem::exists(actualPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "Metadata file not found: %s", metadataFile.c_str());
            return false;
        }
        
        std::ifstream file(actualPath);
        if (!file.is_open()) {
            LOG_ERROR_FMT("OusterFileLoader", "Failed to open: %s", actualPath.c_str());
            return false;
        }
        
        std::string metadataJson((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        
        if (metadataJson.empty()) {
            LOG_ERROR("OusterFileLoader", "Metadata file is empty");
            return false;
        }
        
        SensorInfo sensorInfoObj(metadataJson);
        stream->GetSensorInfo() = std::make_shared<SensorInfo>(sensorInfoObj);
        
        LOG_INFO_FMT("OusterFileLoader", "Loaded: %s (SN: %llu)", 
                    stream->GetSensorInfo()->prod_line.c_str(), 
                    static_cast<unsigned long long>(stream->GetSensorInfo()->sn));
        
        auto xyzlut = std::make_shared<XYZLut>(*stream->GetSensorInfo(), false);
        stream->GetXyzLutPtr() = std::static_pointer_cast<void>(xyzlut);
        
        LOG_INFO("OusterFileLoader", "XYZ lookup table created");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to load metadata: %s", e.what());
        return false;
    }
#else
    LOG_ERROR("OusterFileLoader", "Ouster SDK not available");
    return false;
#endif
}

// ============================================================================
// LoadPcapFromFile
// ============================================================================

bool LoadPcapFromFile(OusterStream* stream, const std::string& pcapFile, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
    
#ifdef __APPLE__
    LOG_INFO_FMT("OusterFileLoader", "Loading PCAP: %s with metadata: %s", pcapFile.c_str(), metadataFile.c_str());
    
#ifdef OUSTER_SDK_AVAILABLE
    try {
        std::string actualPcapPath = FindFile(pcapFile);
        std::string actualMetadataPath = FindFile(metadataFile);
        
        if (!std::filesystem::exists(actualPcapPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "PCAP not found: %s", pcapFile.c_str());
            return false;
        }
        
        if (!std::filesystem::exists(actualMetadataPath)) {
            LOG_ERROR_FMT("OusterFileLoader", "Metadata not found: %s", metadataFile.c_str());
            return false;
        }
        
        LOG_INFO_FMT("OusterFileLoader", "Using: %s, %s", actualPcapPath.c_str(), actualMetadataPath.c_str());
        
        PcapScanSourceOptions opts;
        opts.meta = {actualMetadataPath};
        opts.index = false;
        opts.soft_id_check = true;
        
        LOG_INFO("OusterFileLoader", "Creating PcapScanSource...");
        PcapScanSource scanSource(actualPcapPath, [&opts](PcapScanSourceOptions& o) { o = opts; });
        
        const auto& sensorInfos = scanSource.sensor_info();
        if (sensorInfos.empty() || !sensorInfos[0]) {
            LOG_ERROR("OusterFileLoader", "No sensor info in PCAP");
            return false;
        }
        
        const SensorInfo& sensorInfo = *sensorInfos[0];
        LOG_INFO_FMT("OusterFileLoader", "Sensor: %s (SN: %llu)", sensorInfo.prod_line.c_str(), 
                    static_cast<unsigned long long>(sensorInfo.sn));
        
        XYZLut lut(sensorInfo, false);
        
        stream->GetLoadedPacketPoints().clear();
        stream->SetPacketLoadedFromFile(true);
        stream->SetConnected(true);
        stream->SetStreaming(false);
        
        size_t scanCount = 0;
        size_t totalPoints = 0;
        const OusterConfig& config = stream->GetConfig();
        
        auto scanIt = scanSource.begin();
        auto scanEnd = scanSource.end();
        
        if (scanIt == scanEnd && scanSource.sensor_info().size() > 0) {
            scanIt = scanSource.begin(0);
            scanEnd = scanSource.end();
        }
        
        if (scanIt == scanEnd) {
            LOG_ERROR("OusterFileLoader", "PCAP empty - no scans found");
            return false;
        }
        
        LOG_INFO("OusterFileLoader", "Processing scans...");
        
        for (; scanIt != scanEnd; ++scanIt) {
            LidarScanSet scanSet = *scanIt;
            
            for (const auto& scanPtr : scanSet) {
                if (!scanPtr) continue;
                
                const LidarScan& scan = *scanPtr;
                scanCount++;
                
                if (!scan.has_field(ChanField::RANGE)) {
                    LOG_WARNING_FMT("OusterFileLoader", "Scan %zu missing RANGE", scanCount);
                    continue;
                }
                
                auto ranges = scan.field<uint32_t>(ChanField::RANGE);
                bool hasSignal = scan.has_field(ChanField::SIGNAL);
                auto xyz = lut(ranges);
                
                constexpr double rangeUnit = RANGE_UNIT;
                size_t w = scan.w;
                size_t h = scan.h;
                
                for (size_t col = 0; col < w; col++) {
                    for (size_t row = 0; row < h; row++) {
                        uint32_t rangeCounts = ranges(row, col);
                        if (rangeCounts == 0) continue;
                        
                        double rangeM = static_cast<double>(rangeCounts) * rangeUnit;
                        if (rangeM <= 0.0 || rangeM < config.minRange || rangeM > config.maxRange) {
                            continue;
                        }
                        
                        size_t idx = row * w + col;
                        double x = xyz(idx, 0);
                        double y = xyz(idx, 1);
                        double z = xyz(idx, 2);
                        
                        uint16_t signal = 0;
                        if (hasSignal) {
                            auto signals = scan.field<uint16_t>(ChanField::SIGNAL);
                            signal = signals(row, col);
                        }
                        
                        PointCloudPoint point;
                        // Use meters to match Python/SDK (xyzlut returns meters)
                        point.position = glm::vec3(
                            static_cast<float>(x),
                            static_cast<float>(y),
                            static_cast<float>(z)
                        );
                        point.cameraId = config.cameraId;
                        point.color = stream->MapToColor(static_cast<float>(signal), static_cast<float>(rangeM));
                        
                        stream->GetLoadedPacketPoints().push_back(point);
                        totalPoints++;
                    }
                }
                
                if (scanCount % 10 == 0 || scanCount == 1) {
                    LOG_INFO_FMT("OusterFileLoader", "Scan %zu: %zu points", scanCount, totalPoints);
                }
            }
        }
        
        LOG_INFO_FMT("OusterFileLoader", "PCAP loaded: %zu scans, %zu points", scanCount, totalPoints);
        
        if (stream->GetLoadedPacketPoints().empty()) {
            LOG_WARNING("OusterFileLoader", "No points extracted");
            return false;
        }
        
        stream->SetLoadedPacketTimestamp(static_cast<double>(scanCount) * 0.1);
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR_FMT("OusterFileLoader", "SDK error: %s", e.what());
        LOG_WARNING("OusterFileLoader", "Falling back to manual parsing");
        return LoadPcapFromFileManual(stream, pcapFile, metadataFile);
    }
#else
    LOG_WARNING("OusterFileLoader", "SDK not available, using manual parsing");
    return LoadPcapFromFileManual(stream, pcapFile, metadataFile);
#endif
#else
    LOG_ERROR("OusterFileLoader", "Only macOS is currently supported");
    return false;
#endif
}

// ============================================================================
// LoadPacketFromFile
// ============================================================================

bool LoadPacketFromFile(OusterStream* stream, const std::string& filename) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
    
#ifdef __APPLE__
    LOG_INFO_FMT("OusterFileLoader", "Loading packet: %s", filename.c_str());
    
    std::string actualPath = FindFile(filename);
    FILE* file = fopen(actualPath.c_str(), "rb");
    
    if (!file) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to open: %s", filename.c_str());
        return false;
    }
    
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (fileSize <= 0 || fileSize > 2 * 1024 * 1024) {
        LOG_ERROR_FMT("OusterFileLoader", "Invalid file size: %ld", fileSize);
        fclose(file);
        return false;
    }
    
    std::vector<uint8_t> buffer(fileSize);
    size_t read = fread(buffer.data(), 1, fileSize, file);
    fclose(file);
    
    if (read != static_cast<size_t>(fileSize)) {
        LOG_ERROR_FMT("OusterFileLoader", "Read error: got %zu, expected %ld", read, fileSize);
        return false;
    }
    
    LOG_INFO_FMT("OusterFileLoader", "Loaded %zu bytes", read);
    
    stream->GetLoadedPacketPoints().clear();
    stream->SetPacketLoadedFromFile(true);
    
    // Process packet
    ProcessLidarPacketFromFileManual(stream, buffer.data(), read);
    
    // Move queued frames to loaded points
    {
        std::lock_guard<std::mutex> lock(stream->GetPointCloudMutex());
        auto& queue = stream->GetPointCloudQueue();
        while (!queue.empty()) {
            auto& frame = queue.front();
            stream->GetLoadedPacketPoints().insert(
                stream->GetLoadedPacketPoints().end(),
                frame.first.begin(), frame.first.end()
            );
            stream->SetLoadedPacketTimestamp(frame.second);
            queue.pop();
        }
    }
    
    LOG_INFO_FMT("OusterFileLoader", "Extracted %zu points", stream->GetLoadedPacketPoints().size());
    
    stream->SetConnected(true);
    stream->SetStreaming(false);
    return true;
#else
    LOG_ERROR("OusterFileLoader", "Only macOS is currently supported");
    return false;
#endif
}

// ============================================================================
// ParseMetadataFileImpl - Parse metadata JSON
// ============================================================================

bool ParseMetadataFileImpl(OusterStream* stream, const std::string& metadataFile) {
    if (!stream) {
        LOG_ERROR("OusterFileLoader", "Stream is null");
        return false;
    }
    
    FILE* file = fopen(metadataFile.c_str(), "r");
    if (!file) {
        LOG_ERROR_FMT("OusterFileLoader", "Failed to open: %s", metadataFile.c_str());
        return false;
    }
    
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (fileSize <= 0 || fileSize > 1024 * 1024) {
        fclose(file);
        return false;
    }
    
    std::vector<char> buffer(fileSize + 1);
    size_t read = fread(buffer.data(), 1, fileSize, file);
    fclose(file);
    
    if (read != static_cast<size_t>(fileSize)) return false;
    buffer[fileSize] = '\0';
    
    std::string jsonStr(buffer.data());
    
    // Helper lambdas for parsing
    auto findField = [&jsonStr](const std::string& parent, const std::string& field) -> size_t {
        size_t parentPos = jsonStr.find("\"" + parent + "\"");
        if (parentPos == std::string::npos) {
            return jsonStr.find("\"" + field + "\"");
        }
        size_t braceStart = jsonStr.find('{', parentPos);
        if (braceStart == std::string::npos) return std::string::npos;
        
        int depth = 1;
        size_t pos = braceStart + 1;
        while (pos < jsonStr.length() && depth > 0) {
            if (jsonStr[pos] == '{') depth++;
            else if (jsonStr[pos] == '}') depth--;
            pos++;
        }
        size_t braceEnd = pos - 1;
        
        size_t fieldPos = jsonStr.find("\"" + field + "\"", braceStart);
        return (fieldPos != std::string::npos && fieldPos < braceEnd) ? fieldPos : std::string::npos;
    };
    
    auto extractString = [&jsonStr](size_t fieldPos) -> std::string {
        size_t colonPos = jsonStr.find(':', fieldPos);
        if (colonPos == std::string::npos) return "";
        size_t quoteStart = jsonStr.find('"', colonPos);
        if (quoteStart == std::string::npos) return "";
        size_t quoteEnd = jsonStr.find('"', quoteStart + 1);
        if (quoteEnd == std::string::npos) return "";
        return jsonStr.substr(quoteStart + 1, quoteEnd - quoteStart - 1);
    };
    
    auto extractNumber = [&jsonStr](size_t fieldPos) -> float {
        size_t colonPos = jsonStr.find(':', fieldPos);
        if (colonPos == std::string::npos) return 0.0f;
        size_t numStart = jsonStr.find_first_of("0123456789.-", colonPos);
        if (numStart == std::string::npos) return 0.0f;
        size_t numEnd = numStart;
        while (numEnd < jsonStr.length() && 
               (std::isdigit(jsonStr[numEnd]) || jsonStr[numEnd] == '.' || 
                jsonStr[numEnd] == 'e' || jsonStr[numEnd] == 'E' || 
                jsonStr[numEnd] == '+' || jsonStr[numEnd] == '-')) {
            numEnd++;
        }
        try { return std::stof(jsonStr.substr(numStart, numEnd - numStart)); }
        catch (...) { return 0.0f; }
    };
    
    auto parseArray = [&jsonStr](size_t fieldPos) -> std::vector<float> {
        std::vector<float> values;
        size_t arrayStart = jsonStr.find('[', fieldPos);
        if (arrayStart == std::string::npos) return values;
        size_t arrayEnd = jsonStr.find(']', arrayStart);
        if (arrayEnd == std::string::npos) return values;
        
        std::string arrayStr = jsonStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
        size_t pos = 0;
        while (pos < arrayStr.length()) {
            while (pos < arrayStr.length() && 
                   (arrayStr[pos] == ' ' || arrayStr[pos] == '\n' || 
                    arrayStr[pos] == ',' || arrayStr[pos] == '\t')) {
                pos++;
            }
            if (pos >= arrayStr.length()) break;
            
            size_t end = pos;
            while (end < arrayStr.length() && 
                   (std::isdigit(arrayStr[end]) || arrayStr[end] == '.' || 
                    arrayStr[end] == '-' || arrayStr[end] == '+' ||
                    arrayStr[end] == 'e' || arrayStr[end] == 'E')) {
                end++;
            }
            
            if (end > pos) {
                try { values.push_back(std::stof(arrayStr.substr(pos, end - pos))); }
                catch (...) {}
            }
            pos = end;
        }
        return values;
    };
    
    // Parse beam_altitude_angles
    size_t altPos = jsonStr.find("\"beam_altitude_angles\"");
    if (altPos != std::string::npos) {
        auto angles = parseArray(altPos);
        stream->GetBeamAltitudeAngles() = std::vector<float>(angles.begin(), angles.end());
        LOG_INFO_FMT("OusterFileLoader", "Parsed %zu altitude angles", stream->GetBeamAltitudeAngles().size());
    }
    
    // Parse beam_azimuth_angles
    size_t azPos = jsonStr.find("\"beam_azimuth_angles\"");
    if (azPos != std::string::npos) {
        auto angles = parseArray(azPos);
        stream->GetBeamAzimuthAngles() = std::vector<float>(angles.begin(), angles.end());
        LOG_INFO_FMT("OusterFileLoader", "Parsed %zu azimuth angles", stream->GetBeamAzimuthAngles().size());
    }
    
    // Parse UDP profile
    size_t profilePos = findField("data_format", "udp_profile_lidar");
    if (profilePos == std::string::npos) {
        profilePos = findField("lidar_data_format", "udp_profile_lidar");
    }
    
    if (profilePos != std::string::npos) {
        std::string profile = extractString(profilePos);
        stream->SetUdpProfileLidar(profile);
        LOG_INFO_FMT("OusterFileLoader", "UDP profile: %s", profile.c_str());
        
        if (profile == "LEGACY") {
            stream->SetPixelStride(12);
            stream->SetMeasurementBlockHeaderSize(0);  // LEGACY: 4 bytes are column footer, not block header
        } else if (profile == "RNG19_RFL8_SIG16_NIR16") {
            stream->SetPixelStride(8);
            stream->SetMeasurementBlockHeaderSize(4);
        } else if (profile == "RNG15_RFL8_NIR8") {
            stream->SetPixelStride(4);
            stream->SetMeasurementBlockHeaderSize(4);
        } else {
            stream->SetPixelStride(12);
            stream->SetMeasurementBlockHeaderSize(4);
        }
    } else {
        stream->SetUdpProfileLidar("LEGACY");
        stream->SetPixelStride(12);
        stream->SetMeasurementBlockHeaderSize(4);
    }
    
    // Parse range_unit
    size_t rangeUnitPos = findField("data_format", "range_unit");
    if (rangeUnitPos == std::string::npos) {
        rangeUnitPos = findField("lidar_data_format", "range_unit");
    }
    if (rangeUnitPos != std::string::npos) {
        float rangeUnit = extractNumber(rangeUnitPos);
        if (rangeUnit > 0.0f) {
            stream->SetRangeUnit(rangeUnit);
            LOG_INFO_FMT("OusterFileLoader", "Range unit: %.6f", rangeUnit);
        }
    }
    
    // Parse columns_per_packet
    size_t colsPacketPos = findField("data_format", "columns_per_packet");
    if (colsPacketPos == std::string::npos) {
        colsPacketPos = findField("lidar_data_format", "columns_per_packet");
    }
    if (colsPacketPos != std::string::npos) {
        int cols = static_cast<int>(extractNumber(colsPacketPos));
        if (cols > 0) {
            stream->SetColumnsPerPacket(cols);
            LOG_INFO_FMT("OusterFileLoader", "Columns per packet: %d", cols);
        }
    }
    
    // Parse columns_per_frame
    size_t colsFramePos = findField("data_format", "columns_per_frame");
    if (colsFramePos == std::string::npos) {
        colsFramePos = findField("lidar_data_format", "columns_per_frame");
    }
    if (colsFramePos != std::string::npos) {
        int cols = static_cast<int>(extractNumber(colsFramePos));
        if (cols > 0) {
            stream->SetColumnsPerFrame(cols);
            stream->SetEncoderTicksPerRev(cols == 1024 ? 90112 : cols * 88);
            LOG_INFO_FMT("OusterFileLoader", "Columns per frame: %d", cols);
        }
    }
    
    // Parse pixels_per_column
    size_t pixelsPos = findField("data_format", "pixels_per_column");
    if (pixelsPos == std::string::npos) {
        pixelsPos = findField("lidar_data_format", "pixels_per_column");
    }
    if (pixelsPos != std::string::npos) {
        int pixels = static_cast<int>(extractNumber(pixelsPos));
        if (pixels > 0) {
            stream->SetBeamCount(pixels);
            LOG_INFO_FMT("OusterFileLoader", "Pixels per column: %d", pixels);
        }
    }
    
    // Parse pixel_shift_by_row
    size_t shiftPos = jsonStr.find("\"pixel_shift_by_row\"");
    if (shiftPos != std::string::npos) {
        auto shifts = parseArray(shiftPos);
        stream->GetPixelShiftByRow().clear();
        for (float s : shifts) {
            stream->GetPixelShiftByRow().push_back(static_cast<int>(s));
        }
        LOG_INFO_FMT("OusterFileLoader", "Parsed %zu pixel shifts", stream->GetPixelShiftByRow().size());
    }
    
    // Parse lidar_origin_to_beam_origin_mm
    size_t originPos = jsonStr.find("\"lidar_origin_to_beam_origin_mm\"");
    if (originPos != std::string::npos) {
        float origin = extractNumber(originPos);
        stream->SetLidarOriginToBeamOriginMm(origin);
        LOG_INFO_FMT("OusterFileLoader", "Lidar origin to beam: %.3f mm", origin);
    }
    
    // Parse transforms
    size_t beamTransformPos = jsonStr.find("\"beam_to_lidar_transform\"");
    if (beamTransformPos != std::string::npos) {
        auto values = parseArray(beamTransformPos);
        if (values.size() == 16) {
            std::memcpy(stream->GetBeamToLidarTransform(), values.data(), 16 * sizeof(float));
            LOG_INFO("OusterFileLoader", "Parsed beam_to_lidar_transform");
        }
    }
    
    size_t lidarTransformPos = jsonStr.find("\"lidar_to_sensor_transform\"");
    if (lidarTransformPos != std::string::npos) {
        auto values = parseArray(lidarTransformPos);
        if (values.size() == 16) {
            std::memcpy(stream->GetLidarToSensorTransform(), values.data(), 16 * sizeof(float));
            LOG_INFO("OusterFileLoader", "Parsed lidar_to_sensor_transform");
        }
    }
    
    return !stream->GetBeamAltitudeAngles().empty() && !stream->GetBeamAzimuthAngles().empty();
}

// ============================================================================
// ProcessLidarPacketFromFileImpl
// ============================================================================

void ProcessLidarPacketFromFileImpl(OusterStream* stream, const uint8_t* data, size_t size) {
    if (!stream) return;
    ProcessLidarPacketFromFileManual(stream, data, size);
}

} // namespace OusterFileLoader
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
