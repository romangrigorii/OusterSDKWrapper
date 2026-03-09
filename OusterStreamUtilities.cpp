// ============================================================================
// OusterStreamUtilities.cpp - Port management utilities
// ============================================================================

#include "OusterStreamUtilities.h"
#include "platform/Logger.h"
#ifdef _WIN32
#include <windows.h>
using pid_t = DWORD;
static inline pid_t ouster_getpid() { return static_cast<pid_t>(GetCurrentProcessId()); }
#else
#include <unistd.h>
#define ouster_getpid getpid
#endif
#include <cstdio>
#include <cstring>
#include <vector>
#include <thread>
#include <chrono>

namespace Engine {
namespace Editors {
namespace CameraSystem {
namespace OusterUtilities {

bool KillProcessesOnPort(int port) {
    pid_t current_pid = ouster_getpid();
    bool ports_freed = false;

#ifdef _WIN32
    (void)port;
    (void)current_pid;
    ports_freed = true;  // No port-kill implementation on Windows; assume port is free
#elif defined(__APPLE__)
    char cmd[512];
    snprintf(cmd, sizeof(cmd), 
        "for pid in $(lsof -ti :%d 2>/dev/null); do if [ \"$pid\" != \"%d\" ]; then kill -9 $pid 2>/dev/null; fi; done", 
        port, current_pid);
    (void)system(cmd);  // Ignore return value - we check port status separately
    
    // Wait a moment for ports to be released
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Check if port is now free and get details about what's using it
    // First check if current process is using the port
    char currentPidCheckCmd[256];
    snprintf(currentPidCheckCmd, sizeof(currentPidCheckCmd), "lsof -ti :%d 2>/dev/null | grep '^%d$' | wc -l", port, current_pid);
    FILE* currentPidFp = popen(currentPidCheckCmd, "r");
    bool currentProcessUsingPort = false;
    if (currentPidFp) {
        char buf[128];
        if (fgets(buf, sizeof(buf), currentPidFp) != nullptr) {
            int count = atoi(buf);
            currentProcessUsingPort = (count > 0);
        }
        pclose(currentPidFp);
    }
    
    // Check for other processes using the port
    char checkCmd[256];
    snprintf(checkCmd, sizeof(checkCmd), "lsof -ti :%d 2>/dev/null | grep -v '^%d$'", port, current_pid);
    FILE* fp = popen(checkCmd, "r");
    if (fp) {
        char buf[128];
        std::vector<pid_t> remaining_pids;
        while (fgets(buf, sizeof(buf), fp) != nullptr) {
            pid_t pid = static_cast<pid_t>(atoi(buf));
            if (pid > 0 && pid != current_pid) {
                remaining_pids.push_back(pid);
            }
        }
        pclose(fp);
        ports_freed = remaining_pids.empty();
        
        if (currentProcessUsingPort) {
            LOG_WARNING_FMT("OusterStream", "Port %d is being used by current process (PID %d) - waiting for SDK to release...", port, current_pid);
            // Current process is holding the port - this means a previous SDK connection hasn't fully released
            // Wait longer for the SDK to release the ports
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            // Check again
            snprintf(currentPidCheckCmd, sizeof(currentPidCheckCmd), "lsof -ti :%d 2>/dev/null | grep '^%d$' | wc -l", port, current_pid);
            currentPidFp = popen(currentPidCheckCmd, "r");
            if (currentPidFp) {
                char buf2[128];
                if (fgets(buf2, sizeof(buf2), currentPidFp) != nullptr) {
                    int count = atoi(buf2);
                    currentProcessUsingPort = (count > 0);
                }
                pclose(currentPidFp);
            }
            if (!currentProcessUsingPort) {
                LOG_INFO_FMT("OusterStream", "Port %d released by current process after waiting", port);
            } else {
                LOG_WARNING_FMT("OusterStream", "Port %d still held by current process - may need longer wait or manual cleanup", port);
            }
        }
        
        if (!ports_freed) {
            // Get more details about what's using the port
            char detailCmd[512];
            snprintf(detailCmd, sizeof(detailCmd), "lsof -i :%d 2>/dev/null | grep -v '^%d ' | tail -1", port, current_pid);
            FILE* detailFp = popen(detailCmd, "r");
            if (detailFp) {
                char detailBuf[512];
                if (fgets(detailBuf, sizeof(detailBuf), detailFp) != nullptr) {
                    // Remove trailing newline
                    size_t len = strlen(detailBuf);
                    if (len > 0 && detailBuf[len-1] == '\n') {
                        detailBuf[len-1] = '\0';
                    }
                    LOG_WARNING_FMT("OusterStream", "Port %d still in use: %s", port, detailBuf);
                }
                pclose(detailFp);
            }
            LOG_WARNING_FMT("OusterStream", "Port %d is still in use by %zu process(es)", port, remaining_pids.size());
            for (pid_t pid : remaining_pids) {
                // Check if process actually exists
                char psCmd[128];
                snprintf(psCmd, sizeof(psCmd), "ps -p %d -o comm= 2>/dev/null", pid);
                FILE* psFp = popen(psCmd, "r");
                if (psFp) {
                    char psBuf[128];
                    if (fgets(psBuf, sizeof(psBuf), psFp) != nullptr) {
                        size_t len = strlen(psBuf);
                        if (len > 0 && psBuf[len-1] == '\n') {
                            psBuf[len-1] = '\0';
                        }
                        LOG_WARNING_FMT("OusterStream", "  - PID %d: %s (may be zombie or in TIME_WAIT)", pid, psBuf);
                    } else {
                        LOG_WARNING_FMT("OusterStream", "  - PID %d: (process not found - likely TIME_WAIT socket)", pid);
                    }
                    pclose(psFp);
                }
            }
        }
    }
    
    // Port is free if no other processes are using it AND current process has released it
    ports_freed = ports_freed && !currentProcessUsingPort;
    
    if (ports_freed) {
        LOG_INFO_FMT("OusterStream", "Port %d is now free", port);
    } else {
        if (currentProcessUsingPort) {
            LOG_WARNING_FMT("OusterStream", "Port %d still held by current process (previous SDK connection not fully released)", port);
        } else {
            LOG_WARNING_FMT("OusterStream", "Port %d may still be in use after cleanup (may be TIME_WAIT state)", port);
        }
    }
#elif __linux__
    // Linux version using fuser or lsof, exclude current PID
    char cmd[512];
    snprintf(cmd, sizeof(cmd), 
        "fuser -k %d/udp 2>/dev/null || (for pid in $(lsof -ti :%d 2>/dev/null); do if [ \"$pid\" != \"%d\" ]; then kill -9 $pid 2>/dev/null; fi; done) || true", 
        port, port, current_pid);
    (void)system(cmd);  // Ignore return value - we check port status separately
    
    // Wait a moment for ports to be released
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Check if port is now free and get details about what's using it
    char checkCmd[256];
    snprintf(checkCmd, sizeof(checkCmd), "lsof -ti :%d 2>/dev/null | grep -v '^%d$'", port, current_pid);
    FILE* fp = popen(checkCmd, "r");
    if (fp) {
        char buf[128];
        std::vector<pid_t> remaining_pids;
        while (fgets(buf, sizeof(buf), fp) != nullptr) {
            pid_t pid = static_cast<pid_t>(atoi(buf));
            if (pid > 0 && pid != current_pid) {
                remaining_pids.push_back(pid);
            }
        }
        pclose(fp);
        ports_freed = remaining_pids.empty();
        
        if (!ports_freed) {
            // Get more details about what's using the port
            char detailCmd[512];
            snprintf(detailCmd, sizeof(detailCmd), "lsof -i :%d 2>/dev/null | grep -v '^%d ' | tail -1", port, current_pid);
            FILE* detailFp = popen(detailCmd, "r");
            if (detailFp) {
                char detailBuf[512];
                if (fgets(detailBuf, sizeof(detailBuf), detailFp) != nullptr) {
                    // Remove trailing newline
                    size_t len = strlen(detailBuf);
                    if (len > 0 && detailBuf[len-1] == '\n') {
                        detailBuf[len-1] = '\0';
                    }
                    LOG_WARNING_FMT("OusterStream", "Port %d still in use: %s", port, detailBuf);
                }
                pclose(detailFp);
            }
            LOG_WARNING_FMT("OusterStream", "Port %d is still in use by %zu process(es)", port, remaining_pids.size());
        }
    }
    
    if (ports_freed) {
        LOG_INFO_FMT("OusterStream", "Port %d is now free", port);
    } else {
        LOG_WARNING_FMT("OusterStream", "Port %d may still be in use after cleanup (may be TIME_WAIT state)", port);
    }
#endif
    
    return ports_freed;
}

bool VerifyPortIsFree(int port) {
#ifdef __APPLE__
    // NOTE: We no longer do a test bind because it creates TIME_WAIT state
    // which prevents the actual bind later. Instead, just check if any
    // process is actively listening on the port using lsof.
    
    char checkCmd[256];
    snprintf(checkCmd, sizeof(checkCmd), "lsof -ti :%d 2>/dev/null | wc -l", port);
    FILE* fp = popen(checkCmd, "r");
    if (fp) {
        char buf[128];
        if (fgets(buf, sizeof(buf), fp) != nullptr) {
            int count = atoi(buf);
            pclose(fp);
            // Port is free if no processes are using it
            return (count == 0);
        }
        pclose(fp);
    }
    
    // If we can't check, assume port is free (let the actual bind fail if not)
    return true;
#else
    // On other platforms, just return true (assume port is free)
    return true;
#endif
}

} // namespace OusterUtilities
} // namespace CameraSystem
} // namespace Editors
} // namespace Engine
