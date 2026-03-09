# Ouster LiDAR Network Setup for Windows
#
# Adds static IPs to the ETHERNET adapter only so you can reach the Ouster.
# Wi-Fi is NEVER touched - your internet stays intact.
#
# Adds two IPs to Ethernet (so sensor is reachable on either subnet):
#   - 172.17.1.100  (sensor 172.17.1.56)
#   - 192.168.168.100 (sensor 192.168.168.1, Ouster factory default)
# Then tries to configure the sensor to send UDP to this PC.
#
# Run: .\setup_ouster_network.ps1

# Self-elevate if not running as Administrator (shows UAC prompt)
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "Requesting administrator rights... A NEW WINDOW will open (UAC). Check that window for output."
    Start-Process powershell.exe -ArgumentList "-NoProfile -ExecutionPolicy Bypass -NoExit -File `"$PSCommandPath`"" -Verb RunAs
    exit
}

# First thing when running as admin: show we started (so window isn't blank)
Write-Host "Ouster network setup (running as Administrator)..." -ForegroundColor Cyan
Write-Host ""

$ErrorActionPreference = "Stop"
try {

$LIDAR_PORT = 7510
$IMU_PORT   = 7511

# Subnets we add to Ethernet only (sensor might be on either)
$SUBNETS = @(
    @{ PC_IP = "172.17.1.100";   SENSOR_IP = "172.17.1.56";   Name = "172.17.1.x" },
    @{ PC_IP = "192.168.168.100"; SENSOR_IP = "192.168.168.1"; Name = "192.168.168.x (Ouster default)" }
)

# Exclude Wi-Fi / Wireless - we never touch your internet connection
$excludePattern = "Wi-Fi|Wireless|WLAN|802\.11|Wireless LAN|WiFi|Bluetooth"

$allPhysical = Get-NetAdapter -Physical | Where-Object {
    $_.Status -eq "Up" -and
    $_.InterfaceDescription -notmatch "Loopback|Virtual|VPN" -and
    $_.InterfaceDescription -notmatch $excludePattern -and
    $_.Name -notmatch $excludePattern
}
$adapters = $allPhysical | Sort-Object {
    if ($_.Name -match "Ethernet" -or $_.InterfaceDescription -match "Ethernet") { 0 } else { 1 }
}

if (-not $adapters -or $adapters.Count -eq 0) {
    Write-Host "No Ethernet adapter found (Wi-Fi is intentionally skipped)." -ForegroundColor Red
    Write-Host "Connect the Ethernet cable from your PC to the Ouster, then run this script again." -ForegroundColor Yellow
    exit 1
}

Write-Host "=========================================="
Write-Host "Ouster LiDAR Network Setup (Windows)"
Write-Host "=========================================="
Write-Host "Wi-Fi will NOT be changed." -ForegroundColor Green
Write-Host ""

$adapter = $adapters[0]
$ifIndex = $adapter.ifIndex
Write-Host "Configuring Ethernet only: $($adapter.Name)" -ForegroundColor Cyan
Write-Host ""

# Add each subnet to Ethernet (adds IPs; does not remove existing like 169.254.x.x)
foreach ($sub in $SUBNETS) {
    $existing = Get-NetIPAddress -InterfaceIndex $ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.IPAddress -eq $sub.PC_IP }
    if ($existing) {
        Write-Host "  $($sub.Name): $($sub.PC_IP) already set." -ForegroundColor Gray
    } else {
        try {
            New-NetIPAddress -InterfaceIndex $ifIndex -IPAddress $sub.PC_IP -PrefixLength 24 -ErrorAction Stop | Out-Null
            Write-Host "  $($sub.Name): added $($sub.PC_IP)" -ForegroundColor Green
        } catch {
            if ($_.Exception.Message -match "already exists") {
                Write-Host "  $($sub.Name): $($sub.PC_IP) already set." -ForegroundColor Gray
            } else {
                Write-Host "  $($sub.Name): failed - $_" -ForegroundColor Yellow
            }
        }
    }
}

Write-Host ""
Write-Host "Looking for sensor (will configure UDP destination when found) ..."

$sensorFound = $null
foreach ($sub in $SUBNETS) {
    try {
        $r = Invoke-WebRequest -Uri "http://$($sub.SENSOR_IP)/api/v1/system/info" -UseBasicParsing -TimeoutSec 3 -ErrorAction Stop
        Write-Host "  Sensor found at $($sub.SENSOR_IP) ($($sub.Name))." -ForegroundColor Green
        $sensorFound = $sub
        break
    } catch {
        Write-Host "  No reply from $($sub.SENSOR_IP)." -ForegroundColor Gray
    }
}

if ($sensorFound) {
    try {
        $body = @{ udp_dest = $sensorFound.PC_IP; udp_port_lidar = $LIDAR_PORT; udp_port_imu = $IMU_PORT } | ConvertTo-Json
        Invoke-WebRequest -Uri "http://$($sensorFound.SENSOR_IP)/api/v1/sensor/config" -Method POST -Body $body -ContentType "application/json" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop | Out-Null
        Write-Host "  Sensor config sent: UDP -> $($sensorFound.PC_IP):$LIDAR_PORT (lidar), $IMU_PORT (imu)." -ForegroundColor Green
        # Reinitialize so the sensor actually applies the new UDP destination and starts streaming
        try {
            Invoke-WebRequest -Uri "http://$($sensorFound.SENSOR_IP)/api/v1/sensor/cmd/reinitialize" -Method GET -UseBasicParsing -TimeoutSec 15 -ErrorAction Stop | Out-Null
            Write-Host "  Sensor reinitialized (UDP streaming active)." -ForegroundColor Green
        } catch {
            Write-Host "  Reinitialize failed (sensor may still apply config): $_" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "  Could not set sensor UDP config: $_" -ForegroundColor Yellow
    }
} else {
    Write-Host "  Sensor not reachable on 172.17.1.56 or 192.168.168.1. Check cable and power." -ForegroundColor Yellow
    Write-Host "  If the sensor is on 169.254.x.x (link-local), run: python setup_ouster.py --check" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Firewall: If you get no data in live view, allow inbound UDP ${LIDAR_PORT} and ${IMU_PORT}:"
Write-Host "  netsh advfirewall firewall add rule name=\"Ouster Lidar\" dir=in action=allow protocol=UDP localport=$LIDAR_PORT"
Write-Host "  netsh advfirewall firewall add rule name=\"Ouster IMU\" dir=in action=allow protocol=UDP localport=$IMU_PORT"
Write-Host ""
Write-Host "=========================================="
Write-Host "Done (Wi-Fi unchanged)"
Write-Host "=========================================="
if ($sensorFound) {
    Write-Host "Stream live: python setup_ouster.py --live --sensor-ip $($sensorFound.SENSOR_IP)"
} else {
    Write-Host "Stream live: python setup_ouster.py --live --sensor-ip <sensor-ip>"
}
Write-Host ""

} catch {
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "At: $($_.ScriptStackTrace)" -ForegroundColor Gray
}

Read-Host "Press Enter to close this window"
