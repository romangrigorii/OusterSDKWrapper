#!/bin/bash
# Script to configure Mac network for Ouster LiDAR sensor
# Sensor IP: 172.17.1.56
# Mac IP: 172.17.1.100

set -e

SENSOR_IP="172.17.1.56"
MAC_IP="172.17.1.100"
SUBNET="172.17.1.0"
NETMASK="255.255.255.0"
LIDAR_PORT=7504
IMU_PORT=7503

echo "=========================================="
echo "Ouster LiDAR Network Configuration"
echo "=========================================="
echo "Sensor IP: $SENSOR_IP"
echo "Mac IP:    $MAC_IP"
echo ""

# Function to find the active Ethernet interface
find_ethernet_interface() {
    # Try to find Ethernet interface (en*)
    # Prefer en8, en0, or any active Ethernet interface
    local interfaces=$(ifconfig | grep -E "^[a-z0-9]+:" | grep -v "lo0" | awk '{print $1}' | sed 's/://')
    
    for iface in en8 en0 en1 en2 en3 en4 en5 en6 en7; do
        if echo "$interfaces" | grep -q "^${iface}$"; then
            # Check if it's an Ethernet interface (not Wi-Fi)
            local media=$(ifconfig "$iface" 2>/dev/null | grep "media:" | head -1)
            if [[ "$media" == *"autoselect"* ]] || [[ "$media" == *"10baseT"* ]] || [[ "$media" == *"100baseTX"* ]] || [[ "$media" == *"1000baseT"* ]]; then
                echo "$iface"
                return 0
            fi
        fi
    done
    
    # Fallback: return first non-loopback interface
    echo "$interfaces" | head -1
}

# Find Ethernet interface
INTERFACE=$(find_ethernet_interface)

if [ -z "$INTERFACE" ]; then
    echo "✗ Error: Could not find Ethernet interface"
    echo "  Please connect an Ethernet cable to your Mac"
    exit 1
fi

echo "Using interface: $INTERFACE"
echo ""

# Check if interface already has the correct IP
CURRENT_IP=$(ifconfig "$INTERFACE" | grep "inet " | awk '{print $2}' | head -1)

if [ "$CURRENT_IP" == "$MAC_IP" ]; then
    echo "✓ Interface $INTERFACE already configured with IP $MAC_IP"
else
    echo "Configuring interface $INTERFACE with IP $MAC_IP..."
    
    # Remove existing IP configuration if present
    sudo ifconfig "$INTERFACE" down 2>/dev/null || true
    
    # Configure interface with static IP
    sudo ifconfig "$INTERFACE" "$MAC_IP" netmask "$NETMASK" up
    
    if [ $? -eq 0 ]; then
        echo "✓ Interface $INTERFACE configured successfully"
    else
        echo "✗ Error: Failed to configure interface"
        exit 1
    fi
fi

# Verify configuration
echo ""
echo "Verifying network configuration..."
sleep 1

NEW_IP=$(ifconfig "$INTERFACE" | grep "inet " | awk '{print $2}' | head -1)
if [ "$NEW_IP" == "$MAC_IP" ]; then
    echo "✓ IP address verified: $NEW_IP"
else
    echo "⚠ Warning: IP address mismatch (expected $MAC_IP, got $NEW_IP)"
fi

# Test connectivity to sensor
echo ""
echo "Testing connectivity to sensor at $SENSOR_IP..."
if ping -c 2 -W 2000 "$SENSOR_IP" > /dev/null 2>&1; then
    echo "✓ Sensor is reachable"
else
    echo "⚠ Warning: Cannot ping sensor at $SENSOR_IP"
    echo "  - Check Ethernet cable connection"
    echo "  - Verify sensor is powered on"
    echo "  - Check sensor IP configuration"
fi

# Check if UDP ports are accessible (optional - just informational)
echo ""
echo "Checking UDP port availability..."
if lsof -i :$LIDAR_PORT > /dev/null 2>&1; then
    echo "⚠ Port $LIDAR_PORT (lidar) is in use"
    lsof -i :$LIDAR_PORT | head -2
else
    echo "✓ Port $LIDAR_PORT (lidar) is available"
fi

if lsof -i :$IMU_PORT > /dev/null 2>&1; then
    echo "⚠ Port $IMU_PORT (IMU) is in use"
    lsof -i :$IMU_PORT | head -2
else
    echo "✓ Port $IMU_PORT (IMU) is available"
fi

# Display network configuration summary
echo ""
echo "=========================================="
echo "Network Configuration Summary"
echo "=========================================="
echo "Interface:    $INTERFACE"
echo "Mac IP:       $MAC_IP"
echo "Sensor IP:    $SENSOR_IP"
echo "Subnet:       $SUBNET/$NETMASK"
echo "Lidar Port:   $LIDAR_PORT"
echo "IMU Port:     $IMU_PORT"
echo ""
echo "✓ Network configuration complete!"
echo ""
echo "Note: This configuration is temporary and will be lost after reboot."
echo "      Run this script again after restarting your Mac."
echo ""
echo "To test the connection:"
echo "  ping $SENSOR_IP"
echo "  curl http://$SENSOR_IP/api/v1/system/info"
echo ""
