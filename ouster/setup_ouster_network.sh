#!/bin/bash
# Ouster LiDAR Network Setup for macOS
#
# This script configures your Mac's network interface to communicate with
# an Ouster sensor and sets the sensor's UDP destination.
#
# Sensor: 172.17.1.56 (static IP)
# Mac:    172.17.1.100 (assigned to Ethernet interface)

set -e

# Configuration
SENSOR_IP="${OUSTER_SENSOR_IP:-172.17.1.56}"
MAC_IP="172.17.1.100"
NETMASK="255.255.255.0"
LIDAR_PORT=7510
IMU_PORT=7511

echo "=========================================="
echo "Ouster LiDAR Network Setup"
echo "=========================================="
echo "Sensor: $SENSOR_IP"
echo "Mac:    $MAC_IP"
echo ""

# Find Ethernet interface (prefer en8, then en0-en7)
find_interface() {
    for iface in en8 en0 en1 en2 en3 en4 en5 en6 en7; do
        if ifconfig "$iface" 2>/dev/null | grep -q "status: active"; then
            echo "$iface"
            return 0
        fi
    done
    echo ""
}

INTERFACE=$(find_interface)
if [ -z "$INTERFACE" ]; then
    echo "✗ No active Ethernet interface found"
    echo "  Connect Ethernet cable and try again"
    exit 1
fi
echo "Interface: $INTERFACE"

# Configure IP address
if ifconfig "$INTERFACE" | grep -q "inet $MAC_IP"; then
    echo "✓ IP already configured"
else
    echo "Configuring IP..."
    sudo ifconfig "$INTERFACE" "$MAC_IP" netmask "$NETMASK"
    echo "✓ IP configured"
fi

# Test sensor connectivity
echo ""
echo "Testing sensor connection..."
if curl -s -m 3 "http://$SENSOR_IP/api/v1/system/info" > /dev/null 2>&1; then
    echo "✓ Sensor reachable"
    
    # Configure sensor UDP destination
    echo "Configuring sensor UDP..."
    curl -s -X POST -H 'Content-Type: application/json' \
        -d "{\"udp_dest\": \"$MAC_IP\", \"udp_port_lidar\": $LIDAR_PORT, \"udp_port_imu\": $IMU_PORT}" \
        "http://$SENSOR_IP/api/v1/sensor/config" > /dev/null
    echo "✓ Sensor configured: UDP -> $MAC_IP:$LIDAR_PORT"
else
    echo "⚠ Cannot reach sensor at $SENSOR_IP"
    echo "  Check cable and power"
fi

# Summary
echo ""
echo "=========================================="
echo "Ready!"
echo "=========================================="
echo ""
echo "To stream live data:"
echo "  python3 setup_ouster.py --live --metadata example/metadata.json"
echo ""
