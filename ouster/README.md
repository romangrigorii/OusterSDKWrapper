# Ouster LiDAR Setup Guide

## Your Sensor
| Property | Value |
|----------|-------|
| Model | OS-0-128 |
| Serial | 122212001833 |
| Firmware | v2.3.1 |
| IP Address | **172.17.1.56** |
| Web UI | http://172.17.1.56 |

## Quick Start

### 1. Connect the Ouster
- Power the Type-3 interface box (12V@3A or 24V@1.5A)
- Connect RJ45 Ethernet to your USB-C adapter

### 2. Run the Setup Script
```bash
bash /Users/rgrigorii/Desktop/projects/ouster/setup_ouster_network.sh
```

This adds the `172.17.1.100` IP to your Ethernet adapter so you can reach the sensor.

### 3. Verify Connection
```bash
ping 172.17.1.56
```

### 4. Access Web UI
Open in browser: http://172.17.1.56

### 5. View Live Point Cloud
```bash
ouster-cli source 172.17.1.56 viz
```

**Note:** Your sensor has firmware v2.3.1 which is slightly older. Some CLI features require v2.4+.

## Recording Data

### Save PCAP (raw packets)
```bash
ouster-cli source 172.17.1.56 save data.pcap
```

### Replay recorded data
```bash
ouster-cli source data.pcap viz
```

## Network Configuration

Your setup uses dual IPs to avoid conflicts with your home Wi-Fi:

| Interface | IP | Subnet | Purpose |
|-----------|-----|--------|---------|
| Wi-Fi (en0) | DHCP | 192.168.1.x | Internet (NETGEAR) |
| Ethernet (en8) | 172.17.1.100 | 172.17.1.x | Ouster sensor |

**Important:** The Ethernet IP doesn't persist after reboot. Run `setup_ouster_network.sh` each time you connect the Ouster.

## Troubleshooting

### Internet stops working when Ouster is connected
Run the setup script to configure the network properly:
```bash
bash setup_ouster_network.sh
```

### Can't reach the sensor
1. Check cable connection
2. Verify Ethernet link: `ifconfig en8 | grep status`
3. Discover sensors: `ouster-cli discover`

### CLI shows "firmware version not supported"
Your sensor has v2.3.1. Some CLI features require v2.4+. Use the web UI instead, or update firmware from http://172.17.1.56

## Python API Example

```python
from ouster import client

sensor_hostname = "172.17.1.56"

with client.Sensor(sensor_hostname) as sensor:
    info = sensor.metadata
    print(f"Product: {info.prod_line}")
    print(f"Serial: {info.sn}")
    
    for scan in client.Scans(sensor):
        range_field = scan.field(client.ChanField.RANGE)
        print(f"Scan timestamp: {scan.timestamp}")
        break
```

## Useful Links
- [Sensor Web UI](http://172.17.1.56)
- [Ouster SDK Documentation](https://static.ouster.dev/sdk-docs/)
- [Ouster GitHub](https://github.com/ouster-lidar/ouster-sdk)
