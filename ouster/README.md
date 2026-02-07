# Ouster LiDAR Setup Guide

## Sensor Info

| Property | Value |
|----------|-------|
| Model | OS-0-128 |
| Serial | 122212001833 |
| Sensor IP | 172.17.1.56 |
| Mac IP | 172.17.1.100 |
| Lidar Port | 7510 |
| IMU Port | 7513 |

## Quick Start

### 1. Configure Network

```bash
cd /Users/rgrigorii/Desktop/projects/camera_system/LidarSDKWrapper/ouster
bash setup_ouster_network.sh
```

This assigns `172.17.1.100` to your Ethernet adapter so you can reach the sensor at `172.17.1.56`.

### 2. Verify Connection

```bash
ping 172.17.1.56
curl http://172.17.1.56/api/v1/sensor/metadata
```

## Running Live LiDAR in camera_system

Due to macOS firewall restrictions on unsigned binaries, we use a Python UDP bridge.

### Terminal 1: Start the UDP Bridge

```bash
cd /Users/rgrigorii/Desktop/projects/camera_system/LidarSDKWrapper/ouster
python3 udp_bridge.py
```

The bridge listens on UDP port 7510 and forwards packets via Unix socket to the C++ app.

### Terminal 2: Run the Application

```bash
cd /Users/rgrigorii/Desktop/projects/camera_system/build
./bin/camera_system_editor \
  --ouster-metadata ../LidarSDKWrapper/ouster/example/metadata.json \
  --lidar-config ../config/lidar.config
```

Then enable "Lidar Live" in the UI.

## Get Fresh Metadata

If point cloud looks wrong, fetch metadata directly from your sensor:

```bash
curl http://172.17.1.56/api/v1/sensor/metadata > fresh_metadata.json
```

Then use `--ouster-metadata fresh_metadata.json`

## Configuration Files

| File | Purpose |
|------|---------|
| `config/lidar.config` | Range filtering, frame retention, downsampling |
| `example/metadata.json` | Sensor calibration data for XYZ transformation |

### lidar.config Options

```ini
retain_frames = 0       # 0 = show current frame only, N = accumulate N frames
min_range = 0.5         # Filter points closer than this (meters)
max_range = 100.0       # Filter points farther than this (meters)
downsample_factor = 1   # 1 = all points, 2 = every 2nd point, etc.
```

## Troubleshooting

### No data received
1. Check bridge is running: `python3 udp_bridge.py`
2. Verify sensor is streaming: `curl http://172.17.1.56/api/v1/sensor/config | grep udp`
3. Check network: `ping 172.17.1.56`

### Points only show close range
- Fetch fresh metadata from sensor (see above)
- Check sensor orientation - is it blocked by nearby objects?

### "No data for X seconds" in logs
- Sensor may have gone idle - access web UI to wake it
- Check UDP bridge is receiving packets

## Alternative: View with ouster-cli

```bash
ouster-cli source 172.17.1.56 viz
```

## Useful Links

- [Sensor Web UI](http://172.17.1.56)
- [Ouster SDK Docs](https://static.ouster.dev/sdk-docs/)
