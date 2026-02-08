#!/usr/bin/env python3
"""
Ouster LiDAR PCAP Capture and Visualization

Captures live lidar data to PCAP file and metadata, or loads and plots existing PCAP files.
"""

import subprocess
import socket
import time
import sys
import os
import json
import argparse
from typing import Optional, Tuple
import select

# ============================================================================
# Configuration
# ============================================================================

OUSTER_IP = "172.17.1.56"
MAC_IP = "172.17.1.100"
LIDAR_PORT = 7510
IMU_PORT = 7511

# ============================================================================
# SDK and Library Availability
# ============================================================================

_script_dir = os.path.dirname(os.path.abspath(__file__))

def _setup_sdk_path():
    """Find and configure Ouster SDK path."""
    global _sdk_python_path
    _sdk_python_path = None
    
    # Try pip-installed SDK first
    try:
        import ouster.sdk
        _sdk_python_path = os.path.dirname(os.path.dirname(ouster.sdk.__file__))
        print(f"✓ Found Ouster SDK (pip) at: {_sdk_python_path}")
        return
    except ImportError:
        pass
    
    # Fallback: local SDK paths
    sdk_paths = [
        os.path.join(_script_dir, "ouster-sdk", "build", f"py{v}", "ouster-sdk")
        for v in ["3.14.2", "3.13.2", "3.12.2", "3.11.2", "3.9.6"]
    ] + [
        os.path.join(_script_dir, "ouster-sdk", "python", "src"),
        os.path.join(_script_dir, "ouster-sdk", "python"),
        os.path.join(_script_dir, "ouster-sdk"),
    ]
    
    for path in sdk_paths:
        if os.path.exists(path):
            ouster_sdk_path = os.path.join(path, "ouster", "sdk")
            if os.path.exists(ouster_sdk_path) or os.path.exists(os.path.join(path, "ouster")):
                if path not in sys.path:
                    sys.path.insert(0, path)
                _sdk_python_path = path
                print(f"✓ Found Ouster SDK (local) at: {path}")
                return
    
    print("⚠ Ouster SDK not found. Install with: pip install ouster-sdk")

_sdk_python_path = None
_setup_sdk_path()

# Check dependencies
HAS_NUMPY = False
try:
    import numpy as np
    HAS_NUMPY = True
    print("✓ numpy available")
except ImportError:
    print("⚠ numpy required: pip install numpy")

HAS_OUSTER_SDK = False
SDK_IMPORT_ERROR = None

if _sdk_python_path and HAS_NUMPY:
    try:
        from ouster.sdk import pcap, core
        from ouster.sdk.core import LidarScan, ChanField, SensorInfo
        HAS_OUSTER_SDK = True
        print("✓ Ouster SDK imported")
    except ImportError as e:
        SDK_IMPORT_ERROR = str(e)
        print(f"⚠ SDK import failed: {e}")

# Optional visualization libraries
HAS_OPEN3D = False
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    pass

HAS_MATPLOTLIB = False
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    pass

OUSTER_CLI_AVAILABLE = False
try:
    result = subprocess.run(["which", "ouster-cli"], capture_output=True, timeout=2)
    OUSTER_CLI_AVAILABLE = (result.returncode == 0)
except:
    pass

# ============================================================================
# Helper Functions
# ============================================================================

def find_file(filename: str, search_dirs: list = None) -> str:
    """Find a file in common locations. Returns the first match or original filename."""
    if os.path.exists(filename):
        return filename
    
    if search_dirs is None:
        search_dirs = [_script_dir, os.path.join(_script_dir, "example")]
    
    for dir_path in search_dirs:
        path = os.path.join(dir_path, os.path.basename(filename))
        if os.path.exists(path):
            return path
    
    return filename


def load_sensor_info(metadata_file: str, sensor_ip: str = None) -> 'SensorInfo':
    """Load SensorInfo from file or fetch from sensor."""
    from ouster.sdk.core import SensorInfo
    
    if os.path.exists(metadata_file):
        with open(metadata_file, 'r') as f:
            return SensorInfo(f.read())
    
    if sensor_ip:
        try:
            import urllib.request
            url = f"http://{sensor_ip}/api/v1/sensor/metadata"
            with urllib.request.urlopen(url, timeout=5) as response:
                return SensorInfo(response.read().decode('utf-8'))
        except Exception as e:
            raise RuntimeError(f"Failed to fetch metadata from sensor: {e}")
    
    raise FileNotFoundError(f"Metadata file not found: {metadata_file}")


def snap_to_grid(x, y, z, intensity, grid_resolution: float):
    """Snap points to a grid by averaging points within each grid cell."""
    if len(x) == 0:
        return x, y, z, intensity
    
    x_cell = np.floor(x / grid_resolution).astype(int)
    y_cell = np.floor(y / grid_resolution).astype(int)
    z_cell = np.floor(z / grid_resolution).astype(int)
    
    grid_dict = {}
    for i in range(len(x)):
        key = (x_cell[i], y_cell[i], z_cell[i])
        if key not in grid_dict:
            grid_dict[key] = {'x': [], 'y': [], 'z': [], 'i': []}
        grid_dict[key]['x'].append(x[i])
        grid_dict[key]['y'].append(y[i])
        grid_dict[key]['z'].append(z[i])
        grid_dict[key]['i'].append(intensity[i])
    
    x_g = [np.mean(v['x']) for v in grid_dict.values()]
    y_g = [np.mean(v['y']) for v in grid_dict.values()]
    z_g = [np.mean(v['z']) for v in grid_dict.values()]
    i_g = [np.mean(v['i']) for v in grid_dict.values()]
    
    return np.array(x_g), np.array(y_g), np.array(z_g), np.array(i_g)


def filter_points_by_range(x, y, z, ranges, min_range: float = None, max_range: float = None):
    """Filter points by range."""
    valid = ranges > 0
    if min_range is not None:
        valid &= (ranges >= min_range)
    if max_range is not None:
        valid &= (ranges <= max_range)
    return x[valid], y[valid], z[valid], valid

# ============================================================================
# Visualization Functions
# ============================================================================

def render_point_cloud(x, y, z, intensity, title: str = "", point_size: float = 1.0, color_mode: str = 'white'):
    """Render point cloud using available library."""
    if HAS_OPEN3D:
        _render_open3d(x, y, z, intensity, title)
    elif HAS_MATPLOTLIB:
        _render_matplotlib(x, y, z, intensity, title, point_size, color_mode)
    else:
        print("✗ No visualization library: pip install matplotlib or open3d")


def _render_matplotlib(x, y, z, intensity, title: str, point_size: float = 1.0, color_mode: str = 'white'):
    """Render using matplotlib."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    if color_mode == 'height':
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
        colors = plt.cm.turbo(z_norm)
    elif color_mode == 'distance':
        dist = np.sqrt(x**2 + y**2 + z**2)
        d_norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-6)
        colors = plt.cm.viridis(d_norm)
    else:
        colors = 'white'
    
    ax.scatter(x, y, z, c=colors, s=point_size, alpha=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title or f'Point Cloud ({len(x):,} points)')
    ax.set_facecolor('black')
    plt.tight_layout()
    plt.show()


def _render_open3d(x, y, z, intensity, title: str):
    """Render using Open3D."""
    points = np.column_stack((x, y, z))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(np.ones((len(points), 3)))  # White
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title or "Point Cloud", width=1400, height=900)
    vis.add_geometry(pcd)
    
    opt = vis.get_render_option()
    opt.background_color = np.array([0.1, 0.1, 0.15])
    opt.point_size = 2.0
    
    vis.run()
    vis.destroy_window()

# ============================================================================
# PCAP Loading
# ============================================================================

def load_and_plot_pcap(pcap_file: str, metadata_file: str = None, 
                       grid_resolution: float = None, max_range: float = None, min_range: float = None):
    """Load a PCAP file and plot the point cloud."""
    if not HAS_OUSTER_SDK:
        print(f"✗ Ouster SDK required: {SDK_IMPORT_ERROR}")
        sys.exit(1)
    
    if not HAS_MATPLOTLIB and not HAS_OPEN3D:
        print("✗ Install matplotlib or open3d for visualization")
        sys.exit(1)
    
    from ouster.sdk import pcap
    from ouster.sdk.core import SensorInfo, XYZLut, ChanField
    
    # Resolve paths
    pcap_file = find_file(pcap_file)
    if not os.path.exists(pcap_file):
        print(f"✗ PCAP not found: {pcap_file}")
        sys.exit(1)
    
    if metadata_file is None:
        metadata_file = os.path.join(_script_dir, "metadata.json")
    metadata_file = find_file(metadata_file)
    
    if not os.path.exists(metadata_file):
        print(f"✗ Metadata not found: {metadata_file}")
        sys.exit(1)
    
    print(f"Loading: {pcap_file}")
    print(f"Metadata: {metadata_file}")
    
    # Load metadata
    with open(metadata_file, 'r') as f:
        meta = json.load(f)
    
    range_unit = meta.get("data_format", {}).get("range_unit")
    if range_unit is None:
        range_unit = meta.get("lidar_data_format", {}).get("range_unit", 0.001)
    print(f"  Range unit: {range_unit} m/count")
    
    with open(metadata_file, 'r') as f:
        info = SensorInfo(f.read())
    print(f"  Sensor: {info.prod_line} (SN: {info.sn})")
    
    # Open PCAP
    src = pcap.PcapScanSource(pcap_file, sensor_info=[info])
    
    # Get first scan
    scan_data = None
    for scan_set in src:
        if scan_set and len(scan_set) > 0:
            scan_data = scan_set
            break
    
    if not scan_data or not scan_data[0]:
        print("✗ No scans found in PCAP")
        src.close()
        sys.exit(1)
    
    scan = scan_data[0]
    print(f"  Scan: {scan.w}x{scan.h}")
    
    # Extract XYZ
    xyzlut = XYZLut(info)
    xyz = xyzlut(scan.field(ChanField.RANGE))
    ranges_m = scan.field(ChanField.RANGE).astype(float) * range_unit
    intensity = scan.field(ChanField.SIGNAL)
    
    x = xyz[:, :, 0].flatten()
    y = xyz[:, :, 1].flatten()
    z = xyz[:, :, 2].flatten()
    i = intensity.flatten()
    ranges_flat = ranges_m.flatten()
    
    # Filter
    x, y, z, valid = filter_points_by_range(x, y, z, ranges_flat, min_range, max_range)
    i = i[valid]
    
    print(f"  Extracted {len(x):,} valid points")
    src.close()
    
    # Grid snap
    if grid_resolution and grid_resolution > 0:
        x, y, z, i = snap_to_grid(x, y, z, i, grid_resolution)
        print(f"  Grid-snapped to {len(x):,} points")
    
    # Render
    render_point_cloud(x, y, z, i, f"PCAP: {len(x):,} points")

# ============================================================================
# Live Streaming
# ============================================================================

def stream_live(sensor_ip: str = None, metadata_file: str = None, lidar_port: int = None,
                grid_resolution: float = None, max_range: float = None, min_range: float = None,
                accumulate_frames: int = 1, force_matplotlib: bool = False,
                point_size: float = 1.0, color_mode: str = 'white'):
    """Stream live LiDAR data using raw UDP sockets with SDK parsing."""
    
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required: pip install ouster-sdk")
        sys.exit(1)
    
    if not HAS_MATPLOTLIB:
        print("✗ matplotlib required: pip install matplotlib")
        sys.exit(1)
    
    from ouster.sdk.core import (LidarScan, ChanField, SensorInfo, XYZLut, 
                                  ScanBatcher, LidarPacket, PacketFormat)
    
    sensor_ip = sensor_ip or OUSTER_IP
    lidar_port = lidar_port or LIDAR_PORT
    
    print("=" * 50)
    print("   Ouster LiDAR Live Streaming")
    print("=" * 50)
    print(f"Sensor: {sensor_ip}:{lidar_port}")
    print(f"Accumulate: {accumulate_frames} frame(s)")
    print()
    
    # Load metadata
    if metadata_file is None:
        metadata_file = os.path.join(_script_dir, "example", "metadata.json")
    
    try:
        info = load_sensor_info(metadata_file, sensor_ip)
        print(f"  ✓ Metadata: {info.prod_line} SN:{info.sn}")
    except Exception as e:
        print(f"✗ {e}")
        sys.exit(1)
    
    # Create SDK objects
    pf = PacketFormat.from_info(info)
    batcher = ScanBatcher(info)
    xyzlut = XYZLut(info)
    
    print(f"  Packet size: {pf.lidar_packet_size} bytes")
    print(f"  Resolution: {info.format.columns_per_frame}x{info.format.pixels_per_column}")
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(('0.0.0.0', lidar_port))
    sock.setblocking(False)
    
    print(f"  ✓ Listening on UDP {lidar_port}")
    print("\nWaiting for data... (Ctrl+C or close window to exit)")
    print()
    
    # Setup matplotlib
    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('black')
    fig.patch.set_facecolor('black')
    
    zoom_state = {'scale': 1.0}
    def on_scroll(event):
        if event.inaxes == ax:
            zoom_state['scale'] *= 1.2 if event.button == 'down' else 1/1.2
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    print(f"  Controls: Drag=rotate, Scroll=zoom | Color: {color_mode}, Size: {point_size}")
    
    # State
    current_points = None
    frame_count = 0
    packet_count = 0
    last_update = time.time()
    
    scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
    packet = LidarPacket(pf.lidar_packet_size)
    
    try:
        while plt.fignum_exists(fig.number):
            # Drain packets
            for _ in range(200):
                ready, _, _ = select.select([sock], [], [], 0.001)
                if not ready:
                    break
                
                try:
                    data, _ = sock.recvfrom(65535)
                    packet_count += 1
                    
                    if len(data) != pf.lidar_packet_size:
                        continue
                    
                    packet.buf[:] = np.frombuffer(data, dtype=np.uint8)
                    
                    if batcher(packet, scan):
                        frame_count += 1
                        
                        xyz = xyzlut(scan.field(ChanField.RANGE))
                        ranges = scan.field(ChanField.RANGE).astype(np.float32) * 0.001
                        
                        x = xyz[:, :, 0].flatten()
                        y = xyz[:, :, 1].flatten()
                        z = xyz[:, :, 2].flatten()
                        r = ranges.flatten()
                        
                        x, y, z, valid = filter_points_by_range(x, y, z, r, min_range, max_range)
                        
                        if len(x) > 0:
                            new_pts = np.column_stack((x, y, z))
                            
                            if accumulate_frames <= 1:
                                current_points = new_pts
                            else:
                                current_points = new_pts if current_points is None else np.vstack([current_points, new_pts])
                                max_pts = accumulate_frames * 100000
                                if len(current_points) > max_pts:
                                    current_points = current_points[-max_pts:]
                            
                            if frame_count % 10 == 0:
                                dist = np.sqrt(x**2 + y**2 + z**2)
                                print(f"  Frame {frame_count}: {len(x):,} pts, [{dist.min():.1f}, {dist.max():.1f}] m")
                        
                        scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
                        
                except socket.error:
                    break
            
            plt.pause(0.001)
            
            # Update visualization
            now = time.time()
            if current_points is not None and now - last_update >= 0.1:
                last_update = now
                ax.clear()
                
                pts = current_points
                if len(pts) > 50000:
                    pts = pts[np.random.choice(len(pts), 50000, replace=False)]
                
                if color_mode == 'height':
                    z_norm = (pts[:, 2] - pts[:, 2].min()) / (pts[:, 2].max() - pts[:, 2].min() + 1e-6)
                    colors = plt.cm.turbo(z_norm)
                elif color_mode == 'distance':
                    dist = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2)
                    d_norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-6)
                    colors = plt.cm.viridis(d_norm)
                else:
                    colors = 'white'
                
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors, s=point_size, alpha=0.8)
                
                dist_full = np.sqrt(current_points[:, 0]**2 + current_points[:, 1]**2 + current_points[:, 2]**2)
                ax.set_title(f'Frame {frame_count} | {len(current_points):,} pts | {dist_full.min():.1f}-{dist_full.max():.1f}m',
                            color='white', fontsize=10)
                ax.set_facecolor('black')
                ax.set_xlabel('X', color='white')
                ax.set_ylabel('Y', color='white')
                ax.set_zlabel('Z', color='white')
                ax.tick_params(colors='white')
                
                lim = max(np.percentile(np.abs(pts), 95) * 1.2, 3.0) * zoom_state['scale']
                ax.set_xlim(-lim, lim)
                ax.set_ylim(-lim, lim)
                ax.set_zlim(-lim/2, lim/2)
                
                plt.draw()
    
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        sock.close()
        plt.close()
    
    print(f"\nProcessed {frame_count} frames, {packet_count} packets")

# ============================================================================
# PCAP Capture (for completeness)
# ============================================================================

def capture_pcap(hostname: str, lidar_port: int, timeout: float = 15.0) -> Optional[str]:
    """Capture packets and save as PCAP file."""
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required for capture")
        return None
    
    print("Capturing PCAP...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pcap_file = os.path.join(script_dir, "ouster.pcap")
    metadata_file = os.path.join(script_dir, "metadata.json")
    
    if OUSTER_CLI_AVAILABLE:
        try:
            cmd = ["ouster-cli", "source", hostname, "slice", "0s:10s", "save", pcap_file]
            subprocess.run(cmd, timeout=timeout, capture_output=True)
            
            if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                print(f"  ✓ Captured: {pcap_file}")
                return pcap_file
        except Exception as e:
            print(f"  ⚠ ouster-cli failed: {e}")
    
    print("  ✗ Capture failed")
    return None

# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Ouster LiDAR PCAP Capture and Visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 setup_ouster.py --live
  python3 setup_ouster.py --live --sensor-ip 169.254.183.253 --max-range 10
  python3 setup_ouster.py --load-packet ouster.pcap --metadata metadata.json
"""
    )
    
    parser.add_argument('--load-packet', '--load_packet', nargs='?', const='ouster.pcap',
                        metavar='PCAP', dest='load_packet', help='Load and plot PCAP file')
    parser.add_argument('--metadata', metavar='FILE', help='Metadata JSON file')
    parser.add_argument('--max-range', '--max_range', type=float, help='Max range filter (m)')
    parser.add_argument('--min-range', '--min_range', type=float, help='Min range filter (m)')
    parser.add_argument('--grid-resolution', '--grid_resolution', type=float, dest='grid_resolution',
                        help='Grid snap resolution (m)')
    parser.add_argument('--live', action='store_true', help='Stream live data')
    parser.add_argument('--sensor-ip', '--sensor_ip', metavar='IP', dest='sensor_ip',
                        help=f'Sensor IP (default: {OUSTER_IP})')
    parser.add_argument('--port', type=int, help=f'UDP port (default: {LIDAR_PORT})')
    parser.add_argument('--accumulate', type=int, default=1, help='Frames to accumulate')
    parser.add_argument('--no-open3d', action='store_true', dest='no_open3d', help='Force matplotlib')
    parser.add_argument('--point-size', '--point_size', type=float, default=1.0, help='Point size')
    parser.add_argument('--color', choices=['white', 'height', 'distance'], default='white', help='Color mode')
    
    args = parser.parse_args()
    
    if args.live:
        stream_live(
            sensor_ip=args.sensor_ip,
            metadata_file=args.metadata,
            lidar_port=args.port,
            grid_resolution=args.grid_resolution,
            max_range=args.max_range,
            min_range=args.min_range,
            accumulate_frames=args.accumulate,
            force_matplotlib=args.no_open3d,
            point_size=args.point_size,
            color_mode=args.color
        )
        return
    
    if args.load_packet is not None:
        load_and_plot_pcap(args.load_packet, args.metadata, args.grid_resolution,
                          max_range=args.max_range, min_range=args.min_range)
        return
    
    # Default: capture
    pcap_file = capture_pcap(OUSTER_IP, LIDAR_PORT)
    if pcap_file:
        print(f"\n✓ PCAP: {pcap_file}")
        print(f"To visualize: python3 setup_ouster.py --load-packet {os.path.basename(pcap_file)}")


if __name__ == "__main__":
    main()
