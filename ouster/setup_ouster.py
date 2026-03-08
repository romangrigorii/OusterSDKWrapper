#!/usr/bin/env python3
"""Ouster LiDAR: capture, live stream (browser), and view PCAP. Run with --live for browser viewer."""

import subprocess
import socket
import time
import sys
import os
import json
import argparse
import threading
import webbrowser
from typing import Optional, Tuple
from http.server import HTTPServer, BaseHTTPRequestHandler
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

def _install_local_sdk_editable(verbose=True):
    """Install local ouster-sdk in editable mode (builds C++ extensions)."""
    python_dir = os.path.join(_script_dir, "ouster-sdk", "python")
    if not os.path.isfile(os.path.join(python_dir, "pyproject.toml")):
        return False
    python_dir_abs = os.path.abspath(python_dir)
    try:
        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", "-e", python_dir_abs],
            capture_output=not verbose,
            timeout=300,
            text=True,
        )
        if not verbose and result.returncode != 0 and result.stderr:
            print(result.stderr.strip()[:500])
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("  Install timed out (build can take several minutes).")
        return False
    except FileNotFoundError as ex:
        print(f"  pip not found: {ex}")
        return False

def _patch_metadata_version_for_local_sdk():
    """Patch importlib.metadata.version so ouster.sdk can import from source without being installed."""
    import importlib.metadata as _metadata
    _orig = getattr(_metadata, "version", None)
    if _orig is None:
        return False
    def _patched(package_name):
        if package_name == "ouster-sdk":
            return "0.0.0+local"
        return _orig(package_name)
    _metadata.version = _patched
    return True

if _sdk_python_path and HAS_NUMPY:
    try:
        from ouster.sdk import pcap, core
        from ouster.sdk.core import LidarScan, ChanField, SensorInfo
        HAS_OUSTER_SDK = True
        print("✓ Ouster SDK imported")
    except ImportError as e:
        SDK_IMPORT_ERROR = str(e)
        err_msg = str(e).lower()
        python_dir = os.path.join(_script_dir, "ouster-sdk", "python")
        python_dir_abs = os.path.abspath(python_dir)

        # C++ extensions (_bindings) are missing – SDK must be built with pip install -e
        if "_bindings" in err_msg and _sdk_python_path:
            print("  The Ouster Python SDK requires built C++ extensions (_bindings).")
            print("  Build and install from the project root with:")
            print(f"    pip install -e \"{python_dir_abs}\"")
            print("  Requirements: CMake, C++ compiler (e.g. Visual Studio Build Tools on Windows),")
            print("  and vcpkg with libzip if the C++ SDK was built with vcpkg.")
            print("  Alternatively install the pre-built package: pip install ouster-sdk")
        # Local src on path but package not installed -> no metadata for "ouster-sdk"
        elif ("package metadata" in err_msg or "packagenotfound" in err_msg) and _sdk_python_path:
            if os.path.isfile(os.path.join(python_dir, "pyproject.toml")):
                print("  Installing local Ouster SDK in editable mode (builds C++ extensions)...")
                if _install_local_sdk_editable():
                    try:
                        from ouster.sdk import pcap, core
                        from ouster.sdk.core import LidarScan, ChanField, SensorInfo
                        HAS_OUSTER_SDK = True
                        SDK_IMPORT_ERROR = None
                        print("✓ Ouster SDK imported (after editable install)")
                    except ImportError as e2:
                        SDK_IMPORT_ERROR = str(e2)
                        err2 = str(e2).lower()
                        if "_bindings" in err2:
                            print("  Build completed but extensions not found. Try running the pip command above from the project root.")
                        print(f"⚠ SDK import still failed: {e2}")
                else:
                    print("  Editable install failed (see pip errors above). From project root run:")
                    print(f"    pip install -e \"{python_dir_abs}\"")
        if not HAS_OUSTER_SDK:
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
    if sys.platform == "win32":
        # On Windows, ouster-cli may be in Scripts; prefer Python SDK capture
        import shutil
        OUSTER_CLI_AVAILABLE = (shutil.which("ouster-cli") is not None)
    else:
        result = subprocess.run(["which", "ouster-cli"], capture_output=True, timeout=2)
        OUSTER_CLI_AVAILABLE = (result.returncode == 0)
except Exception:
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
# Connectivity check (debug)
# ============================================================================

def _get_ethernet_link_local_ip():
    """If the Ethernet adapter has a 169.254.x.x address, return (ip_octet2, ip_octet3) e.g. (74, 254) for 169.254.74.254. Else return None."""
    if sys.platform != "win32":
        return None
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.InterfaceAlias -notmatch 'Wi-Fi|Wireless|Bluetooth|Loopback|Virtual|VPN' -and $_.IPAddress -like '169.254.*' } | ForEach-Object { $_.IPAddress }"],
            capture_output=True, text=True, timeout=10)
        if r.returncode != 0 or not r.stdout.strip():
            return None
        # First 169.254.x.x (likely Ethernet)
        for line in r.stdout.strip().splitlines():
            line = line.strip()
            if line.startswith("169.254."):
                parts = line.split(".")
                if len(parts) == 4:
                    try:
                        return (int(parts[2]), int(parts[3]))
                    except ValueError:
                        pass
                break
    except Exception:
        pass
    return None


def _probe_sensor_http(ip: str, timeout: float = 2.0) -> bool:
    """Return True if sensor metadata API responds at ip."""
    try:
        from urllib.request import urlopen, Request
        req = Request(f"http://{ip}/api/v1/sensor/metadata/sensor_info", method="GET")
        with urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", errors="ignore")
            return "prod_line" in body or "lidar_origin_to_beam_origin_mm" in body
    except Exception:
        return False


def _probe_ping(ip: str, timeout: int = 3) -> bool:
    """Return True if host replies to ping."""
    try:
        ping_cmd = ["ping", "-n", "1", "-w", str(timeout * 1000), ip] if sys.platform == "win32" else ["ping", "-c", "1", "-W", str(timeout), ip]
        r = subprocess.run(ping_cmd, capture_output=True, text=True, timeout=timeout + 2)
        return r.returncode == 0
    except Exception:
        return False


def run_connectivity_check(sensor_ip: str, lidar_port: int, imu_port: int):
    """Print network and sensor connectivity info to debug 'Timeout' / no data."""
    import re
    import socket
    print("========================================")
    print("  Ouster connectivity check")
    print("========================================")
    print(f"  Sensor: {sensor_ip}")
    print(f"  UDP ports: lidar {lidar_port}, IMU {imu_port}")
    print()
    
    # 1) PC IPs
    print("1. Your PC's IPv4 addresses (one should be in same subnet as sensor):")
    ethernet_link_local = None
    try:
        if sys.platform == "win32":
            r = subprocess.run(
                ["powershell", "-NoProfile", "-Command",
                 "Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' } | ForEach-Object { $_.InterfaceAlias + \": \" + $_.IPAddress + \"/\" + $_.PrefixLength }"],
                capture_output=True, text=True, timeout=10)
            if r.stdout.strip():
                for line in r.stdout.strip().splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    print("  ", line)
                    # Detect Ethernet with 169.254 (link-local)
                    if "Ethernet" in line and "169.254." in line:
                        m = re.search(r"169\.254\.(\d+)\.(\d+)", line)
                        if m:
                            ethernet_link_local = (int(m.group(1)), int(m.group(2)))
            else:
                print("  (run: ipconfig)")
        else:
            r = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5)
            if r.returncode == 0 and r.stdout.strip():
                print(" ", r.stdout.strip())
            else:
                print("  (run: ip addr or ifconfig)")
    except Exception as e:
        print(f"  Could not list IPs: {e}")
    if ethernet_link_local is None:
        ethernet_link_local = _get_ethernet_link_local_ip()
    print()
    
    # 2) Subnet mismatch: Ethernet is link-local (169.254.x.x) but we're trying 172.17.1.56
    if ethernet_link_local and not sensor_ip.startswith("169.254."):
        a, b = ethernet_link_local
        print("  *** ETHERNET IS LINK-LOCAL (169.254.x.x) ***")
        print(f"  Your Ethernet has 169.254.{a}.{b} (no DHCP = direct cable to sensor).")
        print(f"  The sensor is then also on 169.254.x.x, NOT {sensor_ip}.")
        print("  Probing common link-local sensor IPs ...")
        candidates = [
            f"169.254.{a}.1",   # same /24 as PC, .1
            f"169.254.1.1",
            f"169.254.{a}.2",
            "169.254.74.1",    # match your 169.254.74.254
        ]
        seen = set()
        for ip in candidates:
            if ip in seen:
                continue
            seen.add(ip)
            if _probe_ping(ip, timeout=2):
                print(f"     Ping OK: {ip}")
                if _probe_sensor_http(ip, timeout=2):
                    print(f"     HTTP OK: {ip} <- USE THIS")
                    print()
                    print("  >>> Use this sensor IP:")
                    print(f"      python setup_ouster.py --live --sensor-ip {ip}")
                    print(f"      python setup_ouster.py --check --sensor-ip {ip}")
                    print()
                    sensor_ip = ip  # use for section 3 so we show OK
                    break
            else:
                print(f"     No reply: {ip}")
        else:
            print("  No link-local IP responded. Try: python setup_ouster.py --discover")
        print()
    
    # 3) Ping configured sensor
    print("2. Ping sensor:")
    try:
        ping_cmd = ["ping", "-n", "2", sensor_ip] if sys.platform == "win32" else ["ping", "-c", "2", sensor_ip]
        r = subprocess.run(ping_cmd, capture_output=True, text=True, timeout=15)
        if r.returncode == 0:
            print("   OK - sensor replies to ping")
        else:
            print("   FAIL - no reply. Check cable, sensor power, and same subnet (see above).")
    except Exception as e:
        print(f"   Error: {e}")
    print()
    
    # 4) HTTP to sensor
    print("3. HTTP to sensor (metadata API):")
    try:
        if _probe_sensor_http(sensor_ip, timeout=5):
            print("   OK - sensor metadata reachable")
        else:
            from urllib.request import urlopen, Request
            req = Request(f"http://{sensor_ip}/api/v1/sensor/metadata/sensor_info", method="GET")
            with urlopen(req, timeout=5) as resp:
                body = resp.read().decode("utf-8", errors="ignore")
                if "prod_line" in body or "lidar_origin_to_beam_origin_mm" in body:
                    print("   OK - sensor metadata reachable")
                else:
                    print("   Response received but unexpected content")
    except socket.timeout:
        print("   TIMEOUT - sensor not reachable on HTTP. Same subnet? (Ethernet link-local = use --sensor-ip 169.254.x.x)")
    except OSError as e:
        print(f"   FAIL - {e}")
    except Exception as e:
        print(f"   Error: {type(e).__name__}: {e}")
    print()
    
    # 5) Firewall
    print("4. Firewall: allow UDP in on ports", lidar_port, "and", imu_port)
    print("   Windows: Windows Defender Firewall -> Advanced -> Inbound rules -> New rule -> Port -> UDP", lidar_port, imu_port)
    print()
    print("5. If sensor IP differs, run: python setup_ouster.py --discover")
    print("   Then: python setup_ouster.py --live --sensor-ip <IP>")
    print("========================================")


# ============================================================================
# Live Streaming (Web viewer - simple, fast, no Open3D)
# ============================================================================

_WEB_LATEST = {"points": None, "point_size": 0.1}  # shared for /data
_WEB_LOCK = threading.Lock()

OUSTER_WEB_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Ouster Live</title><style>body{margin:0;background:#0a0a0f;}</style></head>
<body><script src="https://unpkg.com/three@0.158.0/build/three.min.js"></script>
<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0a0f);
const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.1, 1000);
camera.position.set(20, 0, 5);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
document.body.appendChild(renderer.domElement);
let rotY = 0, rotX = 0.2, zoom = 1, drag = false, prevX = 0, prevY = 0;
renderer.domElement.onmousedown = () => drag = true;
renderer.domElement.onmouseup = () => drag = false;
renderer.domElement.onmouseleave = () => drag = false;
renderer.domElement.onmousemove = (e) => { if (!drag) return; rotY -= (e.clientX - prevX) * 0.005; rotX += (e.clientY - prevY) * 0.005; prevX = e.clientX; prevY = e.clientY; };
renderer.domElement.onwheel = (e) => { e.preventDefault(); zoom *= e.deltaY > 0 ? 0.9 : 1.1; };
const pointsGeometry = new THREE.BufferGeometry();
const pointsMaterial = new THREE.PointsMaterial({ size: 0.1, color: 0xffffff, sizeAttenuation: true });
const points = new THREE.Points(pointsGeometry, pointsMaterial);
scene.add(points);
window.addEventListener('resize', () => { camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
function updatePoints(arr) {
  if (!arr || arr.length === 0) return;
  const f = new Float32Array(arr.length * 3);
  for (let i = 0; i < arr.length; i++) { f[i*3]=arr[i][0]; f[i*3+1]=arr[i][1]; f[i*3+2]=arr[i][2]; }
  pointsGeometry.setAttribute('position', new THREE.BufferAttribute(f, 3));
  pointsGeometry.attributes.position.needsUpdate = true;
  pointsGeometry.computeBoundingSphere();
}
function poll() { fetch('/data').then(r=>r.json()).then(d=>{ if(d.points) updatePoints(d.points); if(d.point_size!=null) pointsMaterial.size=d.point_size; }).catch(()=>{}); }
setInterval(poll, 80);
function animate() {
  requestAnimationFrame(animate);
  const r = 25 * zoom;
  camera.position.x = r * Math.cos(rotX) * Math.sin(rotY);
  camera.position.y = r * Math.sin(rotX);
  camera.position.z = r * Math.cos(rotX) * Math.cos(rotY);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld();
  renderer.render(scene, camera);
}
renderer.domElement.addEventListener('mousemove', (e) => { prevX = e.clientX; prevY = e.clientY; });
animate();
</script></body></html>
"""


class _OusterWebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(OUSTER_WEB_HTML.encode("utf-8"))
        elif self.path == "/data":
            with _WEB_LOCK:
                data = json.dumps(_WEB_LATEST).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def stream_live_web(sensor_ip: str = None, metadata_file: str = None, lidar_port: int = None,
                    max_range: float = None, min_range: float = None, accumulate_frames: int = 1,
                    grid_resolution: float = None, point_size: float = 0.1,
                    web_port: int = 8765, max_display_pts: int = 25000):
    """Stream live LiDAR to a browser (Three.js). No Open3D/matplotlib. Simple and fast."""
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required: pip install ouster-sdk")
        sys.exit(1)
    from ouster.sdk.core import LidarScan, ChanField, ScanBatcher, LidarPacket, PacketFormat, XYZLut

    with _WEB_LOCK:
        _WEB_LATEST["point_size"] = point_size
    sensor_ip = sensor_ip or OUSTER_IP
    lidar_port = lidar_port or LIDAR_PORT
    if metadata_file is None:
        metadata_file = os.path.join(_script_dir, "example", "metadata.json")
    try:
        info = load_sensor_info(metadata_file, sensor_ip)
    except Exception as e:
        print(f"✗ {e}")
        sys.exit(1)

    pf = PacketFormat.from_info(info)
    batcher = ScanBatcher(info)
    xyzlut = XYZLut(info)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", lidar_port))
    sock.setblocking(False)

    server = HTTPServer(("127.0.0.1", web_port), _OusterWebHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    url = f"http://127.0.0.1:{web_port}/"
    print("Ouster LiDAR Live")
    print(f"  Sensor {sensor_ip}:{lidar_port}  →  {url}")
    webbrowser.open(url)

    current_points = None
    frame_count = 0
    packet_count = 0
    scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
    packet = LidarPacket(pf.lidar_packet_size)
    last_warn = time.time()

    try:
        while True:
            # Drain socket until empty so we don't drop packets (full scan = ~64 packets/frame)
            drain_cap = 2000
            while drain_cap > 0:
                drain_cap -= 1
                ready, _, _ = select.select([sock], [], [], 0.0)
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
                            if grid_resolution and grid_resolution > 0:
                                try:
                                    intensity = scan.field(ChanField.SIGNAL).flatten()[valid]
                                    x, y, z, _ = snap_to_grid(x, y, z, intensity, grid_resolution)
                                except Exception:
                                    pass
                            new_pts = np.column_stack((x, y, z))
                            if accumulate_frames <= 1:
                                current_points = new_pts
                            else:
                                current_points = new_pts if current_points is None else np.vstack([current_points, new_pts])
                                if len(current_points) > accumulate_frames * 100000:
                                    current_points = current_points[-accumulate_frames * 100000:]
                            pts = current_points
                            if len(pts) > max_display_pts:
                                pts = pts[np.random.choice(len(pts), max_display_pts, replace=False)]
                            with _WEB_LOCK:
                                _WEB_LATEST["points"] = pts.tolist()
                                _WEB_LATEST["point_size"] = point_size
                            if frame_count == 1:
                                print(f"  {len(pts):,} pts → browser")
                        scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
                except socket.error:
                    break
            # Slow work (update browser) only after draining
            if packet_count == 0 and (time.time() - last_warn) >= 5:
                print("  No UDP packets yet. Run setup_ouster_network.ps1; allow firewall UDP 7510/7511.")
                last_warn = time.time()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()
        server.shutdown()
    print(f"  Frames: {frame_count}, packets: {packet_count}")


# ============================================================================
# Live Streaming (Open3D / matplotlib)
# ============================================================================

def _points_to_open3d_colors(pts: np.ndarray, color_mode: str) -> np.ndarray:
    """Return Nx3 RGB in [0,1] for Open3D point cloud. No matplotlib dependency for white."""
    n = len(pts)
    if color_mode == 'white':
        return np.ones((n, 3), dtype=np.float64)
    # height or distance: use simple gradient if no matplotlib, else colormap
    try:
        import matplotlib.cm
        if color_mode == 'height':
            z = pts[:, 2]
            z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
            cmap = matplotlib.cm.get_cmap('turbo')
            return cmap(z_norm)[:, :3].astype(np.float64)
        if color_mode == 'distance':
            dist = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2)
            d_norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-6)
            cmap = matplotlib.cm.get_cmap('viridis')
            return cmap(d_norm)[:, :3].astype(np.float64)
    except (ImportError, AttributeError):
        pass
    return np.ones((n, 3), dtype=np.float64)


def stream_live(sensor_ip: str = None, metadata_file: str = None, lidar_port: int = None,
                grid_resolution: float = None, max_range: float = None, min_range: float = None,
                accumulate_frames: int = 1, force_matplotlib: bool = False,
                point_size: float = 1.0, color_mode: str = 'white'):
    """Stream live to desktop window (Open3D or matplotlib). Use --live without --desktop for browser."""
    
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required: pip install ouster-sdk")
        sys.exit(1)
    
    use_open3d = HAS_OPEN3D and not force_matplotlib
    if not use_open3d and not HAS_MATPLOTLIB:
        print("✗ For live view install Open3D (recommended): pip install open3d")
        print("  Or install matplotlib and run with: --no-open3d (may freeze)")
        sys.exit(1)
    if not use_open3d:
        print("  Using matplotlib (may be slow; install open3d for smooth live view)")
    
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
    
    # State
    current_points = None
    frame_count = 0
    packet_count = 0
    last_update = time.time()
    last_packet_log = time.time()
    no_packet_warn_sec = 5.0  # warn if no packets for this long
    scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
    packet = LidarPacket(pf.lidar_packet_size)
    
    if use_open3d:
        # Open3D: create window and point cloud once, update in place (smooth, non-blocking)
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Ouster Live", width=1280, height=800)
        pcd = o3d.geometry.PointCloud()
        # Start with a single point so geometry exists
        pcd.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0]], dtype=np.float64))
        pcd.colors = o3d.utility.Vector3dVector(np.array([[1, 1, 1]], dtype=np.float64))
        vis.add_geometry(pcd)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.08])
        opt.point_size = max(2.0, point_size)  # 2.0 so points are visible at range
        print(f"  Backend: Open3D | Color: {color_mode} | Point size: {point_size}")
        
        max_display_pts = 80000  # cap for responsiveness
        view_fitted = False  # reset camera when first points arrive
        
        try:
            while True:
                # Drain socket until empty so we don't drop packets (full scan = ~64 packets/frame)
                drain_cap = 2000
                while drain_cap > 0:
                    drain_cap -= 1
                    ready, _, _ = select.select([sock], [], [], 0.0)
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
                                new_pts = np.column_stack((x, y, z)).astype(np.float64)
                                if accumulate_frames <= 1:
                                    current_points = new_pts
                                else:
                                    current_points = new_pts if current_points is None else np.vstack([current_points, new_pts])
                                    max_pts = accumulate_frames * 100000
                                    if len(current_points) > max_pts:
                                        current_points = current_points[-max_pts:]
                                if frame_count % 20 == 0:
                                    dist = np.sqrt(x**2 + y**2 + z**2)
                                    print(f"  Frame {frame_count}: {len(x):,} pts, [{dist.min():.1f}, {dist.max():.1f}] m")
                            scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, info.format.udp_profile_lidar)
                    except socket.error:
                        break
                # Slow work (render) only after draining
                now = time.time()
                if packet_count == 0 and (now - last_packet_log) >= no_packet_warn_sec:
                    print(f"  No UDP packets on port {lidar_port} yet ({no_packet_warn_sec:.0f}s). Check: 1) Run setup_ouster_network.ps1 so sensor sends to this PC. 2) Firewall allows inbound UDP {lidar_port}/{lidar_port + 1}. 3) Sensor IP: {sensor_ip}")
                    last_packet_log = now
                elif packet_count > 0 and frame_count == 0 and (now - last_packet_log) >= 5.0:
                    print(f"  Receiving packets ({packet_count}) but no complete frame yet. Use metadata from this sensor: --metadata <path> or fetch from sensor.")
                    last_packet_log = now
                
                # Update Open3D geometry
                if current_points is not None and len(current_points) > 0:
                    pts = current_points
                    if len(pts) > max_display_pts:
                        pts = pts[np.random.choice(len(pts), max_display_pts, replace=False)]
                    pcd.points = o3d.utility.Vector3dVector(pts)
                    pcd.colors = o3d.utility.Vector3dVector(_points_to_open3d_colors(pts, color_mode))
                    vis.update_geometry(pcd)
                    # Fit view to point cloud the first time we have data (camera starts at origin)
                    if not view_fitted:
                        view_fitted = True
                        center = np.mean(pts, axis=0)
                        try:
                            ctr = vis.get_view_control()
                            ctr.set_lookat(center.tolist())
                            ctr.set_front([0, -1, -0.2])
                            ctr.set_up([0, 0, 1])
                            ctr.set_zoom(0.25)
                        except Exception:
                            pass
                        if frame_count == 1:
                            print(f"  First frame displayed ({len(pts):,} points). Drag to rotate, scroll to zoom.")
                
                vis.poll_events()
                vis.update_renderer()
                
        except KeyboardInterrupt:
            print("\n\nStopped")
        finally:
            vis.destroy_window()
            sock.close()
        print(f"\nProcessed {frame_count} frames, {packet_count} packets")
        return
    
    # Matplotlib path (fallback; can be slow and freeze)
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
    
    try:
        while plt.fignum_exists(fig.number):
            # Drain socket until empty so we don't drop packets
            drain_cap = 2000
            while drain_cap > 0:
                drain_cap -= 1
                ready, _, _ = select.select([sock], [], [], 0.0)
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
            now = time.time()
            if packet_count == 0 and (now - last_packet_log) >= no_packet_warn_sec:
                print(f"  No UDP packets on port {lidar_port} yet. Run setup_ouster_network.ps1; allow firewall UDP {lidar_port}/{lidar_port + 1}; sensor IP: {sensor_ip}")
                last_packet_log = now
            plt.pause(0.001)
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
                ax.set_title(f'Frame {frame_count} | {len(current_points):,} pts | {dist_full.min():.1f}-{dist_full.max():.1f}m', color='white', fontsize=10)
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

def capture_pcap(hostname: str, lidar_port: int, imu_port: int = None, timeout: float = 15.0) -> Optional[str]:
    """Capture packets and save as PCAP file. Uses Python SDK (works on Windows); falls back to ouster-cli if available."""
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required for capture")
        return None
    
    if imu_port is None:
        imu_port = lidar_port + 1  # default IMU port next to lidar
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pcap_file = os.path.join(script_dir, "ouster.pcap")
    metadata_file = os.path.join(script_dir, "metadata.json")
    
    # 1) Try Python SDK capture (works on Windows and when ouster-cli is not on PATH)
    try:
        from contextlib import closing
        from ouster.sdk import sensor
        import ouster.sdk.pcap as pcap_module
        
        print("Capturing PCAP (Python SDK)...")
        n_seconds = max(1, min(int(timeout), 30))
        deadline = time.time() + n_seconds
        
        with closing(sensor.SensorPacketSource(hostname, lidar_port=lidar_port, imu_port=imu_port,
                                               buffer_time_sec=1.0)) as source:
            meta = source.sensor_info[0]
            with open(metadata_file, "w") as f:
                f.write(meta.to_json_string())
            print(f"  Metadata: {metadata_file}")
            
            def packet_iter():
                for _idx, packet in source:
                    if time.time() >= deadline:
                        break
                    yield packet
            
            n_packets = pcap_module.record(packet_iter(), pcap_file,
                                           lidar_port=lidar_port, imu_port=imu_port)
        
        if n_packets > 0 and os.path.getsize(pcap_file) > 0:
            print(f"  ✓ Captured {n_packets} packets: {pcap_file}")
            return pcap_file
        elif os.path.exists(pcap_file):
            os.remove(pcap_file)
    except Exception as e:
        print(f"  ⚠ Python SDK capture failed: {e}")
    
    # 2) Fallback: ouster-cli (Unix, when on PATH)
    if OUSTER_CLI_AVAILABLE:
        try:
            print("Capturing PCAP (ouster-cli)...")
            cmd = ["ouster-cli", "source", hostname, "slice", "0s:10s", "save", pcap_file]
            subprocess.run(cmd, timeout=timeout, capture_output=True)
            if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                print(f"  ✓ Captured: {pcap_file}")
                return pcap_file
        except Exception as e:
            print(f"  ⚠ ouster-cli failed: {e}")
    
    print("  ✗ Capture failed. Check sensor IP, network, and firewall (lidar/imu UDP ports).")
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
  python3 setup_ouster.py --check             Run connectivity checks (ping, HTTP, show PC IPs)
  python3 setup_ouster.py --discover         Find Ouster sensors on the network (Windows/macOS/Linux)
  python3 setup_ouster.py --live             Stream to browser (default)
  python3 setup_ouster.py --live --desktop  Stream to Open3D/matplotlib window
  python3 setup_ouster.py --live --sensor-ip 169.254.183.253
  python3 setup_ouster.py --load-packet ouster.pcap --metadata metadata.json
"""
    )
    
    parser.add_argument('--discover', action='store_true',
                        help='Discover Ouster sensors on the network (mDNS). Use the shown IP with --sensor-ip.')
    parser.add_argument('--check', action='store_true',
                        help='Run connectivity checks (ping, HTTP, firewall). Use with --sensor-ip if not default.')
    parser.add_argument('--load-packet', '--load_packet', nargs='?', const='ouster.pcap',
                        metavar='PCAP', dest='load_packet', help='Load and plot PCAP file')
    parser.add_argument('--metadata', metavar='FILE', help='Metadata JSON file')
    parser.add_argument('--max-range', '--max_range', type=float, help='Max range filter (m)')
    parser.add_argument('--min-range', '--min_range', type=float, help='Min range filter (m)')
    parser.add_argument('--grid-resolution', '--grid_resolution', type=float, dest='grid_resolution',
                        help='Grid snap resolution (m)')
    parser.add_argument('--live', action='store_true', help='Stream live (opens browser by default)')
    parser.add_argument('--desktop', action='store_true', help='Use Open3D/matplotlib window instead of browser for --live')
    parser.add_argument('--sensor-ip', '--sensor_ip', metavar='IP', dest='sensor_ip',
                        help=f'Sensor IP (default: {OUSTER_IP})')
    parser.add_argument('--port', type=int, help=f'UDP port (default: {LIDAR_PORT})')
    parser.add_argument('--accumulate', type=int, default=1, help='Frames to accumulate')
    parser.add_argument('--no-open3d', action='store_true', dest='no_open3d', help='Use matplotlib instead of Open3D (with --desktop)')
    parser.add_argument('--point-size', '--point_size', type=float, default=0.1, help='Point size in viewer (default 0.1)')
    parser.add_argument('--color', choices=['white', 'height', 'distance'], default='white', help='Color (--desktop only)')
    
    args = parser.parse_args()
    
    if args.check:
        run_connectivity_check(args.sensor_ip or OUSTER_IP, LIDAR_PORT, IMU_PORT)
        return
    
    if args.discover:
        print("Discovering Ouster sensors (mDNS)...")
        print("(Sensor and PC must be on the same subnet. Wait a few seconds.)\n")
        try:
            # ouster.cli.core has no __main__.py; invoke the CLI's run() with discover args
            discover_cmd = [
                sys.executable, "-c",
                "import sys; sys.argv = ['ouster-cli', 'discover', '--timeout', '8']; from ouster.cli.core import run; run()"
            ]
            subprocess.run(discover_cmd, timeout=20)
        except FileNotFoundError:
            print("✗ Could not run discover. Install ouster-sdk: pip install ouster-sdk")
        except subprocess.TimeoutExpired:
            print("✗ Discover timed out.")
        except ImportError as e:
            print(f"✗ Could not run discover: {e}")
        return
    
    if args.live:
        if args.desktop:
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
                color_mode=args.color,
            )
        else:
            stream_live_web(
                sensor_ip=args.sensor_ip,
                metadata_file=args.metadata,
                lidar_port=args.port,
                max_range=args.max_range,
                min_range=args.min_range,
                accumulate_frames=args.accumulate,
                grid_resolution=args.grid_resolution,
                point_size=args.point_size,
            )
        return
    
    if args.load_packet is not None:
        load_and_plot_pcap(args.load_packet, args.metadata, args.grid_resolution,
                          max_range=args.max_range, min_range=args.min_range)
        return
    
    # Default: capture
    pcap_file = capture_pcap(OUSTER_IP, LIDAR_PORT, imu_port=IMU_PORT)
    if pcap_file:
        print(f"\n✓ PCAP: {pcap_file}")
        print(f"To visualize: python3 setup_ouster.py --load-packet {os.path.basename(pcap_file)}")


if __name__ == "__main__":
    main()
