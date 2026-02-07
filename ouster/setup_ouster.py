
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
from typing import Optional

_script_dir = os.path.dirname(os.path.abspath(__file__))

# First, try to import the pip-installed ouster SDK (preferred - has C++ bindings)
_sdk_python_path = None
try:
    import ouster.sdk
    _sdk_python_path = os.path.dirname(os.path.dirname(ouster.sdk.__file__))
    print(f"✓ Found Ouster SDK (pip installed) at: {_sdk_python_path}")
except ImportError:
    # Fallback: Try local SDK paths (built versions first, then source)
    _sdk_paths = [
        # Built/installed versions (preferred)
        os.path.join(_script_dir, "ouster-sdk", "build", "py3.14.2", "ouster-sdk"),
        os.path.join(_script_dir, "ouster-sdk", "build", "py3.13.2", "ouster-sdk"),
        os.path.join(_script_dir, "ouster-sdk", "build", "py3.12.2", "ouster-sdk"),
        os.path.join(_script_dir, "ouster-sdk", "build", "py3.11.2", "ouster-sdk"),
        os.path.join(_script_dir, "ouster-sdk", "build", "py3.9.6", "ouster-sdk"),
        # Source versions (require C++ build)
        os.path.join(_script_dir, "ouster-sdk", "python", "src"),
        os.path.join(_script_dir, "ouster-sdk", "python"),
        os.path.join(_script_dir, "ouster-sdk"),
    ]

    for path in _sdk_paths:
        if os.path.exists(path):
            ouster_sdk_path = os.path.join(path, "ouster", "sdk")
            ouster_path = os.path.join(path, "ouster")
            if os.path.exists(ouster_sdk_path) or os.path.exists(ouster_path):
                if path not in sys.path:
                    sys.path.insert(0, path)
                _sdk_python_path = path
                print(f"✓ Found Ouster SDK (local) at: {path}")
                break
    
    if not _sdk_python_path:
        print(f"⚠ Ouster SDK not found. Install with: pip install ouster-sdk")

HAS_OUSTER_SDK = False
SDK_IMPORT_ERROR = None

# Check for required dependencies first - must have numpy before using SDK
try:
    import numpy
    print(f"✓ numpy is available")
except ImportError:
    print(f"⚠ numpy is required for Ouster SDK but not installed")
    print(f"  Install with: pip install numpy")
    SDK_IMPORT_ERROR = "numpy not installed"

if _sdk_python_path and SDK_IMPORT_ERROR is None:
    try:
        from ouster.sdk import pcap, core
        from ouster.sdk.core import LidarScan, ChanField, SensorInfo, Scans
        HAS_OUSTER_SDK = True
        print(f"✓ Successfully imported Ouster SDK")
    except ImportError as e:
        HAS_OUSTER_SDK = False
        SDK_IMPORT_ERROR = str(e)
        error_msg = str(e)
        print(f"⚠ Failed to import Ouster SDK: {error_msg}")
        
        if "_bindings" in error_msg.lower() or "initialization failed" in error_msg.lower():
            print(f"  C++ bindings not built or incompatible")
            print(f"  Install with: pip install ouster-sdk")
        elif "No module named" in error_msg:
            print(f"  Missing module - install with: pip install ouster-sdk")
    except Exception as e:
        HAS_OUSTER_SDK = False
        SDK_IMPORT_ERROR = str(e)
        print(f"⚠ Unexpected error importing Ouster SDK: {e}")

OUSTER_CLI_AVAILABLE = False
try:
    result = subprocess.run(["which", "ouster-cli"], capture_output=True, timeout=2)
    OUSTER_CLI_AVAILABLE = (result.returncode == 0)
except:
    pass

HAS_OPEN3D = False
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

HAS_PLOTLY = False
try:
    import plotly.graph_objects as go
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False

HAS_MATPLOTLIB = False
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

OUSTER_IP = "172.17.1.56"  # Sensor IP
MAC_IP = "172.17.1.100"
LIDAR_PORT = 7510
IMU_PORT = 7511


def capture_pcap(hostname: str, lidar_port: int, timeout: float = 15.0) -> Optional[str]:
    """Capture packets and save as PCAP file. Returns path to saved PCAP file."""
    if HAS_OUSTER_SDK:
        from ouster.sdk import pcap, core, sensor
        from ouster.sdk.core import LidarScan, ChanField, SensorInfo, Scans
    
    print("Capturing packets and saving as PCAP...")
    
    if OUSTER_CLI_AVAILABLE:
        print("  Using ouster-cli to record PCAP...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pcap_file = os.path.join(script_dir, "ouster.pcap")
        metadata_file = os.path.join(script_dir, "metadata.json")
        
        try:
            print(f"  Recording to {pcap_file}...")
            capture_time = 15.0  # Increased timeout for recording
            # ouster-cli syntax: source <host> slice <range> save <file>
            # slice is required - can be frame count (0:100) or time (0s:10s)
            cmd_variations = [
                # Time-based slice (10 seconds of data)
                ["ouster-cli", "source", hostname, "slice", "0s:10s", "save", pcap_file],
                # Frame-based slice (200 frames ~= 10 seconds at 20Hz)
                ["ouster-cli", "source", hostname, "slice", "0:200", "save", pcap_file],
                # Try without explicit output path (auto-generates filename)
                ["ouster-cli", "source", hostname, "slice", "0s:10s", "save", ".pcap"],
                # Older syntax variations (for compatibility)
                ["ouster-cli", "source", hostname, "save", pcap_file],
                ["ouster-cli", "source", hostname, "record", pcap_file],
            ]
            
            for i, cmd in enumerate(cmd_variations):
                if i > 0:
                    print(f"  Trying syntax variation {i+1}...")
                
                result = subprocess.run(
                    cmd,
                    timeout=capture_time,
                    capture_output=True,
                    text=True
                )
                
                # Check if PCAP file was created (ouster-cli may create it with a different name)
                # Check both the specified name and auto-generated names
                possible_files = [
                    pcap_file,
                    os.path.join(os.path.dirname(pcap_file), f"OS-*.pcap"),  # Auto-generated name pattern
                ]
                
                # Also check current directory for auto-generated files
                script_dir = os.path.dirname(os.path.abspath(__file__))
                for file_pattern in [pcap_file, os.path.join(script_dir, "*.pcap")]:
                    import glob
                    matching_files = glob.glob(file_pattern)
                    for f in matching_files:
                        if os.path.exists(f) and os.path.getsize(f) > 0:
                            # If it's not the expected name, rename it
                            if f != pcap_file:
                                if os.path.exists(pcap_file):
                                    os.remove(pcap_file)
                                os.rename(f, pcap_file)
                            print(f"  ✓ PCAP file created: {pcap_file}")
                            break
                    if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                        break
                
                if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                    print(f"  ✓ PCAP file created")
                    break
                
                # Print error for debugging (only on last attempt)
                if result.returncode != 0 and i == len(cmd_variations) - 1:
                    print(f"    All command variations failed")
                    print(f"    Last attempt: {' '.join(cmd)} (exit {result.returncode})")
                    if result.stderr:
                        print(f"    stderr: {result.stderr[:300]}")
                    if result.stdout:
                        print(f"    stdout: {result.stdout[:300]}")
                
                if "unexpected" in result.stderr.lower() or "unexpected" in result.stdout.lower():
                    continue
                
                # If command succeeded but no file created, ouster-cli might output to stdout
                if result.returncode == 0 and not os.path.exists(pcap_file):
                    # Try redirecting stdout to file (for commands that output PCAP to stdout)
                    try:
                        with open(pcap_file, 'wb') as f:
                            redirect_cmd = ["ouster-cli", "source", hostname, "save", "-"]  # Output to stdout
                            redirect_result = subprocess.run(
                                redirect_cmd,
                                timeout=capture_time,
                                stdout=f,
                                stderr=subprocess.PIPE,
                                text=False
                            )
                            if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                                print(f"  ✓ PCAP file created from stdout redirect")
                                break
                    except Exception as e:
                        pass  # Ignore redirect errors, continue to next variation
            
            if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                file_size = os.path.getsize(pcap_file)
                print(f"  ✓ PCAP recorded: {pcap_file} ({file_size} bytes)")
                
                print("  Fetching sensor metadata...")
                try:
                    metadata_cmd = ["ouster-cli", "source", hostname, "metadata"]
                    metadata_result = subprocess.run(
                        metadata_cmd,
                        timeout=10,
                        capture_output=True,
                        text=True
                    )
                    if metadata_result.returncode == 0 and metadata_result.stdout:
                        with open(metadata_file, 'w') as f:
                            f.write(metadata_result.stdout)
                        print(f"  ✓ Metadata saved: {metadata_file}")
                    else:
                        print(f"  ⚠ Could not fetch metadata via ouster-cli")
                except Exception as e:
                    print(f"  ⚠ Could not fetch metadata: {e}")
                
                return pcap_file
            else:
                print(f"  ⚠ PCAP file not created or empty")
                print("  Falling back to SDK capture...")
        except subprocess.TimeoutExpired:
            if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
                file_size = os.path.getsize(pcap_file)
                print(f"  ✓ PCAP recorded: {pcap_file} ({file_size} bytes)")
                try:
                    metadata_cmd = ["ouster-cli", "source", hostname, "metadata"]
                    metadata_result = subprocess.run(
                        metadata_cmd,
                        timeout=10,
                        capture_output=True,
                        text=True
                    )
                    if metadata_result.returncode == 0 and metadata_result.stdout:
                        with open(metadata_file, 'w') as f:
                            f.write(metadata_result.stdout)
                        print(f"  ✓ Metadata saved: {metadata_file}")
                except:
                    pass
                return pcap_file
            else:
                print("  ⚠ Timeout but no PCAP file created")
                print("  Falling back to SDK capture...")
        except Exception as e:
            print(f"  ⚠ Error using ouster-cli: {e}")
            print("  Falling back to SDK capture...")
    
    if HAS_OUSTER_SDK:
        print("  Using Ouster SDK to capture PCAP...")
        try:
            print(f"  Connecting to sensor {hostname}...")
            # Try to get sensor info - this may fail if sensor is unreachable
            try:
                info = SensorInfo.from_hostname(hostname, lidar_port=lidar_port)
                print(f"  ✓ Connected: {info.prod_line} (SN: {info.sn})")
            except Exception as e:
                print(f"  ⚠ Could not get sensor info: {e}")
                print(f"  Attempting to capture packets anyway...")
                info = None
            
            script_dir = os.path.dirname(os.path.abspath(__file__))
            metadata_file = os.path.join(script_dir, "metadata.json")
            if info:
                with open(metadata_file, 'w') as f:
                    f.write(info.json())
                print(f"  ✓ Saved metadata to {metadata_file}")
            
            print(f"  Starting packet capture on port {lidar_port}...")
            # Try to create packet source - may need to use different API
            packet_source = None
            try:
                # Try SensorPacketSource first (newer API)
                if hasattr(sensor, 'SensorPacketSource'):
                    packet_source = sensor.SensorPacketSource(hostname, lidar_port=lidar_port)
                else:
                    raise AttributeError("SensorPacketSource not available")
            except (AttributeError, TypeError) as e:
                # Fallback to older API or different method
                print(f"  ⚠ SensorPacketSource not available ({e}), trying alternative method...")
                try:
                    # Try using client.Sensor instead
                    from ouster.sdk import client
                    packet_source = client.Sensor(hostname, lidar_port, 7503)
                except Exception as e2:
                    print(f"  ✗ Could not create packet source: {e2}")
                    return None
            
            packets = []
            start_time = time.time()
            packet_count = 0
            max_packets = 2000
            
            try:
                # Handle different packet source types
                if hasattr(packet_source, '__iter__'):
                    # It's iterable directly
                    for packet in packet_source:
                        packets.append(packet)
                        packet_count += 1
                        
                        if time.time() - start_time > 10.0 or packet_count >= max_packets:
                            break
                        
                        if packet_count % 100 == 0:
                            print(f"    Captured {packet_count} packets...")
                elif hasattr(packet_source, 'read'):
                    # It has a read method
                    while time.time() - start_time < 10.0 and packet_count < max_packets:
                        try:
                            packet = packet_source.read()
                            if packet:
                                packets.append(packet)
                                packet_count += 1
                                if packet_count % 100 == 0:
                                    print(f"    Captured {packet_count} packets...")
                            else:
                                time.sleep(0.01)
                        except Exception:
                            break
            except Exception as e:
                print(f"  ⚠ Packet capture ended: {e}")
                import traceback
                traceback.print_exc()
            finally:
                if hasattr(packet_source, 'close'):
                    packet_source.close()
            
            if len(packets) == 0:
                print("  ✗ No packets captured")
                return None
            
            pcap_file = os.path.join(script_dir, "ouster.pcap")
            print(f"  Saving {len(packets)} packets to {pcap_file}...")
            
            pcap.record(
                packets,
                pcap_file,
                src_ip=hostname,
                dst_ip=MAC_IP,
                lidar_port=lidar_port
            )
            
            print(f"  ✓ Saved PCAP file: {pcap_file}")
            return pcap_file
            
        except Exception as e:
            print(f"  ✗ Error capturing PCAP: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    print("  ✗ SDK and ouster-cli not available")
    if SDK_IMPORT_ERROR:
        print(f"  SDK import error: {SDK_IMPORT_ERROR}")
    if not _sdk_python_path:
        print(f"  Could not find ouster-sdk directory")
        print(f"  Expected locations:")
        for path in _sdk_paths[:3]:  # Show first 3
            print(f"    - {path}")
    return None


def load_and_plot_pcap(pcap_file: str, metadata_file: Optional[str] = None, grid_resolution: Optional[float] = None,
                       max_range: Optional[float] = None, min_range: Optional[float] = None):
    """Load a PCAP file and plot the point cloud
    
    Args:
        pcap_file: Path to the PCAP file
        metadata_file: Path to metadata JSON file
        grid_resolution: Optional grid snapping resolution in meters
        max_range: Only show points closer than this distance (meters)
        min_range: Only show points farther than this distance (meters)
    """
    global HAS_OUSTER_SDK, SDK_IMPORT_ERROR
    
    if HAS_OUSTER_SDK:
        from ouster.sdk import pcap, core, sensor
        from ouster.sdk.core import LidarScan, ChanField, SensorInfo, Scans
    
    print("=" * 50)
    print("   Ouster LiDAR PCAP Visualization")
    print("=" * 50)
    print()
    
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK Python package not available - cannot parse PCAP files")
        print(f"  Import error: {SDK_IMPORT_ERROR}")
        print()
        print("  Build the SDK first:")
        print("    cd ouster-sdk/python")
        print("    pip install -e .")
        sys.exit(1)
    
    if not HAS_OPEN3D and not HAS_PLOTLY and not HAS_MATPLOTLIB:
        print("✗ No visualization library available - cannot plot point cloud")
        print("  Install Plotly (recommended, browser-based): pip install plotly")
        print("  Or install Open3D (desktop app): pip install open3d")
        print("  Or install matplotlib (basic): pip install matplotlib")
        sys.exit(1)
    
    if not os.path.isabs(pcap_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pcap_file = os.path.join(script_dir, pcap_file)
    
    if not os.path.exists(pcap_file):
        print(f"✗ PCAP file not found: {pcap_file}")
        sys.exit(1)
    
    if metadata_file is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        metadata_file = os.path.join(script_dir, "metadata.json")
    
    if not os.path.exists(metadata_file):
        print(f"✗ Metadata file not found: {metadata_file}")
        sys.exit(1)
    
    print(f"Loading PCAP file: {pcap_file}")
    print(f"Using metadata: {metadata_file}")
    
    try:
        with open(metadata_file, 'r') as f:
            meta = json.load(f)
        
        range_unit = meta.get("data_format", {}).get("range_unit")
        if range_unit is None:
            range_unit = meta.get("lidar_data_format", {}).get("range_unit")
        if range_unit is None:
            print("  ⚠ range_unit not found in metadata, using default 0.001 m/count")
            range_unit = 0.001
        else:
            print(f"  ✓ Found range_unit: {range_unit} m/count")
        
        # Check if PCAP file is valid before trying to open it
        pcap_size = os.path.getsize(pcap_file)
        if pcap_size < 100:
            print(f"  ✗ PCAP file is too small ({pcap_size} bytes) - likely invalid or empty")
            print(f"  This usually means the capture failed or the file is corrupted")
            sys.exit(1)
        
        # Check if it's a valid binary file (PCAP files should be binary)
        try:
            with open(pcap_file, 'rb') as f:
                header = f.read(4)
                # PCAP magic numbers: 0xa1b2c3d4 (little-endian) or 0xd4c3b2a1 (big-endian)
                magic = int.from_bytes(header, byteorder='little')
                magic_be = int.from_bytes(header, byteorder='big')
                if magic != 0xa1b2c3d4 and magic_be != 0xd4c3b2a1:
                    print(f"  ✗ Invalid PCAP file format (magic number: {hex(magic)} or {hex(magic_be)})")
                    print(f"  Expected PCAP magic number: 0xa1b2c3d4 or 0xd4c3b2a1")
                    print(f"  File might be corrupted or not a PCAP file")
                    sys.exit(1)
        except Exception as e:
            print(f"  ✗ Error reading PCAP file: {e}")
            sys.exit(1)
        
        print(f"  ✓ PCAP file appears valid ({pcap_size} bytes)")
        
        with open(metadata_file, 'r') as f:
            metadata_json = f.read()
        info = SensorInfo(metadata_json)
        print(f"  ✓ Loaded metadata: {info.prod_line} (SN: {info.sn})")
        
        try:
            # Note: complete parameter is no longer supported in newer SDK versions
            src = pcap.PcapScanSource(pcap_file, sensor_info=[info])
            print(f"  ✓ Opened PCAP file")
        except RuntimeError as e:
            if "unknown file format" in str(e).lower():
                print(f"  ✗ PCAP file format error: {e}")
                print(f"  This could mean:")
                print(f"    - The PCAP file is corrupted")
                print(f"    - The PCAP file is not in a supported format")
                print(f"    - The metadata file doesn't match the PCAP file")
                print(f"  Try using a different PCAP file or re-capturing")
            else:
                print(f"  ✗ Error opening PCAP file: {e}")
            sys.exit(1)
        except ValueError as e:
            print(f"  ✗ Error opening PCAP file: {e}")
            sys.exit(1)
        
        print("  Extracting point cloud from first scan...")
        print("  Note: This may take a moment if the PCAP file is large...")
        
        scans = iter(src)
        scan_data = None
        scan_count = 0
        max_attempts = 100  # Try up to 100 scans before giving up
        
        # Try to get at least one valid scan (complete or incomplete)
        for _ in range(max_attempts):
            try:
                scan_data = next(scans, None)
                if scan_data is not None and len(scan_data) > 0:
                    # Check if scan is complete (but accept incomplete scans too)
                    scan = scan_data[0]
                    if scan is not None:
                        is_complete = scan.complete() if hasattr(scan, 'complete') else True
                        if not is_complete:
                            print(f"  ⚠ Found incomplete scan (scan {scan_count + 1}), but processing anyway...")
                        break
                scan_count += 1
            except StopIteration:
                break
            except Exception as e:
                print(f"  ⚠ Error iterating scans: {e}")
                break
        
        if scan_data is None or len(scan_data) == 0:
            print(f"  ✗ No scans found in PCAP file after checking {scan_count} iterations")
            print(f"  This could mean:")
            print(f"    - The PCAP file doesn't contain complete lidar scans")
            print(f"    - The metadata file doesn't match the PCAP file format")
            print(f"    - The PCAP file contains only partial/incomplete scans")
            print(f"  ")
            print(f"  Attempting packet-level processing as fallback...")
            
            # Try packet-level processing as fallback
            try:
                from ouster.sdk.pcap import Pcap
                from ouster.sdk.client import ScanBatcher
                
                pcap_reader = Pcap(pcap_file, info)
                batcher = ScanBatcher(info)
                scan = None
                packet_count = 0
                
                print(f"  Reading packets from PCAP...")
                for packet in pcap_reader:
                    if hasattr(packet, 'packet_type') and 'lidar' in str(packet.packet_type).lower():
                        packet_count += 1
                        if scan is None:
                            scan = LidarScan(info.format.columns_per_frame, info.format.pixels_per_column, info.format.udp_profile_lidar)
                        
                        if batcher(packet, scan):
                            # Got a complete scan
                            print(f"  ✓ Found scan from packet-level processing (packet {packet_count})")
                            scan_data = [scan]
                            break
                        
                        if packet_count % 100 == 0:
                            print(f"  Processed {packet_count} packets...")
                        
                        if packet_count > 10000:  # Limit to avoid infinite loops
                            print(f"  ⚠ Processed {packet_count} packets but no complete scan found")
                            break
                
                if scan_data is None or len(scan_data) == 0:
                    print(f"  ✗ Packet-level processing also found no complete scans")
                    print(f"  Total packets processed: {packet_count}")
                    print(f"  ")
                    print(f"  Possible solutions:")
                    print(f"    1. Verify the metadata file matches the PCAP file")
                    print(f"    2. Re-capture the PCAP file with complete scans")
                    print(f"    3. Check if the PCAP file format is supported")
                    pcap_reader.close()
                    src.close()
                    sys.exit(1)
            except Exception as e:
                print(f"  ✗ Packet-level processing failed: {e}")
                import traceback
                traceback.print_exc()
                src.close()
                sys.exit(1)
        
        scan = scan_data[0]
        if scan is None:
            print("  ✗ Invalid scan")
            src.close()
            sys.exit(1)
        
        print(f"  ✓ Scan: {scan.w} columns x {scan.h} pixels")
        
        ranges_m = scan.field(ChanField.RANGE).astype(float) * range_unit
        
        SPEED_OF_LIGHT = 299792458.0
        tof_s = 2.0 * ranges_m / SPEED_OF_LIGHT
        print(f"  ✓ Calculated ToF: min={tof_s.min():.2e}s, max={tof_s.max():.2e}s")
        
        from ouster.sdk.core import XYZLut
        xyzlut = XYZLut(info)
        xyz = xyzlut(scan.field(ChanField.RANGE))
        
        intensity = scan.field(ChanField.SIGNAL)
        
        x = xyz[:, :, 0].flatten()
        y = xyz[:, :, 1].flatten()
        z = xyz[:, :, 2].flatten()
        i = intensity.flatten()
        
        ranges_flat = ranges_m.flatten()
        valid = ranges_flat > 0
        
        # Apply range filtering
        if min_range is not None:
            valid = valid & (ranges_flat >= min_range)
            print(f"  ✓ Filtering: min range = {min_range:.2f}m")
        if max_range is not None:
            valid = valid & (ranges_flat <= max_range)
            print(f"  ✓ Filtering: max range = {max_range:.2f}m")
        
        x = x[valid]
        y = y[valid]
        z = z[valid]
        i = i[valid]
        
        print(f"  ✓ Extracted {len(x)} valid points")
        
        src.close()
        
        if grid_resolution is not None and grid_resolution > 0:
            print(f"  Snapping points to grid (resolution: {grid_resolution} m)...")
            x, y, z, i = snap_to_grid(x, y, z, i, grid_resolution)
            print(f"  ✓ Grid-snapped to {len(x)} points")
            title_suffix = f"(grid: {grid_resolution}m)"
        else:
            title_suffix = ""
        
        print("  Rendering point cloud...")
        render_point_cloud(x, y, z, i, len(x), title_suffix)
        
        print("\nDone!")
        
    except Exception as e:
        print(f"✗ Error loading/plotting PCAP: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def snap_to_grid(x, y, z, intensity, grid_resolution):
    """
    Snap points to a grid by averaging points within each grid cell.
    
    Args:
        x, y, z: Point coordinates arrays
        intensity: Intensity values array
        grid_resolution: Grid cell size in meters
    
    Returns:
        Tuple of (x_grid, y_grid, z_grid, i_grid) with averaged grid points
    """
    import numpy as np
    
    if len(x) == 0:
        return x, y, z, intensity
    
    # Calculate grid cell indices for each point
    x_cell = np.floor(x / grid_resolution).astype(int)
    y_cell = np.floor(y / grid_resolution).astype(int)
    z_cell = np.floor(z / grid_resolution).astype(int)
    
    # Create a dictionary to group points by grid cell
    grid_dict = {}
    for i in range(len(x)):
        cell_key = (x_cell[i], y_cell[i], z_cell[i])
        if cell_key not in grid_dict:
            grid_dict[cell_key] = {'x': [], 'y': [], 'z': [], 'i': []}
        grid_dict[cell_key]['x'].append(x[i])
        grid_dict[cell_key]['y'].append(y[i])
        grid_dict[cell_key]['z'].append(z[i])
        grid_dict[cell_key]['i'].append(intensity[i])
    
    # Average points within each grid cell
    x_grid = []
    y_grid = []
    z_grid = []
    i_grid = []
    
    for cell_key, points in grid_dict.items():
        x_grid.append(np.mean(points['x']))
        y_grid.append(np.mean(points['y']))
        z_grid.append(np.mean(points['z']))
        i_grid.append(np.mean(points['i']))
    
    return np.array(x_grid), np.array(y_grid), np.array(z_grid), np.array(i_grid)


def render_point_cloud(x, y, z, intensity, num_points, title_suffix=""):
    """Render point cloud using Open3D (preferred), Plotly, or matplotlib (fallback)"""
    import numpy as np
    
    if HAS_OPEN3D:
        render_point_cloud_open3d(x, y, z, intensity, num_points, title_suffix)
    elif HAS_PLOTLY:
        render_point_cloud_plotly(x, y, z, intensity, num_points, title_suffix)
    elif HAS_MATPLOTLIB:
        render_point_cloud_matplotlib(x, y, z, intensity, num_points, title_suffix)
    else:
        print("✗ No visualization library available")
        print("  Install one of: pip install plotly, pip install open3d, pip install matplotlib")
        sys.exit(1)


def render_point_cloud_open3d(x, y, z, intensity, num_points, title_suffix=""):
    """Render point cloud using Open3D - fast, interactive, best for large point clouds"""
    import numpy as np
    
    points = np.column_stack((x, y, z))
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Use white color for all points
    colors = np.ones((len(points), 3))  # White (1, 1, 1)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Create coordinate frame at origin (1 meter axes)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    
    # Create a ground grid for reference
    grid_lines = []
    grid_size = 20  # meters
    grid_step = 2   # meters
    for i in range(-grid_size, grid_size + 1, grid_step):
        # Lines parallel to X axis
        grid_lines.append([[i, -grid_size, 0], [i, grid_size, 0]])
        # Lines parallel to Y axis
        grid_lines.append([[-grid_size, i, 0], [grid_size, i, 0]])
    
    grid_points = []
    grid_line_indices = []
    for i, line in enumerate(grid_lines):
        grid_points.extend(line)
        grid_line_indices.append([i*2, i*2+1])
    
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(grid_points)
    grid.lines = o3d.utility.Vector2iVector(grid_line_indices)
    grid.colors = o3d.utility.Vector3dVector([[0.3, 0.3, 0.3] for _ in grid_line_indices])
    
    title = f'Ouster LiDAR Point Cloud ({num_points:,} points)'
    if title_suffix:
        title += f" {title_suffix}"
    
    print()
    print("  " + "=" * 50)
    print("  Interactive 3D Point Cloud Viewer")
    print("  " + "=" * 50)
    print()
    print("  Mouse Controls:")
    print("    Left-drag:    Rotate view")
    print("    Shift+drag:   Pan view")
    print("    Ctrl+drag:    Roll view")
    print("    Scroll:       Zoom in/out")
    print()
    print("  Keyboard Shortcuts:")
    print("    Q:            Quit viewer")
    print("    H:            Print help")
    print("    R:            Reset view")
    print("    +:            Increase point size")
    print("    -:            Decrease point size")
    print("    1:            Toggle point visibility")
    print("    2:            Toggle normal visualization")
    print("    L:            Toggle lighting")
    print("    P:            Take screenshot")
    print()
    print(f"  Grid: {grid_size*2}m x {grid_size*2}m (step: {grid_step}m)")
    print(f"  Coordinate frame: X=Red, Y=Green, Z=Blue (1m)")
    print()
    
    # Use advanced visualizer for more control
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1920, height=1080)
    
    vis.add_geometry(pcd)
    vis.add_geometry(coord_frame)
    vis.add_geometry(grid)
    
    # Set rendering options
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.1, 0.1, 0.15])  # Dark background
    opt.show_coordinate_frame = False  # We have our own
    
    # Set initial view
    ctr = vis.get_view_control()
    ctr.set_zoom(0.5)
    ctr.set_front([0, -1, 0.3])
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, 0, 1])
    
    vis.run()
    vis.destroy_window()


def render_point_cloud_plotly(x, y, z, intensity, num_points, title_suffix=""):
    """Render point cloud using Plotly - interactive browser-based visualization"""
    import numpy as np
    import plotly.graph_objects as go
    
    # Subsample if too many points (Plotly can be slow with >500k points)
    max_points = 500000
    if len(x) > max_points:
        indices = np.random.choice(len(x), max_points, replace=False)
        x, y, z = x[indices], y[indices], z[indices]
        intensity = intensity[indices]
        print(f"  Note: Subsampled to {max_points:,} points for smooth visualization")
    
    title = f'Ouster LiDAR Point Cloud ({num_points:,} points)'
    if title_suffix:
        title += f" {title_suffix}"
    
    print()
    print("  " + "=" * 50)
    print("  Interactive 3D Point Cloud Viewer (Plotly)")
    print("  " + "=" * 50)
    print()
    print("  Opening in web browser...")
    print()
    print("  Controls:")
    print("    Left-drag:    Rotate view")
    print("    Right-drag:   Pan view")
    print("    Scroll:       Zoom in/out")
    print("    Double-click: Reset view")
    print()
    
    # Create the 3D scatter plot with white points
    fig = go.Figure(data=[go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(
            size=1.5,
            color='white',
            opacity=0.8,
        ),
        hovertemplate='X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Z: %{z:.2f}m<extra></extra>'
    )])
    
    # Add coordinate axes at origin
    axis_length = 2.0
    fig.add_trace(go.Scatter3d(x=[0, axis_length], y=[0, 0], z=[0, 0], 
                               mode='lines', line=dict(color='red', width=5), name='X', showlegend=False))
    fig.add_trace(go.Scatter3d(x=[0, 0], y=[0, axis_length], z=[0, 0], 
                               mode='lines', line=dict(color='green', width=5), name='Y', showlegend=False))
    fig.add_trace(go.Scatter3d(x=[0, 0], y=[0, 0], z=[0, axis_length], 
                               mode='lines', line=dict(color='blue', width=5), name='Z', showlegend=False))
    
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data',
            bgcolor='rgb(20, 20, 30)',
            xaxis=dict(gridcolor='rgb(50, 50, 60)', showbackground=True, backgroundcolor='rgb(30, 30, 40)'),
            yaxis=dict(gridcolor='rgb(50, 50, 60)', showbackground=True, backgroundcolor='rgb(30, 30, 40)'),
            zaxis=dict(gridcolor='rgb(50, 50, 60)', showbackground=True, backgroundcolor='rgb(30, 30, 40)'),
        ),
        paper_bgcolor='rgb(10, 10, 20)',
        font=dict(color='white'),
        margin=dict(l=0, r=0, b=0, t=40)
    )
    
    fig.show()


def viridis_colormap(values):
    """Viridis-like colormap: purple -> blue -> green -> yellow"""
    import numpy as np
    
    values = np.clip(values, 0, 1)
    
    r = np.interp(values, [0, 0.25, 0.5, 0.75, 1.0], [0.27, 0.16, 0.13, 0.33, 0.99])
    g = np.interp(values, [0, 0.25, 0.5, 0.75, 1.0], [0.00, 0.11, 0.43, 0.68, 0.99])
    b = np.interp(values, [0, 0.25, 0.5, 0.75, 1.0], [0.33, 0.54, 0.71, 0.60, 0.70])
    
    return np.column_stack([r, g, b])


def render_point_cloud_matplotlib(x, y, z, intensity, num_points, title_suffix=""):
    """Render point cloud using matplotlib - fallback option"""
    import numpy as np
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(x, y, z, c='white', s=0.5, alpha=0.6)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    title = f'Ouster LiDAR Point Cloud ({num_points:,} points)'
    if title_suffix:
        title += f" {title_suffix}"
    ax.set_title(title)
    
    # No colorbar needed for white points
    ax.set_facecolor('black')
    plt.tight_layout()
    plt.show()


def stream_live(sensor_ip: str = None, metadata_file: str = None, lidar_port: int = None,
                grid_resolution: float = None, max_range: float = None, min_range: float = None,
                accumulate_frames: int = 1, force_matplotlib: bool = False,
                point_size: float = 1.0, color_mode: str = 'white'):
    """
    Stream live LiDAR data using raw UDP sockets (bypasses macOS firewall) 
    and SDK for packet parsing. Uses Open3D for fast visualization if available.
    
    Args:
        sensor_ip: Sensor IP address (default: OUSTER_IP)
        metadata_file: Path to metadata JSON file
        lidar_port: UDP port for lidar data (default: LIDAR_PORT)
        point_size: Size of points in visualization (default: 1.0)
        color_mode: 'white', 'height', or 'distance' (default: 'white')
        grid_resolution: Snap points to grid with this resolution (meters)
        max_range: Only show points closer than this distance (meters)
        min_range: Only show points farther than this distance (meters)
        accumulate_frames: Number of frames to accumulate (1 = current frame only)
        force_matplotlib: If True, use matplotlib even if Open3D is available
    """
    import numpy as np
    import select
    
    global HAS_OUSTER_SDK
    
    if not HAS_OUSTER_SDK:
        print("✗ Ouster SDK required for live streaming")
        print("  Install with: pip install ouster-sdk")
        sys.exit(1)
    
    # Use matplotlib by default (Open3D has GLFW conflicts with Ouster SDK on macOS)
    use_open3d = False
    if not HAS_MATPLOTLIB:
        print("✗ matplotlib required for visualization")
        print("  Install with: pip install matplotlib")
        sys.exit(1)
    print("  Using matplotlib for visualization")
    
    from ouster.sdk.core import (LidarScan, ChanField, SensorInfo, XYZLut, 
                                  ScanBatcher, LidarPacket, PacketFormat)
    
    sensor_ip = sensor_ip or OUSTER_IP
    lidar_port = lidar_port or LIDAR_PORT
    
    print("=" * 50)
    print("   Ouster LiDAR Live Streaming")
    print("=" * 50)
    print()
    print(f"Sensor IP: {sensor_ip}")
    print(f"Lidar Port: {lidar_port}")
    print(f"Accumulate: {accumulate_frames} frame(s)")
    print()
    
    # Load metadata
    if metadata_file is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        metadata_file = os.path.join(script_dir, "example", "metadata.json")
    
    if not os.path.exists(metadata_file):
        print(f"✗ Metadata file not found: {metadata_file}")
        print("  Attempting to fetch from sensor...")
        try:
            import urllib.request
            url = f"http://{sensor_ip}/api/v1/sensor/metadata"
            with urllib.request.urlopen(url, timeout=5) as response:
                metadata_json = response.read().decode('utf-8')
            info = SensorInfo(metadata_json)
            print(f"  ✓ Fetched metadata from sensor: {info.prod_line} SN:{info.sn}")
        except Exception as e:
            print(f"  ✗ Failed to fetch metadata: {e}")
            sys.exit(1)
    else:
        print(f"Loading metadata: {metadata_file}")
        with open(metadata_file, 'r') as f:
            metadata_json = f.read()
        info = SensorInfo(metadata_json)
        print(f"  ✓ Loaded metadata: {info.prod_line} SN:{info.sn}")
    
    # Create SDK objects for packet parsing
    pf = PacketFormat.from_info(info)
    batcher = ScanBatcher(info)
    xyzlut = XYZLut(info)
    
    expected_packet_size = pf.lidar_packet_size
    print(f"  Expected packet size: {expected_packet_size} bytes")
    print(f"  Resolution: {info.format.columns_per_frame}x{info.format.pixels_per_column}")
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(('0.0.0.0', lidar_port))
    sock.setblocking(False)
    
    print(f"  ✓ Listening on UDP port {lidar_port}")
    print()
    print("Waiting for LiDAR data... (press Ctrl+C or close window to exit)")
    print()
    
    # Initialize visualization
    if use_open3d:
        vis = o3d.visualization.Visualizer()
        vis.create_window("Ouster Live - Press Q to quit", width=1400, height=900)
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        
        # Set rendering options
        opt = vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.point_size = 1.5
        
        # Set initial view
        ctr = vis.get_view_control()
        ctr.set_zoom(0.3)
    else:
        plt.ion()
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('black')
        fig.patch.set_facecolor('black')
        scatter = None
        
        # Add mouse wheel zoom
        zoom_state = {'scale': 1.0}
        def on_scroll(event):
            if event.inaxes != ax:
                return
            factor = 1.2 if event.button == 'down' else 1/1.2
            zoom_state['scale'] *= factor
            # Scale affects the axis limits - will be applied during updates
        fig.canvas.mpl_connect('scroll_event', on_scroll)
        
        print(f"  Controls: Drag to rotate, Scroll to zoom")
        print(f"  Color: {color_mode}, Point size: {point_size}")
    
    # State variables
    current_points = None  # Current frame's points (no accumulation by default)
    frame_count = 0
    packet_count = 0
    parse_errors = 0
    last_update_time = time.time()
    
    # Create LidarScan buffer
    scan = LidarScan(info.format.pixels_per_column, info.format.columns_per_frame, 
                     info.format.udp_profile_lidar)
    packet = LidarPacket(expected_packet_size)
    
    running = True
    try:
        while running:
            # Check if window is still open
            if use_open3d:
                if not vis.poll_events():
                    break
                vis.update_renderer()
            else:
                if not plt.fignum_exists(fig.number):
                    break
            
            # Drain ALL available packets from socket buffer (critical for keeping up with 10Hz sensor)
            packets_this_iteration = 0
            max_packets_per_iteration = 200  # Process up to 200 packets before updating display
            
            while packets_this_iteration < max_packets_per_iteration:
                ready, _, _ = select.select([sock], [], [], 0.001)
                if not ready:
                    break
                    
                try:
                    data, addr = sock.recvfrom(65535)
                    packet_count += 1
                    packets_this_iteration += 1
                    
                    if len(data) != expected_packet_size:
                        parse_errors += 1
                        continue
                    
                    packet.buf[:] = np.frombuffer(data, dtype=np.uint8)
                    
                    # Process packet - returns True when full scan is complete
                    if batcher(packet, scan):
                        frame_count += 1
                        
                        # Extract XYZ using SDK
                        xyz = xyzlut(scan.field(ChanField.RANGE))
                        ranges = scan.field(ChanField.RANGE).astype(np.float32) * 0.001  # mm to m
                        
                        # Debug: show raw range stats on first few frames
                        if frame_count <= 3:
                            total_pts = ranges.size
                            nonzero = np.count_nonzero(ranges)
                            if nonzero > 0:
                                r_valid = ranges[ranges > 0]
                                print(f"  DEBUG frame {frame_count}: {total_pts} total, {nonzero} nonzero ({100*nonzero/total_pts:.1f}%), range: [{r_valid.min():.2f}, {r_valid.max():.2f}] m")
                            else:
                                print(f"  DEBUG frame {frame_count}: {total_pts} total, 0 nonzero")
                        
                        # Flatten
                        x = xyz[:, :, 0].flatten()
                        y = xyz[:, :, 1].flatten()
                        z = xyz[:, :, 2].flatten()
                        r = ranges.flatten()
                        
                        # Filter valid points (range > 0)
                        valid = r > 0
                        if min_range is not None:
                            valid &= (r >= min_range)
                        if max_range is not None:
                            valid &= (r <= max_range)
                        
                        x, y, z = x[valid], y[valid], z[valid]
                        
                        if len(x) > 0:
                            new_points = np.column_stack((x, y, z))
                            
                            # Replace current points (no accumulation when accumulate_frames=1)
                            if accumulate_frames <= 1:
                                current_points = new_points
                            else:
                                # Accumulate frames
                                if current_points is None:
                                    current_points = new_points
                                else:
                                    current_points = np.vstack([current_points, new_points])
                                    # Limit total points
                                    max_pts = accumulate_frames * 100000
                                    if len(current_points) > max_pts:
                                        current_points = current_points[-max_pts:]
                            
                            # Log stats
                            if frame_count == 1 or frame_count % 10 == 0:
                                dist = np.sqrt(x**2 + y**2 + z**2)
                                print(f"  Frame {frame_count}: {len(x):,} pts, range: [{dist.min():.1f}, {dist.max():.1f}] m")
                        
                        # Reset scan buffer
                        scan = LidarScan(info.format.pixels_per_column, 
                                        info.format.columns_per_frame,
                                        info.format.udp_profile_lidar)
                        
                except socket.error:
                    break  # No more data available
                except Exception as e:
                    parse_errors += 1
                    if parse_errors <= 3:
                        print(f"  ⚠ Parse error: {e}")
            
            # Allow matplotlib to process events
            if not use_open3d:
                plt.pause(0.001)
            
            # Update visualization (every frame for Open3D, throttled for matplotlib)
            current_time = time.time()
            should_update = current_points is not None
            
            if not use_open3d:
                # Throttle matplotlib updates
                should_update = should_update and (current_time - last_update_time >= 0.1)
            
            if should_update:
                last_update_time = current_time
                
                if use_open3d:
                    # Open3D update - very fast
                    pcd.points = o3d.utility.Vector3dVector(current_points)
                    
                    # Color by height
                    colors = np.zeros((len(current_points), 3))
                    z_norm = (current_points[:, 2] - current_points[:, 2].min()) 
                    z_range = current_points[:, 2].max() - current_points[:, 2].min()
                    if z_range > 0:
                        z_norm = z_norm / z_range
                    colors[:, 0] = 1.0  # Red channel
                    colors[:, 1] = 1.0 - z_norm * 0.5  # Slight gradient
                    colors[:, 2] = 1.0 - z_norm * 0.5
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    
                    vis.update_geometry(pcd)
                else:
                    # Matplotlib update - slower
                    ax.clear()
                    
                    # Subsample for performance
                    pts = current_points
                    if len(pts) > 50000:
                        idx = np.random.choice(len(pts), 50000, replace=False)
                        pts = pts[idx]
                    
                    # Calculate distances for stats/coloring
                    dist = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2)
                    
                    # Determine colors based on color_mode
                    if color_mode == 'height':
                        z_vals = pts[:, 2]
                        z_norm = (z_vals - z_vals.min()) / (z_vals.max() - z_vals.min() + 1e-6)
                        colors = plt.cm.turbo(z_norm)
                    elif color_mode == 'distance':
                        d_norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-6)
                        colors = plt.cm.viridis(d_norm)
                    else:  # white
                        colors = 'white'
                    
                    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], 
                              c=colors, s=point_size, alpha=0.8)
                    
                    dist_full = np.sqrt(current_points[:, 0]**2 + current_points[:, 1]**2 + current_points[:, 2]**2)
                    ax.set_title(f'Frame {frame_count} | {len(current_points):,} pts | {dist_full.min():.1f}-{dist_full.max():.1f}m',
                                color='white', fontsize=10)
                    ax.set_facecolor('black')
                    ax.set_xlabel('X', color='white')
                    ax.set_ylabel('Y', color='white')
                    ax.set_zlabel('Z', color='white')
                    ax.tick_params(colors='white')
                    
                    # Auto-scale with zoom
                    lim = np.percentile(np.abs(pts), 95) * 1.2
                    lim = max(lim, 3.0)
                    lim = lim * zoom_state['scale']  # Apply zoom
                    ax.set_xlim(-lim, lim)
                    ax.set_ylim(-lim, lim)
                    ax.set_zlim(-lim/2, lim/2)
                    
                    plt.draw()
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        sock.close()
        if use_open3d:
            vis.destroy_window()
        else:
            plt.close()
    
    print(f"\nProcessed {frame_count} frames, {packet_count} packets")


def main():
    parser = argparse.ArgumentParser(
        description="Ouster LiDAR PCAP Capture and Visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Stream live LiDAR data (uses raw sockets to bypass macOS firewall)
  python3 setup_ouster.py --live
  python3 setup_ouster.py --live --sensor-ip 169.254.183.253 --metadata example/metadata.json
  python3 setup_ouster.py --live --max-range 10  # Only show points within 10m
  
  # Load and plot a saved PCAP file
  python3 setup_ouster.py --load-packet ouster.pcap --metadata metadata.json
  
  # Load and plot with grid snapping (10cm resolution)
  python3 setup_ouster.py --load-packet ouster.pcap --grid-resolution 0.1

Note: For best visualization, install Open3D: pip install open3d
      The --live mode uses raw Python sockets which bypass macOS firewall restrictions
      on unsigned binaries, while still using the Ouster SDK for packet parsing.
        """
    )
    
    parser.add_argument(
        '--load-packet',
        '--load_packet',
        nargs='?',
        const='ouster.pcap',
        metavar='PCAP_FILE',
        dest='load_packet',
        help='Load and plot a saved PCAP file (default: ouster.pcap)'
    )
    
    parser.add_argument(
        '--metadata',
        metavar='METADATA_FILE',
        help='Metadata JSON file (default: metadata.json)'
    )
    
    parser.add_argument(
        '--max-range',
        '--max_range',
        type=float,
        metavar='DISTANCE',
        help='Only show points closer than this distance (in meters)'
    )
    
    parser.add_argument(
        '--min-range',
        '--min_range',
        type=float,
        metavar='DISTANCE',
        help='Only show points farther than this distance (in meters)'
    )
    
    parser.add_argument(
        '--grid-resolution',
        '--grid_resolution',
        type=float,
        metavar='RESOLUTION',
        dest='grid_resolution',
        help='Snap points to grid with given resolution in meters (e.g., 0.1 for 10cm grid)'
    )
    
    parser.add_argument(
        '--live',
        action='store_true',
        help='Stream live LiDAR data from sensor (uses raw sockets to bypass macOS firewall)'
    )
    
    parser.add_argument(
        '--sensor-ip',
        '--sensor_ip',
        metavar='IP',
        dest='sensor_ip',
        help=f'Sensor IP address for live streaming (default: {OUSTER_IP})'
    )
    
    parser.add_argument(
        '--port',
        type=int,
        metavar='PORT',
        help=f'UDP port for lidar data (default: {LIDAR_PORT})'
    )
    
    parser.add_argument(
        '--accumulate',
        type=int,
        default=1,
        metavar='FRAMES',
        help='Number of frames to accumulate for live display (default: 1 = current frame only)'
    )
    
    parser.add_argument(
        '--no-open3d',
        action='store_true',
        dest='no_open3d',
        help='Force matplotlib visualization (avoid Open3D/GLFW conflicts)'
    )
    
    parser.add_argument(
        '--point-size',
        '--point_size',
        type=float,
        default=1.0,
        metavar='SIZE',
        help='Point size for visualization (default: 1.0)'
    )
    
    parser.add_argument(
        '--color',
        choices=['white', 'height', 'distance'],
        default='white',
        help='Point coloring mode: white, height (z-axis), or distance (default: white)'
    )
    
    args = parser.parse_args()
    
    # Live streaming mode
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
        pcap_file = args.load_packet if args.load_packet else 'ouster.pcap'
        load_and_plot_pcap(pcap_file, args.metadata, args.grid_resolution,
                          max_range=args.max_range, min_range=args.min_range)
        return
    
    print("=" * 50)
    print("   Ouster LiDAR PCAP Capture")
    print("=" * 50)
    print()
    
    packet_file = capture_pcap(OUSTER_IP, LIDAR_PORT)
    if not packet_file:
        print("\n✗ Failed to capture PCAP file")
        sys.exit(1)
    
    print("\n" + "=" * 50)
    print("   Capture Complete!")
    print("=" * 50)
    print()
    print(f"PCAP file: {packet_file}")
    print(f"Metadata file: metadata.json")
    print()
    print("To visualize, run:")
    print(f"  python3 setup_ouster.py --load-packet {os.path.basename(packet_file)}")
    print()


if __name__ == "__main__":
    main()
