
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

# Try multiple SDK paths (check built versions first, then source)
_sdk_paths = [
    # Built/installed versions (preferred)
    os.path.join(_script_dir, "ouster-sdk", "build", "py3.14.2", "ouster-sdk"),
    os.path.join(_script_dir, "ouster-sdk", "build", "py3.13.2", "ouster-sdk"),
    os.path.join(_script_dir, "ouster-sdk", "build", "py3.12.2", "ouster-sdk"),
    os.path.join(_script_dir, "ouster-sdk", "build", "py3.11.2", "ouster-sdk"),
    # Source versions
    os.path.join(_script_dir, "ouster-sdk", "python", "src"),
    os.path.join(_script_dir, "ouster-sdk", "python"),
    os.path.join(_script_dir, "ouster-sdk"),
    # Absolute paths (fallback)
    "/Users/rgrigorii/Desktop/projects/thegame/camera_system/ouster/ouster-sdk/build/py3.14.2/ouster-sdk",
    "/Users/rgrigorii/Desktop/projects/thegame/camera_system/ouster/ouster-sdk/python/src",
    "/Users/rgrigorii/Desktop/projects/thegame/camera_system/ouster/ouster-sdk/python",
    "/Users/rgrigorii/Desktop/projects/thegame/camera_system/ouster/ouster-sdk",
]

_sdk_python_path = None
for path in _sdk_paths:
    if os.path.exists(path):
        # Check if it's a valid Python package path
        # For built versions, the structure is: build/py3.x.x/ouster-sdk/ouster/sdk/...
        # For source versions, the structure is: python/src/ouster/sdk/...
        ouster_sdk_path = os.path.join(path, "ouster", "sdk")
        ouster_path = os.path.join(path, "ouster")
        if os.path.exists(ouster_sdk_path) or os.path.exists(ouster_path):
            if path not in sys.path:
                sys.path.insert(0, path)
            _sdk_python_path = path
            print(f"✓ Found Ouster SDK at: {path}")
            break

HAS_OUSTER_SDK = False
SDK_IMPORT_ERROR = None
if _sdk_python_path:
    try:
        # Check for required dependencies first - must have numpy before importing SDK
        try:
            import numpy
            print(f"✓ numpy is available")
        except ImportError:
            print(f"⚠ numpy is required for Ouster SDK but not installed")
            print(f"  Install with: pip3 install numpy")
            HAS_OUSTER_SDK = False
            SDK_IMPORT_ERROR = "numpy not installed"
        else:
            # Only try to import SDK if numpy is available
            try:
                from ouster.sdk import pcap, core, sensor
                from ouster.sdk.core import LidarScan, ChanField, SensorInfo, Scans
                HAS_OUSTER_SDK = True
                print(f"✓ Successfully imported Ouster SDK from: {_sdk_python_path}")
            except ImportError as e:
                HAS_OUSTER_SDK = False
                SDK_IMPORT_ERROR = str(e)
                error_msg = str(e)
                print(f"⚠ Failed to import Ouster SDK: {error_msg}")
                print(f"  SDK path: {_sdk_python_path}")
                
                # Provide helpful error messages based on error type
                if "_bindings" in error_msg.lower() or "initialization failed" in error_msg.lower():
                    print(f"  C++ bindings not built or incompatible")
                    print(f"  The SDK Python package requires compiled C++ extensions")
                    print(f"  Try building the SDK: cd ouster-sdk && mkdir -p build && cd build && cmake .. && make")
                elif "No module named" in error_msg:
                    print(f"  Missing Python module - check SDK installation")
                
                # Only print full traceback if it's not a known dependency issue
                if "_bindings" not in error_msg.lower() and "initialization failed" not in error_msg.lower():
                    import traceback
                    traceback.print_exc()
    except Exception as e:
        HAS_OUSTER_SDK = False
        SDK_IMPORT_ERROR = str(e)
        print(f"⚠ Unexpected error checking Ouster SDK: {e}")
else:
    print(f"⚠ Ouster SDK not found in any of these locations:")
    for path in _sdk_paths[:6]:  # Show first 6 paths
        exists = "✓" if os.path.exists(path) else "✗"
        print(f"  {exists} {path}")

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

HAS_MATPLOTLIB = False
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

OUSTER_IP = "172.17.1.56"
MAC_IP = "172.17.1.100"
LIDAR_PORT = 7504


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
            capture_time = 10.0  # Increased timeout for recording
            # Try different command syntaxes - ouster-cli uses "save" command, not "record"
            cmd_variations = [
                ["ouster-cli", "source", hostname, "save", pcap_file],
                ["ouster-cli", "source", hostname, "save", "-o", pcap_file],
                ["ouster-cli", "source", hostname, "save", "--output", pcap_file],
                # Also try record command (older syntax or different versions)
                ["ouster-cli", "source", hostname, "record", "-o", pcap_file],
                ["ouster-cli", "source", hostname, "record", "--output", pcap_file],
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


def load_and_plot_pcap(pcap_file: str, metadata_file: Optional[str] = None, grid_resolution: Optional[float] = None):
    """Load a PCAP file and plot the point cloud"""
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
    
    if not HAS_OPEN3D and not HAS_MATPLOTLIB:
        print("✗ No visualization library available - cannot plot point cloud")
        print("  Install Open3D (recommended): pip install open3d")
        print("  Or install matplotlib: pip install matplotlib")
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
        
        valid = ranges_m.flatten() > 0
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
    """Render point cloud using Open3D (preferred) or matplotlib (fallback)"""
    import numpy as np
    
    if HAS_OPEN3D:
        render_point_cloud_open3d(x, y, z, intensity, num_points, title_suffix)
    elif HAS_MATPLOTLIB:
        render_point_cloud_matplotlib(x, y, z, intensity, num_points, title_suffix)
    else:
        print("✗ No visualization library available")
        sys.exit(1)


def render_point_cloud_open3d(x, y, z, intensity, num_points, title_suffix=""):
    """Render point cloud using Open3D - fast, interactive, best for large point clouds"""
    import numpy as np
    
    points = np.column_stack((x, y, z))
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    i_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-6)
    
    if HAS_MATPLOTLIB:
        colors = plt.cm.viridis(i_norm)[:, :3]
    else:
        colors = viridis_colormap(i_norm)
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    title = f'Ouster LiDAR Point Cloud ({num_points:,} points)'
    if title_suffix:
        title += f" {title_suffix}"
    
    print(f"  Opening interactive 3D viewer...")
    print(f"  Controls: Mouse drag to rotate, Shift+drag to pan, Scroll to zoom")
    print(f"  Press 'Q' or close window to exit")
    
    o3d.visualization.draw_geometries(
        [pcd],
        window_name=title,
        width=1920,
        height=1080,
        point_show_normal=False
    )


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
    
    i_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-6)
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(x, y, z, c=i_norm, cmap='viridis', s=0.5, alpha=0.6)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    title = f'Ouster LiDAR Point Cloud ({num_points:,} points)'
    if title_suffix:
        title += f" {title_suffix}"
    ax.set_title(title)
    
    plt.colorbar(scatter, ax=ax, label='Intensity (normalized)')
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Ouster LiDAR PCAP Capture and Visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Capture live lidar data to PCAP
  python3 setup_ouster.py
  
  # Load and plot a saved PCAP file (uses Open3D if available, matplotlib otherwise)
  python3 setup_ouster.py --load-packet ouster.pcap --metadata metadata.json
  
  # Load and plot with grid snapping (10cm resolution)
  python3 setup_ouster.py --load-packet ouster.pcap --grid-resolution 0.1

Note: For best visualization, install Open3D: pip install open3d
      Open3D provides interactive 3D viewing with rotation, zoom, and pan controls.
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
        '--grid-resolution',
        '--grid_resolution',
        type=float,
        metavar='RESOLUTION',
        dest='grid_resolution',
        help='Snap points to grid with given resolution in meters (e.g., 0.1 for 10cm grid)'
    )
    
    args = parser.parse_args()
    
    if args.load_packet is not None:
        pcap_file = args.load_packet if args.load_packet else 'ouster.pcap'
        load_and_plot_pcap(pcap_file, args.metadata, args.grid_resolution)
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
