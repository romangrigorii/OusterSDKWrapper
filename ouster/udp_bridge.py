#!/usr/bin/env python3
"""
UDP Bridge for Ouster LiDAR packets.

Receives UDP packets from the sensor and forwards them via Unix domain 
STREAM socket to the C++ app. Uses simple 4-byte length prefix framing.

Usage:
    python3 udp_bridge.py [--src-ip 0.0.0.0] [--src-port 7510]
"""

import socket
import select
import argparse
import signal
import sys
import os
import struct
import threading

SOCKET_PATH = '/tmp/ouster_bridge.sock'

running = True
client_sock = None
client_lock = threading.Lock()

def signal_handler(sig, frame):
    global running
    print("\nShutting down...")
    running = False

def accept_clients(server_sock):
    """Accept client connections in background thread"""
    global client_sock, running
    
    server_sock.settimeout(1.0)
    while running:
        try:
            conn, _ = server_sock.accept()
            with client_lock:
                if client_sock:
                    try:
                        client_sock.close()
                    except:
                        pass
                client_sock = conn
                # Keep blocking mode for reliable sends
            print("C++ client connected!")
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"Accept error: {e}")

def main():
    global client_sock, running
    
    parser = argparse.ArgumentParser(description='UDP Bridge for Ouster LiDAR')
    parser.add_argument('--src-ip', default='0.0.0.0', help='Source IP to listen on')
    parser.add_argument('--src-port', type=int, default=7510, help='Source port to listen on')
    args = parser.parse_args()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Remove old socket file
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    
    # Create Unix STREAM socket server
    server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(SOCKET_PATH)
    server_sock.listen(1)
    os.chmod(SOCKET_PATH, 0o777)
    
    # Start accept thread
    accept_thread = threading.Thread(target=accept_clients, args=(server_sock,), daemon=True)
    accept_thread.start()
    
    # Create UDP receiver socket
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    recv_sock.bind((args.src_ip, args.src_port))
    recv_sock.setblocking(False)
    
    print(f"UDP Bridge started:")
    print(f"  Listening on UDP {args.src_ip}:{args.src_port}")
    print(f"  Forwarding to Unix socket: {SOCKET_PATH}")
    print(f"  Waiting for C++ client...")
    print()
    
    packet_count = 0
    bytes_forwarded = 0
    
    while running:
        try:
            ready = select.select([recv_sock], [], [], 0.5)
            if ready[0]:
                data, addr = recv_sock.recvfrom(65536)
                
                with client_lock:
                    if client_sock:
                        try:
                            # Simple framing: 4-byte length prefix (little-endian) + data
                            header = struct.pack('<I', len(data))
                            client_sock.sendall(header + data)
                            packet_count += 1
                            bytes_forwarded += len(data)
                            
                            if packet_count <= 5:
                                print(f"Forwarded packet {packet_count}: {len(data)} bytes from {addr}")
                            elif packet_count == 6:
                                print("... (suppressing further packet logs)")
                            elif packet_count % 10000 == 0:
                                print(f"Stats: {packet_count} packets, {bytes_forwarded / 1024 / 1024:.1f} MB")
                        except BrokenPipeError:
                            print("Client disconnected")
                            client_sock = None
                        except Exception as e:
                            if packet_count <= 10:
                                print(f"Send error: {e}")
                            client_sock = None
        except Exception as e:
            if running:
                print(f"Error: {e}")
    
    recv_sock.close()
    server_sock.close()
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    print(f"\nFinal stats: {packet_count} packets, {bytes_forwarded / 1024 / 1024:.1f} MB")

if __name__ == '__main__':
    main()
