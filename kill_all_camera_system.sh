#!/bin/bash
# Aggressive script to kill ALL camera_system_editor processes
# Use this if the regular kill_port.sh doesn't work

echo "Finding all camera_system_editor processes..."

# Find all processes matching camera_system_editor
pids=$(pgrep -f camera_system_editor 2>/dev/null || true)

if [ -z "$pids" ]; then
    echo "No camera_system_editor processes found"
    exit 0
fi

echo "Found processes: $pids"
echo ""

for pid in $pids; do
    echo "Killing PID $pid and all its children..."
    
    # Get process info
    ps -p $pid -o pid=,pgid=,comm=,args= 2>/dev/null || continue
    
    # Get process group
    pgid=$(ps -o pgid= -p $pid 2>/dev/null | tr -d ' ' || echo "")
    
    # Kill entire process group
    if [ -n "$pgid" ] && [ "$pgid" != "1" ]; then
        echo "  Killing process group $pgid..."
        kill -9 -$pgid 2>/dev/null || true
    fi
    
    # Find and kill all children
    children=$(pgrep -P $pid 2>/dev/null || true)
    if [ -n "$children" ]; then
        echo "  Killing children: $children"
        for child in $children; do
            kill -9 $child 2>/dev/null || true
        done
    fi
    
    # Kill the main process
    echo "  Killing main process $pid..."
    kill -9 $pid 2>/dev/null || true
    
    # Wait a moment
    sleep 0.5
    
    # Verify it's dead
    if kill -0 $pid 2>/dev/null; then
        echo "  WARNING: PID $pid still running! Trying sudo..."
        sudo kill -9 -$pgid 2>/dev/null || true
        sudo kill -9 $pid 2>/dev/null || true
    else
        echo "  ✓ PID $pid killed"
    fi
done

echo ""
echo "Waiting for processes to fully terminate..."
sleep 2

# Final check
remaining=$(pgrep -f camera_system_editor 2>/dev/null || true)
if [ -z "$remaining" ]; then
    echo "✓ All camera_system_editor processes terminated"
else
    echo "✗ WARNING: Some processes may still be running: $remaining"
    echo "  Try running: sudo pkill -9 -f camera_system_editor"
fi

# Also check ports
echo ""
echo "Checking ports 7503 and 7504..."
port_pids=$(lsof -ti :7503 :7504 2>/dev/null | sort -u || true)
if [ -z "$port_pids" ]; then
    echo "✓ Ports 7503 and 7504 are free"
else
    echo "✗ Ports still in use by PIDs: $port_pids"
    for ppid in $port_pids; do
        if kill -0 $ppid 2>/dev/null; then
            echo "  Killing PID $ppid..."
            sudo kill -9 $ppid 2>/dev/null || true
        fi
    done
fi
