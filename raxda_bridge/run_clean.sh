#!/bin/bash
echo "========================================"
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."
sudo fuser -k /dev/ttyS2
echo "✅ Port Cleared."
echo "========================================"
sleep 1
echo "🚀 STARTING RADXA BRIDGE..."
# Use Absolute Path to ensure finding the file
sudo python3 /home/shreyash/drone_project/raxda_bridge/real_bridge_service.py
