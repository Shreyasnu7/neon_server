#!/bin/bash
# install_peripherals.sh
# Automates YDLidar SDK Installation and Python Libs
# FORCE ABSOLUTE PATHS to avoid /root confusion

USER_HOME="/home/shreyash"

echo "========================================="
echo "⚙️  DRONE HARDWARE SETUP WIZARD (PATH FIX)"
echo "========================================="

# 1. PERMISSIONS & DEPENDENCIES
echo "🔑 Requesting Sudo to install packages..."
sudo apt-get update
sudo apt-get install -y cmake pkg-config python3-dev build-essential git swig python3-pip libopencv-dev

# 1.1 PYTHON LIBRARIES
echo "🧠 Installing Python Brains..."
sudo pip3 install aiohttp websockets pymavlink opencv-python-headless --break-system-packages

# 2. NETWORK DIAGNOSTICS (Optional Check)
echo "========================================="
echo "🌐 NETWORK STATUS"
# ip link show
# ping -c 1 8.8.8.8

# 3. YDLIDAR INSTALLATION
echo "📡 Installing YDLidar SDK into $USER_HOME..."
cd $USER_HOME
if [ ! -d "YDLidar-SDK" ]; then
    # Run as USER shreyash to modify home dir, or just sudo it (files will be owned by root, but that's ok for drivers)
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
fi

cd $USER_HOME/YDLidar-SDK
mkdir -p build
cd build
cmake ..
make
sudo make install
echo "✅ SDK Installed."

# 4. PYTHON BINDINGS
echo "🐍 Installing Python Bindings..."
cd $USER_HOME/YDLidar-SDK/python
sudo python3 setup.py install
echo "✅ Python Driver Installed."

# 5. UDEV RULES
echo "🔌 Setting up USB Permissions..."
cd $USER_HOME/YDLidar-SDK
sudo cp startup/* /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✅ Udev Rules Applied."

echo "========================================="
echo "🎉 SETUP COMPLETE!"
echo "Please Reboot if USB doesn't work: 'sudo reboot'"
echo "========================================="
