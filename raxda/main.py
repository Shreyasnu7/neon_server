#!/usr/bin/env python3
"""
ULTRA DRONE - ONBOARD (RADXA ZERO)
----------------------------------
ARCHITECTURE:
1. FLIGHT: Mavlink to Mini Pix (/dev/ttyAML0)
2. SENSORS: UDP Listener for ESP32 (Port 8888)
3. CLOUD: WebSocket to Server (wss://...)
"""

import asyncio
import aiohttp
import json
import time
import socket
from pymavlink import mavutil

# --- CONFIGURATION ---
FC_PORT = "/dev/ttyAML0" # Pin 8/10
FC_BAUD = 57600
UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 8888    # ESP32 sends here
CLOUD_URL = "wss://web-production-fdccc.up.railway.app/ws/connect/drone_onboard"
DRONE_ID = "ultra_drone_01"

# --- STATE ---
mav_conn = None
udp_sock = None
sensor_data = {
    "dist_fl": -1, "dist_fr": -1, "dist_rl": -1, "dist_rr": -1,
    "ext_gyro": {"x":0, "y":0, "z":0}
}

# --- MAVLINK ---
def setup_mavlink():
    global mav_conn
    print(f"🔌 Connecting to Mini Pix at {FC_PORT}...")
    try:
        mav_conn = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
        print("✅ Mini Pix Port Opened")
    except Exception as e:
        print(f"⚠️ FC Connection Failed: {e}")

def send_velocity(vx, vy, vz, yaw_rate):
    """
    Send GUIDED Mode velocity target (NED Frame).
    """
    if not mav_conn: return
    # Type mask (ignore pos/accel, use vel/yawrate)
    type_mask = 0b0000111111000111 
    mav_conn.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0, # pos
        vx, vy, vz, # vel (m/s)
        0, 0, 0, # accel
        0, yaw_rate
    )

def set_mode(mode):
    if not mav_conn: return
    mode_id = mav_conn.mode_mapping().get(mode)
    if mode_id is None: return
    mav_conn.mav.set_mode_send(
        mav_conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# --- UDP SERVER (For ESP32) ---
def setup_udp():
    global udp_sock
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_IP, UDP_PORT))
    udp_sock.setblocking(False)
    print(f"✅ Listening for ESP32 on UDP :{UDP_PORT}")

def send_udp_command(cmd_dict, target_ip):
    """Send JSON command back to ESP32"""
    if not udp_sock or not target_ip: return
    try:
        payload = json.dumps(cmd_dict).encode('utf-8')
        udp_sock.sendto(payload, (target_ip, UDP_PORT))
    except Exception as e:
        print(f"UDP Send Error: {e}")

async def udp_loop():
    """Poll UDP socket for sensor packets"""
    global sensor_data
    while True:
        try:
            data, addr = udp_sock.recvfrom(1024)
            if data:
                try:
                    # ESP32 sends: {"tof":[120, 450, 900, 200], "gyro":[...]}
                    pkt = json.loads(data.decode('utf-8'))
                    
                    # Store latest state
                    if "tof" in pkt:
                        sensor_data["dist_fl"] = pkt["tof"][0]
                        sensor_data["dist_fr"] = pkt["tof"][2] # Order depends on array index
                        sensor_data["dist_rl"] = pkt["tof"][1]
                        sensor_data["dist_rr"] = pkt["tof"][3]
                    
                except json.JSONDecodeError:
                    pass
        except BlockingIOError:
            pass # No data
        except Exception as e:
            print(f"UDP Error: {e}")
            
        if mav_conn and sensor_data["dist_fl"] > 0:
            # Forward ToF Data to ArduPilot (Simple 4-sensor map)
            # MAVLink DISTANCE_SENSOR: https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
            
            # Helper to send single sensor
            def send_dist(dist_mm, orientation_id, sensor_id):
                if dist_mm <= 0: return
                mav_conn.mav.distance_sensor_send(
                    0, # time_boot_ms
                    10, # min_distance (cm)
                    200, # max_distance (cm) - VL53L0X is ~2m
                    dist_mm / 10, # current_distance (cm)
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    sensor_id,
                    orientation_id, # 0=Forward, 2=Right, 4=Back, 6=Left
                    0 # covariance
                )

            # Valid Map:
            # FL -> Forward-Left? Stick to Cardinal for simplicity or mix?
            # User has FL, FR, RL, RR.
            # Best mapping for ArduPilot Simple Avoidance (Sector based) or use OBSTACLE_DISTANCE
            
            # Let's map to Cardinals for basic "Stop" behavior
            # FL+FR -> Forward (0)
            # RR+FR -> Right (2)
            # RL+RR -> Back (4)
            # FL+RL -> Left (6)
            
            # Actually, let's just send 4 sectors roughly
            send_dist(sensor_data["dist_fl"], 7, 1) # 7=NorthWest (Forward-Left) (MAV_SENSOR_ROTATION_YAW_315)
            send_dist(sensor_data["dist_fr"], 1, 2) # 1=NorthEast (Forward-Right) (MAV_SENSOR_ROTATION_YAW_45)
            send_dist(sensor_data["dist_rl"], 5, 3) # 5=SouthWest (Back-Left) (MAV_SENSOR_ROTATION_YAW_225)
            send_dist(sensor_data["dist_rr"], 3, 4) # 3=SouthEast (Back-Right) (MAV_SENSOR_ROTATION_YAW_135)
            
        await asyncio.sleep(0.1) # 10Hz Mavlink Update

# --- CLOUD CLIENT ---
async def cloud_loop():
    esp32_ip = None # Learned from incoming packets
    
    while True:
        try:
            print(f"☁️ Connecting to {CLOUD_URL}")
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(CLOUD_URL) as ws:
                    print("✅ Cloud Connected!")
                    await ws.send_json({"type":"shakehand", "client_type":"drone", "id":DRONE_ID})
                    
                    async for msg in ws:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            data = json.loads(msg.data)
                            mtype = data.get("type")
                            
                            # 1. PILOT CONTROL
                            if mtype == "control_update":
                                ax = data.get("axes", {})
                                send_velocity(
                                    ax.get("pitch",0)*5, 
                                    ax.get("roll",0)*5, 
                                    ax.get("throttle",0)*-2, 
                                    ax.get("yaw",0)*1.5
                                )
                                
                            # 2. GIMBAL / LED (Forward to ESP32)
                            elif mtype == "aux_command":
                                # Cloud sends {"type":"aux", "led":"RED", "gimbal":[0, 45]}
                                # We forward to ESP32 via UDP
                                # We need the ESP32 IP (assume we got a packet recently)
                                # For now, assume broadcast or static IP if known, 
                                # but usually we reply to whoever sent us data.
                                pass 
                                
                        elif msg.type == aiohttp.WSMsgType.ERROR:
                            break
                            
                    # Periodic Telemetry Send
                    # await ws.send_json(...)
                    
        except Exception as e:
            print(f"Cloud Error: {e}")
            await asyncio.sleep(5)

async def main():
    print("🚀 Ultra Drone (Radxa) Starting...")
    setup_mavlink()
    setup_udp()
    
    await asyncio.gather(
        udp_loop(),
        cloud_loop()
    )

if __name__ == "__main__":
    asyncio.run(main())
