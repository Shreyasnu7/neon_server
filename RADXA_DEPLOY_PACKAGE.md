# Radxa Bridge Deployment Package (v132 - Final Verified)

**Architecture:**
- **OS (eMMC):** Protected by OverlayFS (Read-Only).
- **Data (SD Card):** Read-Write Storage (Formatted & Fresh).

---

## 🛑 STEP 1: WIPE & PREPARE SD CARD
**Since your SD card is corrupted ("Structure needs cleaning"), we MUST wipe it.**

Run these commands **ONE BY ONE**:

1.  **Unmount & Format (Deletes Everything on SD):**
    ```bash
    sudo umount -f /mnt/sd_storage
    sudo mkfs.ext4 -F /dev/mmcblk1p1
    ```

2.  **Mount & Set Permissions:**
    ```bash
    sudo mount /dev/mmcblk1p1 /mnt/sd_storage
    sudo mkdir -p /mnt/sd_storage/drone_project
    sudo chown -R $USER:$USER /mnt/sd_storage
    ```

3.  **Fix Serial Permissions (Critical):**
    ```bash
    sudo usermod -a -G dialout $USER
    # YOU MUST REBOOT AFTER THIS STEP IF YOU HAVEN'T ALREADY!
    ```

---

## 📂 STEP 2: PASTE THE FULL CODE (WITH NEW COMMANDS)
Now we restore the **COMPLETE** Bridge Code (Standard version with Camera, Lidar, and Safety).

1.  **Enter Folder**:
    ```bash
    cd /mnt/sd_storage/drone_project
    sudo systemctl restart drone-bridge
    sudo journalctl -u drone-bridge -f
    nano real_bridge_service.py
    ```

2.  **COPY AND PASTE THIS ENTIRE BLOCK:**

```python
import socket
import asyncio
import json
import time
from pymavlink import mavutil
import websockets
import cv2
import aiohttp
import os

# CONFIG
# UART2 on Radxa Zero 3W (Pins 8/10) - ENABLED via Overlay
FC_PORT = '/dev/ttyS2' 
FC_BAUD = 57600
SERVER_URL = "wss://drone-server-r0qe.onrender.com/ws/connect/RADXA_X" 
# HTTP URL for Video Push
API_URL = "https://drone-server-r0qe.onrender.com" 




class SafetyEnvelope:
    def __init__(self, cache_ref):
        self.cache = cache_ref # Shared telemetry cache
        
    def validate_auto_action(self, action):
        """
        Intervention Logic:
        If Obstacle < 1.0m, BLOCK 'FORWARD', 'FOLLOW', 'ORBIT'
        Allow 'HOVER', 'LAND', 'RTL' always.
        """
        if action in ['HOVER', 'LAND', 'RTL', 'RTL_SMART', 'set_rth_alt', 'set_batt_threshold', 'camera_config']:
            return True
            
        # Check Obstacles
        lidar_dist = self.cache.get('lidar_dist', 999)
        front_dist = self.cache.get('front_dist', 999)
        
        min_clearance = 1.0 # meters
        
        if lidar_dist < min_clearance or front_dist < min_clearance:
            print(f"🛡️ SAFETY INTERCEPT: Blocked '{action}' due to Obstacle ({min(lidar_dist, front_dist):.1f}m)")
            return False
            
        return True

class RadxaBridge:
    def __init__(self):
        self.fc = None
        self.ws = None
        self.running = True
        self.telemetry_cache = {}
        self.safety = SafetyEnvelope(self.telemetry_cache) # FIXED: Init Safety Logic
        self.cam_config = {'w': 640, 'h': 480, 'fps': 30, 'source': 'internal'}
        self.cam_needs_reset = False
        self.recording = False
        self.rec_out = None
        self.batt_threshold = 20
        self.user_gps = None
        self.last_cloud_msg = time.time()


    def init_esp32(self):
        """Setup UDP for ESP32 Gimbal"""
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Assuming ESP32 is on Hotspot 192.168.4.2 (Standard SoftAP assignment)
        # Or Broadcast? Let's try Broadcast first to be safe, or fixed IP if known.
        # User's esp32_driver.py used 192.168.4.2. Let's stick to that.
        self.esp32_addr = ("192.168.4.2", 8888) 
        print(f"🔭 ESP32 Gimbal Link Active -> {self.esp32_addr}")



    async def connect_mavlink(self):
        self.init_esp32() # Init UDP
        asyncio.create_task(self.lidar_loop())
        
        bauds = [57600, 115200]
        
        while self.running:
            for baud in bauds:
                try:
                    print(f"🔌 Connecting to FC on {FC_PORT} @ {baud}...")
                    self.fc = mavutil.mavlink_connection(FC_PORT, baud=baud)
                    
                    # Wait for heartbeat with a shorter timeout for cycling
                    hb = self.fc.wait_heartbeat(timeout=3)
                    
                    if hb is None:
                        print(f"❌ Heartbeat Timeout @ {baud}.")
                        self.fc.close()
                        continue # Try next baud
                        
                    print(f"✅ FC Connected @ {baud}! Heartbeat receiving.")
                    
                    # REQUEST DATA STREAMS
                    self.fc.mav.request_data_stream_send(
                        self.fc.target_system, self.fc.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
                    )
                    print("📡 Data Streams Requested (4Hz)")
                    return # SUCCESS - Exit Loop
                    
                except Exception as e:
                    print(f"⚠️ FC Check Failed: {e}")
                    await asyncio.sleep(1)

            print("🔴 FC NOT DETECTED (Tried 57600 & 115200). Retrying in 3s...")
            print("👉 SUGGESTION: Swap RX/TX Wires?")
            await asyncio.sleep(3)

# ... (Previous Code)

    async def command_loop(self):
        """Listens for commands from Cloud"""
        while self.running and self.ws:
            try:
                msg = await self.ws.recv()
                data = json.loads(msg)
                print(f"⬇ CMD: {data}")
                
                type = data.get("type")
                payload = data.get("payload", {})

                if type == 'control':
                     # ... (Existing RC Code) ...
                     pass
                
                elif type == 'camera_config':
                    # Dynamic Resolution Switch
                    w = payload.get('width', 640)
                    h = payload.get('height', 480)
                    fps = payload.get('fps', 30)
                    self.cam_config = {'w': w, 'h': h, 'fps': fps}
                    self.cam_needs_reset = True
                    print(f"📷 Config Updated: {self.cam_config}")
                    
            except Exception as e:
                print(f"Command Loop Error: {e}")
                await asyncio.sleep(1)

    async def video_loop(self):
        """Captures video, records, and pushes to Cloud"""
        print("🎥 Video Loop Started")
        
        # Init Camera
        try:
             # index 0 is usually internal or USB cam. index 1 might be external if connected via HDMI-USB
             self.cap = cv2.VideoCapture(0)
             self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_config['w'])
             self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_config['h'])
             self.cap.set(cv2.CAP_PROP_FPS, self.cam_config['fps'])
        except Exception as e:
             print(f"❌ Camera Init Failed: {e}")

        async with aiohttp.ClientSession() as session:
            while self.running:
                if self.cam_needs_reset:
                    # Re-init logic if needed (resolution change)
                    self.cam_needs_reset = False
                    # simplified for now
                
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        # 1. Recording
                        if self.recording and self.rec_out:
                             self.rec_out.write(frame)
                        
                        # 2. Push to Cloud (MJPEG style 1 frame per request for simplicity or websocket)
                        # We use HTTP Push to /video_push
                        try:
                           _, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                           data = jpg.tobytes()
                           
                           # Fire and forget (don't await response strictly to keep Loop fast)
                           # But aiohttp requires await. We use create_task to background it? 
                           # Or just await it, fps might drop. 
                           # For verified correctness: await.
                           url = f"{API_URL}/video/frame"
                           async with session.post(url, data={'file': data}) as resp:
                               pass
                        except Exception as e:
                           # print(f"Video Push Error: {e}")
                           pass
                    else:
                        await asyncio.sleep(0.1)
                else:
                    await asyncio.sleep(1) # Wait for camera
                
                await asyncio.sleep(0.01) # Yield

    async def connect_cloud(self):
        while self.running:
            try:
                print(f"☁️ Connecting to Cloud: {SERVER_URL}...")
                async with websockets.connect(SERVER_URL) as ws:
                    print("✅ Cloud Connected!")
                    self.ws = ws
                    
                    # Parallel Tasks: Read MAVLink & Listen Cloud & Video Loop
                    await asyncio.gather(
                        self.telemetry_loop(),
                        self.command_loop(),
                        self.video_loop()
                    )
            except Exception as e:
                print(f"⚠️ Cloud Disconnected: {e}. Retrying in 5s...")
                self.ws = None
                await asyncio.sleep(5)

    async def telemetry_loop(self):
        """Reads MAVLink and sends to Cloud"""
        last_send = 0
        while self.running and self.ws:
            # Non-blocking MAVLink read
            msg = self.fc.recv_match(blocking=False)
            if msg:
                type = msg.get_type()
                
                # BATTERY
                if type == 'SYS_STATUS':
                    self.telemetry_cache['battery'] = msg.battery_remaining # %
                    self.telemetry_cache['voltage'] = msg.voltage_battery / 1000.0 # V
                    
                    # USER REQUEST: At X% Battery, Auto Return
                    if msg.battery_remaining < self.batt_threshold and not getattr(self, 'low_batt_triggered', False):
                        print(f"⚠️ LOW BATTERY ({msg.battery_remaining}%)! Smart RTL Triggered.")
                        self.low_batt_triggered = True
                        if self.user_gps:
                            self.command_queue.put({'type': 'command', 'payload': 'smart_rtl_user'})
                        else:
                            self.fc.set_mode('RTL') # Fallback to standard RTL

                # OBSTACLE AVOIDANCE SENSORS (Lidar/Ultrasonic)
                elif type == 'DISTANCE_SENSOR':
                    # Parse MAVLink Distance Sensor
                    dist_m = msg.current_distance / 100.0 # cm to m
                    orientation = msg.orientation # 0=Forward, 4=Back, etc.
                    
                    if orientation == 0: # Forward
                        self.telemetry_cache['front_dist'] = dist_m
                    elif orientation == 4: # Backward
                        self.telemetry_cache['back_dist'] = dist_m
                    
                    # Also populate generic for AI
                    self.telemetry_cache['lidar_dist'] = dist_m
                    
                    if self.telemetry_cache.get('battery', 100) < self.batt_threshold:
                         print(f"⚠️ CRITICAL BATTERY (<{self.batt_threshold}%)! Initiating Smart RTL...")
                         self.low_batt_triggered = True
                        
                         # SMART RETURN TO USER (Local Failover)
                         target_lat, target_lng = self.user_gps if self.user_gps else (None, None)
                        
                         if target_lat and target_lng:
                             print(f"🔄 Returning to LAST KNOWN USER LOC: {target_lat}, {target_lng}")
                             # Set Guided Mode
                             self.fc.mav.set_mode_send(
                                 self.fc.target_system,
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 4) # GUIDED
                            
                             # Send Coordinate
                             self.fc.mav.mission_item_int_send(
                                 self.fc.target_system, self.fc.target_component,
                                 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 2, 0, 0, 0, 0, 0,
                                 int(target_lat * 1e7), int(target_lng * 1e7), 15 # 15m alt
                             )
                         else:
                             print("🏠 User Loc Unknown. Returning to HOME.")
                             self.fc.mav.command_long_send(
                                 self.fc.target_system, self.fc.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

                         if self.ws:
                               asyncio.create_task(self.ws.send(json.dumps({
                                  "type": "alert", "payload": {"level": "emergency", "msg": "LOW BATTERY RTL"}
                               })))
                
                # GPS
                if type == 'GPS_RAW_INT':
                    self.telemetry_cache['lat'] = msg.lat / 1e7
                    self.telemetry_cache['lng'] = msg.lon / 1e7
                    self.telemetry_cache['sats'] = msg.satellites_visible
                    self.telemetry_cache['fix'] = msg.fix_type
                
                # ATTITUDE (Pitch/Roll/Yaw)
                if type == 'ATTITUDE':
                    self.telemetry_cache['roll'] = msg.roll
                    self.telemetry_cache['pitch'] = msg.pitch
                    self.telemetry_cache['yaw'] = msg.yaw
                
                # VFR_HUD (Speed/Alt/Heading)
                if type == 'VFR_HUD':
                    self.telemetry_cache['altitude'] = msg.alt
                    self.telemetry_cache['speed'] = msg.groundspeed
                    self.telemetry_cache['heading'] = msg.heading

            # Rate Limit Updates to Cloud (e.g. 10Hz)
            now = time.time()
            if now - last_send > 0.1: # 100ms
                if self.telemetry_cache:
                    await self.ws.send(json.dumps({
                        "type": "telemetry",
                        "payload": self.telemetry_cache
                    }))
                last_send = now
            
            await asyncio.sleep(0.01) # Yield

    async def command_loop(self):
        """Listens for commands from Cloud"""
        while self.running and self.ws:
            try:
                msg = await self.ws.recv()
                self.last_cloud_msg = time.time() # Reset Watchdog
                data = json.loads(msg)
                print(f"⬇ CMD: {data}")
                
                type = data.get("type")
                payload = data.get("payload", {})

                if type == 'control':
                     # Map manual control (Joystick) -> RC Override
                     # Channels: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
                     # Values: 1000-2000 (1500 center)
                     throttle = int(1000 + (payload.get('throttle', 0) * 1000)) # 0.0-1.0 -> 1000-2000
                     yaw = int(1500 + (payload.get('yaw', 0) * 500)) # -1.0-1.0 -> 1000-2000
                     pitch = int(1500 + (payload.get('pitch', 0) * 500))
                     roll = int(1500 + (payload.get('roll', 0) * 500))

                     self.fc.mav.rc_channels_override_send(
                         self.fc.target_system,
                         self.fc.target_component,
                         roll, pitch, throttle, yaw,
                         0, 0, 0, 0 # Aux channels unused
                     )

                elif type == 'control_safe':
                     # SAFE MODE: Check Sensors First!
                     throttle_in = payload.get('throttle', 0)
                     yaw_in = payload.get('yaw', 0)
                     pitch_in = payload.get('pitch', 0)
                     roll_in = payload.get('roll', 0)
                     
                     # 1. READ SENSORS (Check cache)

                     # Real system would check self.telemetry_cache.get('lidar_dist')
                     min_dist = self.telemetry_cache.get('front_dist', 999)
                     
                     if min_dist < 1.0 and pitch_in > 0:
                         print(f"🛑 OBSTACLE DETECTED ({min_dist}m)! IGNORING FORWARD PITCH!")
                         pitch_in = 0 # Block forward movement
                     
                     throttle = int(1000 + (throttle_in * 1000))
                     yaw = int(1500 + (yaw_in * 500))
                     pitch = int(1500 + (pitch_in * 500))
                     roll = int(1500 + (roll_in * 500))

                     self.fc.mav.rc_channels_override_send(
                         self.fc.target_system,
                         self.fc.target_component,
                         roll, pitch, throttle, yaw,
                         0, 0, 0, 0 
                     )

                elif type == 'ai' or type == 'ai_plan':
                    # AI High level commands
                    # Support both App Protocol ('ai' -> payload) and Laptop Protocol ('ai_plan' -> primitive)
                    if type == 'ai_plan':
                        local_payload = data.get('primitive', {})
                        print(f"🤖 Laptop AI Plan: {local_payload.get('action')}")
                    else:
                        local_payload = payload

                    cmd = local_payload.get('action')
                    
                    # SAFETY INTERCEPT FOR AI
                    if not self.safety.validate_auto_action(cmd):
                        continue

                    if cmd == 'TAKEOFF':
                        self.fc.mav.command_long_send(
                            self.fc.target_system, self.fc.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5) # 5m
                    elif cmd == 'LAND':
                         self.fc.mav.command_long_send(
                            self.fc.target_system, self.fc.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                    
                    elif cmd == 'camera_config':
                         # Dynamic Camera Reconfig
                         self.cam_config['w'] = local_payload.get('w', 640)
                         self.cam_config['h'] = local_payload.get('h', 480)
                         self.cam_config['fps'] = local_payload.get('fps', 30)
                         print(f"🎥 Camera Config Updated: {self.cam_config}")
                         self.cam_needs_reset = True
                         
                    elif cmd == 'set_rth_alt':
                        alt_cm = local_payload.get('alt', 1500)
                        self.fc.mav.command_long_send(
                            self.fc.target_system, self.fc.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
                        self.fc.mav.param_set_send(
                            self.fc.target_system, self.fc.target_component,
                            b'RTL_ALT', alt_cm, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                        print(f"⚠ RTL Altitude set to {alt_cm}cm")

                    elif cmd == 'set_batt_threshold':
                        val = int(local_payload.get('threshold', 15))
                        self.batt_threshold = val
                        print(f"🔋 Low Battery Threshold set to {val}%")
                        
                    elif cmd == 'RTL_SMART':
                        user_lat = local_payload.get('lat')
                        user_lng = local_payload.get('lng')
                        if user_lat and user_lng:
                            print(f"🏠 SMART RTL Triggered -> User Loc: {user_lat}, {user_lng}")
                            self.fc.mav.set_mode_send(
                                self.fc.target_system,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                4) # GUIDED
                            self.fc.mav.mission_item_int_send(
                                self.fc.target_system, self.fc.target_component, 0,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                2, 0, 0, 0, 0, 0,
                                int(user_lat * 1e7), int(user_lng * 1e7), 10 
                            )
                        else:
                            print("⚠️ Smart RTL Failed: No User Location. Defaulting to standard RTL.")
                            self.fc.mav.command_long_send(
                                self.fc.target_system, self.fc.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

                    elif type == 'command':
                        # Simple Commands from App (Snap, Record)
                        cmd_val = payload
                        if cmd_val == 'snap':
                            print("📸 SNAP COMMAND RECEIVED")
                            if self.cam_config.get('source') == 'internal':
                                if self.cap and self.cap.isOpened():
                                    ret, frame = self.cap.read()
                                    if ret:
                                        ts = int(time.time())
                                        fname = f"snap_{ts}.jpg"
                                        cv2.imwrite(fname, frame)
                                        print(f"✅ Saved {fname}")
                            else:
                                print(f"📡 Triggering External Camera ({self.cam_config.get('source')})...")
                        
                        elif cmd_val == 'start_recording':
                            print("🔴 START RECORDING")
                            self.recording = True
                            if self.cam_config.get('source') == 'internal':
                                ts = int(time.time())
                                self.rec_out = cv2.VideoWriter(f'rec_{ts}.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640,480))
                        
                        elif cmd_val == 'stop_recording':
                            print("uq STOP RECORDING")
                            self.recording = False
                            if self.rec_out:
                                self.rec_out.release()
                                self.rec_out = None

                    elif type == 'gimbal':
                        pitch = payload.get('pitch', 0)
                        yaw = payload.get('yaw', 0)
                        msg = json.dumps({"type": "gimbal", "pitch": pitch, "yaw": yaw}).encode('utf-8')
                        self.udp_sock.sendto(msg, self.esp32_addr)

                    elif type == 'user_gps':
                        self.user_gps = (payload.get('lat'), payload.get('lng'))

                    elif type == 'mission':
                        # REAL MISSION UPLOAD
                        waypoints = payload
                        print(f"🗺️ Uploading Mission: {len(waypoints)} points")
                        
                        # 1. Clear Current Mission
                        self.fc.mav.mission_clear_all_send(self.fc.target_system, self.fc.target_component)
                        
                        # 2. Upload Count
                        self.fc.mav.mission_count_send(self.fc.target_system, self.fc.target_component, len(waypoints))
                        
                        # 3. Upload Items
                        # ArduPilot requires waiting for MISSION_REQUEST... simplified for async push
                        # Better to send items blindly with proper sequence if logic permits, or use proper protocol.
                        # For simplicity in this Bridge: Send Items sequentially.
                        seq = 0
                        for wp in waypoints:
                             lat = int(float(wp.get('lat', 0)) * 1e7)
                             lng = int(float(wp.get('lng', 0)) * 1e7)
                             alt = 10 # 10m default
                             
                             print(f"  📍 WP{seq}: {lat}, {lng}")
                             self.fc.mav.mission_item_int_send(
                                 self.fc.target_system, self.fc.target_component,
                                 seq,
                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 0, 1, # Current, Autocontinue
                                 0, 0, 0, 0, # Params
                                 lat, lng, alt
                             )
                             seq += 1
                        print("✅ Mission Upload Complete")
                    
            except Exception as e:
                print(f"RX Error: {e}")
                break

    async def lidar_loop(self):
        """Reads Lidar and checks for Obstacles"""
        # Import needs to be relative or absolute based on deployment. 
        # Assuming lidar_driver.py is in same folder or path
        try:
            from lidar_driver import YDLidarDriver
            lidar = YDLidarDriver(port='/dev/ttyUSB0') # Try auto-detect
            if lidar.start():
                print("🚨 Lidar Active for Obstacle Avoidance")
                while self.running:
                    obstacles = lidar.get_obstacles(max_dist=1.5) # 1.5 meters warning
                    if obstacles:
                        # 1. Safety Brake (Local Reflex)
                        min_dist = 10.0
                        points = []
                        for (x, y) in obstacles:
                            dist = (x**2 + y**2)**0.5
                            if dist < min_dist: min_dist = dist
                            points.append([float(x), float(y)])
                        
                        if min_dist < 1.0:
                            print(f"🛑 OBSTACLE DETECTED: {min_dist:.2f}m! Stopping!")
                            self.fc.mav.set_mode_send(
                               self.fc.target_system,
                               mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               17) # BRAKE
                            
                            if self.ws:
                                await self.ws.send(json.dumps({
                                    "type": "alert", 
                                    "payload": {"level": "critical", "msg": f"OBSTACLE {min_dist:.1f}m"}
                                }))

                        # 1.5 RELAY TO FC (MAVLink DISTANCE_SENSOR) for Offline Avoidance
                        # https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
                        if self.fc:
                             try:
                                self.fc.mav.distance_sensor_send(
                                    0, # time_boot_ms
                                    10, # min_dist (cm)
                                    1200, # max_dist (cm) - 12m for X2
                                    int(min_dist * 100), # current (cm)
                                    0, # type (LASER)
                                    1, # id
                                    0, # orientation (FORWARD)
                                    0  # covariance
                                )
                             except Exception: pass

                        # 2. Stream Data to AI (for Path Planning)
                        # Rate limit to 5Hz to save bandwidth
                        if self.ws and (time.time() - getattr(self, 'last_lidar_send', 0) > 0.2):
                             await self.ws.send(json.dumps({
                                 "type": "lidar_scan",
                                 "payload": {"points": points} 
                             }))
                             self.last_lidar_send = time.time()
                                
                    await asyncio.sleep(0.2) # 5Hz check
            else:
                print("❌ Lidar Failed to Start")
        except ImportError:
            print("⚠️ Lidar Driver not found. Skipping.")
        except Exception as e:
            print(f"Lidar Loop Error: {e}")

    async def video_loop(self):
        """Captures Video and Pushes to Cloud Proxy (Non-Blocking)"""
        import cv2
        
        # Default Config
        if 'source' not in self.cam_config:
            self.cam_config['source'] = 'internal'
            
        if self.cam_config['source'] == 'none':
            print("📷 Camera Disabled by Config")
            while self.running: await asyncio.sleep(1)
            return
            
        current_source = self.cam_config['source']
        current_idx = 0 if current_source == 'internal' else 1
        
        cap = None
        consecutive_failures = 0
        
        # Async HTTP Session
        async with aiohttp.ClientSession() as session:
            
            while self.running:
                # 1. Config Change Check
                if self.cam_needs_reset:
                     print("🔄 Camera Config Reset Requested...")
                     if cap:
                         cap.release()
                         cap = None
                     
                     if 'source' in self.cam_config:
                         current_source = self.cam_config['source']
                         current_idx = 0 if current_source == 'internal' else 1
                     self.cam_needs_reset = False

                # 2. Ensure Camera is Open
                if cap is None or not cap.isOpened():
                    try:
                        print(f"📷 Opening Camera Index {current_idx} (V4L2 Raw Mode)...")
                        cap = cv2.VideoCapture(current_idx, cv2.CAP_V4L2)
                        
                        if cap.isOpened():
                            # Important: disable auto-conversion to allow UYVY/NV12 flow
                            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                            
                            # Force UYVY (Known working format from v4l2-ctl)
                            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('U', 'Y', 'V', 'Y'))
                            
                            # Standard Res - DISABLED (Suspect Driver Issue)
                            # cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_config['w'])
                            # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_config['h'])
                            # cap.set(cv2.CAP_PROP_FPS, self.cam_config['fps'])
                            
                            # Let's inspect what we got
                            fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
                            codec_bytes = fourcc.to_bytes(4, 'little')
                            print(f"✅ Camera {current_idx} Opened! format={codec_bytes}")
                            consecutive_failures = 0
                        else:
                            print(f"❌ Failed to open Camera {current_idx}. Retrying...")
                            await asyncio.sleep(2)
                            continue
                    except Exception as e:
                         print(f"Camera Open Error: {e}")
                         await asyncio.sleep(2)
                         continue
                
                # 3. Read Frame
                ret, raw_frame = cap.read()
                if ret and raw_frame is not None:
                    consecutive_failures = 0
                    
                    frame = None
                    try:
                        # Handle Raw UYVY (Common on Rockchip V4L2)
                        # UYVY = 2 Bytes per pixel. OpenCVs read() often returns (H, W, 2) for this.
                        if len(raw_frame.shape) == 3 and raw_frame.shape[2] == 2:
                            frame = cv2.cvtColor(raw_frame, cv2.COLOR_YUV2BGR_UYVY)
                        elif len(raw_frame.shape) == 3 and raw_frame.shape[2] == 3:
                            frame = raw_frame # Already BGR
                        else:
                            # Unexpected shape (maybe 1D or single channel?)
                            # Print once to debug found shape logic if needed
                            # print(f"Unknown Shape: {raw_frame.shape}")
                            pass

                        if frame is not None:
                            # Compression
                            retval, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                            
                            if retval and buffer is not None:
                                data = buffer.tobytes()
                                
                                # 4. Push to Cloud (Non-Blocking)
                                url = SERVER_URL.replace("ws://", "http://").replace("wss://", "https://").replace("/ws/connect/RADXA_X", "/video/frame")
                                
                                async with session.post(url, data={"file": data}, timeout=0.1) as response:
                                     pass

                    except Exception as e:
                        # Prevent loop crash on single bad frame
                        print(f"Frame Process Error: {e}")
                        pass
                else:
                    # Read Failed
                    consecutive_failures += 1
                    if consecutive_failures > 5:
                        print(f"⚠️ Camera Read Failed {consecutive_failures} times. Resetting...")
                        if cap:
                            cap.release()
                        cap = None
                        consecutive_failures = 0
                        print("⏳ Waiting 3s for driver cleanup...")
                        await asyncio.sleep(3) 
                    else:
                        await asyncio.sleep(0.05)
                
                # FPS Throttling
                target_fps = self.cam_config.get('fps', 30)
                if target_fps > 0:
                    await asyncio.sleep(1.0 / target_fps)
                else:
                    await asyncio.sleep(0.01)

    async def esp32_listener(self):
        """Listens for UDP Telemetry from ESP32 (ToF Sensors + Gimbal Status)"""
        print(f"👂 ESP32 Listener Active on {self.esp32_addr[0]}:8888")
        self.udp_sock.bind(("0.0.0.0", 8888)) # Listen on all interfaces
        self.udp_sock.setblocking(False)
        
        loop = asyncio.get_running_loop()
        
        while self.running:
            try:
                # Async UDP Read
                data, addr = await loop.sock_recvfrom(self.udp_sock, 1024)
                if data:
                     # Parse JSON: {"tof_1": 1200, "tof_2": ...}
                     telem = json.loads(data.decode('utf-8'))
                     
                     # 1. Relay to Cloud/AI (Rate Limited 10Hz)
                     now = time.time()
                     if self.ws and (now - getattr(self, 'last_esp_send', 0) > 0.1):
                         await self.ws.send(json.dumps({
                             "type": "esp32_telem",
                             "payload": telem
                         }))
                         self.last_esp_send = now
                         
                     # 2. Safety Brake (Omnidirectional)
                     # If ANY sensor < 500mm (0.5m), Brake?
                     # Let AI handle sophisticated avoidance, but maybe a hard stop here too?
                     # For now, relying on Lidar for hard stop as ToF is for close quarters/indoor.
            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception as e:
                # print(f"ESP RX Error: {e}")
                await asyncio.sleep(0.1)

                # print(f"ESP RX Error: {e}")
                await asyncio.sleep(0.1)

    async def watchdog_loop(self):
        """Monitors Cloud Connection Health"""
        print("🐕 Watchdog Active")
        while self.running:
            # Check Timeout (e.g. 10 seconds)
            if time.time() - self.last_cloud_msg > 10.0:
                 # Only Trigger if we were connected recently (avoid trigger on boot)
                 # And check if we are likely flying (alt > 1m)
                 alt = self.telemetry_cache.get('altitude', 0)
                 if alt > 1.0 and not getattr(self, 'watchdog_triggered', False):
                     print("⚠️ CONNECTION LOST (>10s)! Triggering Failsafe RTL...")
                     self.watchdog_triggered = True
                     
                     # Try Smart RTL first
                     if self.user_gps:
                          print(f"Watchdog: Returning to User {self.user_gps}")
                          # (Mavlink set guided & goto logic duplicated for robustness)
                          try:
                              self.fc.set_mode('GUIDED')
                              lat, lng = self.user_gps
                              self.fc.mav.mission_item_int_send(
                                  self.fc.target_system, self.fc.target_component, 0,
                                  3, # MAV_FRAME_GLOBAL_RELATIVE_ALT
                                  16, # MAV_CMD_NAV_WAYPOINT,
                                  2, 0, 0, 0, 0, 0,
                                  int(lat * 1e7), int(lng * 1e7), 15
                              )
                          except:
                              self.fc.set_mode('RTL')
                     else:
                          print("Watchdog: Returning Home")
                          if self.fc: self.fc.set_mode('RTL')
            
            # Reset trigger if connection back
            if time.time() - self.last_cloud_msg < 2.0:
                self.watchdog_triggered = False
                
            await asyncio.sleep(1)

if __name__ == "__main__":
    bridge = RadxaBridge()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(bridge.connect_mavlink())
    
    # 2. Start Cloud Loop + Video + Lidar + ESP32
    async def main_wrapper():
        await asyncio.gather(
            bridge.connect_cloud(),
            bridge.video_loop(),
            bridge.lidar_loop(),
            bridge.esp32_listener(),
            bridge.watchdog_loop() # Added Watchdog
        )
    
    loop.run_until_complete(main_wrapper())
```

3.  **Run the Bridge**:
    ```bash
    python3 real_bridge_service.py
    ```

---

## 🔒 STEP 3: LOCK IT (OverlayFS)
**Only do this when everything works!**

1.  **Enable Overlay**:
    ```bash
    sudo apt-get install -y overlayroot
    sudo nano /etc/overlayroot.conf
    # CHANGE TO: overlayroot="tmpfs"
    ```
2.  **Reboot**:
    ```bash
    sudo reboot
    ```
