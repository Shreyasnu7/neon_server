
import asyncio
import time
from pymavlink import mavutil

class AutopilotController:
    """
    Connects to the Flight Controller (FC) via MAVLink.
    Executed by DirectorCore to move the physical drone.
    
    Port: /dev/ttyACM0 (Radxa -> FC) or udpin:localhost:14550 (SITL)
    """
    def __init__(self, connection_string="/dev/ttyACM0", baud=57600):
        self.conn_str = connection_string
        self.baud = baud
        self.master = None
        self.connected = False
        
    def connect(self):
        try:
            print(f"🔌 Connecting to Flight Controller on {self.conn_str}...")
            self.master = mavutil.mavlink_connection(self.conn_str, baud=self.baud)
            self.master.wait_heartbeat(timeout=5)
            self.connected = True
            print("✅ Flight Controller CONNECTED via MAVLink!")
        except Exception as e:
            print(f"⚠️ MAVLink Connection Failed: {e}")
            self.connected = False

    def send_velocity(self, vx, vy, vz, yaw_rate=0):
        """
        Send LOCAL frame velocity commands (NED convention).
        Input: meters/second
        """
        if not self.connected or not self.master:
            return 
            
        # Create SET_POSITION_TARGET_LOCAL_NED message
        # type_mask: ignore pos, accel, only accept vel
        type_mask = 0b0000111111000111
        
        self.master.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0, # x, y, z positions (ignored)
            vx, vy, vz, # velocities
            0, 0, 0, # accel (ignored)
            0, # yaw (ignored)
            yaw_rate # yaw_rate
        )

    def execute_plan_point(self, position, yaw):
        """
        Executes a 3D point from the UltraDirector curve.
        """
        # For simplicity in this v1, we map position changes to velocity
        # Or if position control is desired:
        if not self.connected: return
        
        # Implementation of position setpoint...
        pass
