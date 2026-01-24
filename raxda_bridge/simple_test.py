import time
import sys
from pymavlink import mavutil
import os

print(f"🆔 Process ID: {os.getpid()}")
print("========================================")
print("🚀 SIMPLE MAVLINK TEST (No threads/async)")
print("========================================")

FC_PORT = '/dev/ttyS2'
# Try the one that gave "Port Error" first (might be lively), then standard.
BAUDS = [19200, 9600, 57600, 38400, 115200]

for baud in BAUDS:
    print(f"\n🔌 Testing Baud: {baud}...")
    try:
        # Create connection (Source System 255 = GCS)
        conn = mavutil.mavlink_connection(FC_PORT, baud=baud, source_system=255)
        
        # Send Heartbeats aggressively
        print("   💓 Sending Heartbeats...", end='', flush=True)
        for i in range(5):
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            conn.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            time.sleep(0.1)
            print(".", end='', flush=True)
        print(" Done.")

        # Listen
        print("   👂 Waiting for Heartbeat (5s)...")
        hb = conn.wait_heartbeat(timeout=5)
        
        if hb:
            print(f"   ✅ SUCCESS! Heartbeat received from SysID {conn.target_system}!")
            print(f"   ✅ Baud Rate {baud} is CORRECT.")
            
            # Request all data
            print("   Requesting Data Stream...")
            conn.mav.request_data_stream_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            
            # Read a few messages
            start = time.time()
            while time.time() - start < 3:
                msg = conn.recv_match(blocking=False)
                if msg:
                    print(f"   📨 Msg: {msg.get_type()}")
            sys.exit(0) # Exit with success
        else:
            print("   ❌ Timeout (No Heartbeat).")
            # SNIFFER MODE: Read raw junk to debug
            try:
                n = conn.mav.file.inWaiting()
                if n > 0:
                    data = conn.mav.file.read(n)
                    print(f"   🕵️ SNIFFER: Received {n} bytes of RAW DATA: {data}")
                    print("   (Data found but not MAVLink: Check Baud or Inversion/Option 8)")
            except: pass
            conn.close()

    except Exception as e:
        print(f"   ⚠️ ERROR: {e}")

print("\n🏁 Test Complete. No Connection.")
