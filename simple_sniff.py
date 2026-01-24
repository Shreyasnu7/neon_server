
import serial
import time
import sys

print("🔍 Opening /dev/ttyS2 to sniff for ANY data...")

try:
    # Try common baud rates
    ser = serial.Serial('/dev/ttyS2', 57600, timeout=1)
    
    print("✅ Port Opened. Listening for 10 seconds...")
    start = time.time()
    total_bytes = 0
    
    while time.time() - start < 10:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            print(f"📨 RECEIVED {len(data)} bytes: {data}")
        time.sleep(0.1)
        
    print(f"🏁 Finished. Total Bytes: {total_bytes}")
    
    if total_bytes == 0:
        print("❌ SILENCE. No data received.")
    else:
        print("✅ SUCCESS! Data is flowing.")

except Exception as e:
    print(f"❌ CRITICAL ERROR: {e}")
