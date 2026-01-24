import serial
import time
import sys

import glob

# CONFIG
BAUDS = [57600, 19200, 115200, 38400, 9600, 921600, 230400, 460800]
# Find all potential serial ports
PORTS = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyAML*') + glob.glob('/dev/ttyUSB*')

print(f"🕵️ GOD SCANNER: Checking {len(PORTS)} Ports x {len(BAUDS)} Speeds...")
print(f"------------------------------------------------")

for port in PORTS:
    # Skip console if possible
    if "ttyS0" in port: continue 
    
    print(f"🔎 Checking Port: {port}")
    
    for baud in BAUDS:
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
            # Wake up
            ser.setRTS(False); ser.setDTR(False); time.sleep(0.05)
            ser.setRTS(True); ser.setDTR(True)
            
            # Listen for Heartbeat (1Hz) - Need at least 2s window
            start_time = time.time()
            data_found = b""
            
            while time.time() - start_time < 2.5:
                if ser.in_waiting > 0:
                    chunk = ser.read(ser.in_waiting)
                    data_found += chunk
                    if len(data_found) > 20: break
                time.sleep(0.01)
                
            ser.close()
            
            if len(data_found) > 0:
                hex_str = data_found.hex()
                print(f"   ✨ ACTIVITY FOUND @ {baud}!")
                print(f"      Bytes: {len(data_found)} | Sample: {hex_str[:20]}...")
                
                if "fe" in hex_str or len(data_found) > 10:
                     print(f"      🎉 LOCK CONFIRMED: Port {port}, Baud {baud}")
                     print(f"      👉 ACTION: Update your code/config to use THIS.")
                     sys.exit(0)
        except OSError:
            pass # Port busy or doesn't exist
        except Exception as e:
            # print(f"   ⚠️ {e}")
            pass

print("\n❌ TOTAL SILENCE. God Scanner found nothing.")
print("   This confirms the FC is electrically disconnected or completely silent.")
