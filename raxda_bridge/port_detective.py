import serial
import time
import glob
import os

print("🕵️ PORT DETECTIVE STARTED")
print(f"🆔 PID: {os.getpid()}")

# common baud rates
BAUDS = [57600, 115200, 38400, 19200, 9600]
ports = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyAMA*')

found_data = False

for port in ports:
    print(f"\n🔎 Checking {port}...")
    try:
        # Quick check at 57600 first
        s = serial.Serial(port, 57600, timeout=1)
        data = s.read(100)
        s.close()
        
        if len(data) > 0:
            print(f"   ✅ FOUND DATA on {port} (Len: {len(data)})!")
            print(f"   bytes: {data[:20]}")
            found_data = True
            # Scan bauds on this port
            for b in BAUDS:
                try:
                    s = serial.Serial(port, b, timeout=2)
                    print(f"      Attempting Baud {b}...", end='')
                    d = s.read(50)
                    s.close()
                    if len(d) > 0:
                        # Check for MAVLink magic (0xFE or 0xFD)
                        if b'\xfe' in d or b'\xfd' in d:
                            print(f" 🎯 MATCH! MAVLink Header detected!")
                        else:
                            print(f" (Data found, unknown format)")
                    else:
                        print(" Silence")
                except:
                    pass
        else:
            print("   ❌ Silence.")
            
    except OSError as e:
        print(f"   ⚠️ Busy/Error: {e}")
    except Exception as e:
        print(f"   ⚠️ Error: {e}")

if not found_data:
    print("\n💀 NO DATA FOUND ON ANY PORT.")
    print("Possibilities:")
    print("1. RX/TX Swapped (Physically)")
    print("2. brltty service is hijacking ports")
    print("3. FC is not sending data (wrong TELEM config)")
