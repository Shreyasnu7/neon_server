import serial
import time
import subprocess
import sys

# Standard MAVLink Bauds + High Speed
BAUDS = [57600, 115200, 38400, 921600, 460800, 230400, 19200, 9600]
PORT = '/dev/ttyS2'

def configure_port(baud):
    """Force Raw Mode via stty (Linux Driver Level)"""
    cmd = f"stty -F {PORT} {baud} -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke raw"
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def sniff_baud(baud):
    print(f"🔎 Scanning @ {baud} bps...", end='', flush=True)
    configure_port(baud)
    
    try:
        # Timeout 2s is enough to catch a heartbeat (1Hz)
        with serial.Serial(PORT, baud, timeout=2.0) as ser:
            start = time.time()
            data_buffer = b""
            
            while time.time() - start < 2.5:
                chunk = ser.read(100) # Big chunk
                if chunk:
                    data_buffer += chunk

            # Analyze
            if len(data_buffer) == 0:
                print(" ❌ No Data (Silence)")
                return False
            
            non_null = [b for b in data_buffer if b != 0x00]
            
            if len(non_null) == 0:
                 # We got data, but it was ALL ZEROS (0x00) -> Break/Ground Short
                 print(f" ⚠️  Received {len(data_buffer)} bytes, but ALL were NULL (0x00). (Line Stuck Low?)")
                 return False
            
            # We got REAL data
            print(f" ✅ DATA FOUND! ({len(data_buffer)} bytes)")
            print(f"    Sample: {data_buffer[:20].hex()}")
            
            # Check for MAVLink Magic
            if b'\xfe' in data_buffer or b'\xfd' in data_buffer:
                print("    🎯 MAVLINK MAGIC DETECTED! (0xFE or 0xFD)")
            return True

    except Exception as e:
        print(f" Error: {e}")
        return False

print("=== 🕵️ ULTIMATE SERIAL SCANNER ===")
print(f"Target: {PORT}")

for baud in BAUDS:
    if sniff_baud(baud):
        print(f"\n🎉 MATCH FOUND AT {baud}!")
        print("Update your code to use this baud rate.")
        sys.exit(0)
    time.sleep(0.5)

print("\n❌ Scan Complete. No valid data found on ANY baud rate.")
print("Diagnosis: Wiring Issue (RX/TX Swapped or Ground Short).")
