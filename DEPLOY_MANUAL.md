# 🚀 FINAL DEPLOYMENT MANUAL (CORRECTED)

**CRITICAL FIX:** I previously pointed you to the wrong folder for the Server. The Server code is in `drone_server`, not `drone_project`.
Please execute **Step 1** again to ensure your Cloud Server actually receives the new logic.

## 1. SERVER (Cloud Relay) - [FIXED STEP]
**You must do this to fix the 403 Forbidden Error.**
```powershell
cd C:\Users\adish\.gemini\antigravity\scratch\drone_server
git add .
git commit -m "Deploying Actual Server Code"
git push origin main
```
*Wait for Render to finish building.*

## 2. DRONE APP (Control Station)
Build the final APK to install on your phone.
```powershell
cd C:\Users\adish\.gemini\antigravity\scratch\drone_app
flutter clean
flutter pub get
flutter build apk --release
```
*Output: `build\app\outputs\flutter-apk\app-release.apk`*

## 3. LAPTOP AI (The "Brain")
Run this from your Laptop to process the AI.
```powershell
cd C:\Users\adish\.gemini\antigravity\scratch\drone_project
# The Director (with auto-import path fix)
python ai/camera_brain/laptop_ai/director_core.py
```

## 4. RADXA ZERO 3W (The Bridge)
Transfer the `drone_project` to the Radxa.
**Correct Command (if specific IP is known, e.g. 192.168.0.14):**
```powershell
# Run from Laptop Terminal
scp -r C:\Users\adish\.gemini\antigravity\scratch\drone_project shreyash@192.168.0.14:/mnt/sd_storage/
```

**Then Update on Radxa:**
```bash
ssh shreyash@192.168.0.14
cd /mnt/sd_storage/drone_project
sudo systemctl restart drone-bridge
```
