#!/bin/bash
# enable_uart.sh
# Fixes "No such file or directory: /dev/ttyS2"
# Adding the missing Overlay for UART2

FILE="/boot/armbianEnv.txt"

echo "🔧 Checking $FILE..."

# Check if line exists
if grep -q "overlays=uart2-m0" "$FILE"; then
    echo "✅ UART2 Overlay already present."
else
    echo "➕ Adding 'overlays=uart2-m0'..."
    # Check if 'overlays=' exists but is something else, or if it's empty
    # For safety on a fresh install, we just append or create
    echo "overlays=uart2-m0" | sudo tee -a "$FILE"
    echo "✅ Added."
fi

# Verify
echo "📄 Current Config:"
tail -n 5 "$FILE"

echo "========================================="
echo "⚠️  REBOOT REQUIRED to load overlay!"
echo "========================================="
