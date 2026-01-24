#!/bin/bash
# fix_uart_final.sh
# Fixes the "No such file /dev/ttyS2" by cleanly rewriting armbianEnv.txt
# (Previous attempts caused duplicate lines which broke the bootloader config)

FILE="/boot/armbianEnv.txt"

echo "🔧 FIXING BOOT CONFIGURATION..."

# 1. Extract the Root UUID (Crucial for booting)
# We grep the existing file for the UUID.
UUID=$(grep "rootdev=UUID=" $FILE | head -n 1)

if [ -z "$UUID" ]; then
    echo "❌ CRITICAL: Could not find Root UUID! Aborting to prevent bricking."
    exit 1
fi

echo "✅ Found UUID: $UUID"

# 2. Write a CLEAN file (No duplicates)
# We overwrite the file completely.
cat <<EOF | sudo tee $FILE
verbosity=1
bootlogo=false
console=both
$UUID
rootfstype=ext4
overlays=uart2-m0
usbstoragequirks=0x2537:0x1066:u,0x2537:0x1068:u
EOF

echo "📄 New Config:"
cat $FILE
echo "========================================="
echo "✅ CONFIG REPAIRED. REBOOT NOW."
echo "========================================="
