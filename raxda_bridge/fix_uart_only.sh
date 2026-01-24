#!/bin/bash
# fix_uart_only.sh
# Enables UART2 for the Flight Controller

ENV_FILE="/boot/armbianEnv.txt"

echo "Checking UART config in $ENV_FILE..."

# Check if overlay is missing
if ! grep -q "overlays=uart2-m0" "$ENV_FILE"; then
    echo "🔧 Enabling UART2..."
    # Append the overlay config
    echo "overlays=uart2-m0" | sudo tee -a "$ENV_FILE" > /dev/null
    echo "✅ UART2 enabled."
else
    echo "✅ UART2 is already enabled."
fi

echo "⚠️  You MUST reboot for this to work."
