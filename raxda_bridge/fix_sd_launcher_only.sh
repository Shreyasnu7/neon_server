#!/bin/bash
# fix_sd_launcher_only.sh
# Fixes the syntax error in run_clean.sh on the SD card
# adhering to NO INTERNAL MEMORY USAGE for the script itself.

TARGET="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

echo "Reparing $TARGET..."

# Use printf to write the file line-by-line to avoid heredoc/CRLF issues
printf "#!/bin/bash\n" > "$TARGET"
printf "echo '🚀 Starting Bridge from SD Card...'\n" >> "$TARGET"
printf "\n" >> "$TARGET"
printf "# Use System Python (robust) to run Code on SD Card (persistent)\n" >> "$TARGET"
printf "sudo /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py\n" >> "$TARGET"

chmod +x "$TARGET"
echo "✅ Fixed run_clean.sh on the SD card."
