#!/bin/sh
# Install the PiStorms system files on the Pi (run again after any change):
#   sudo PiStorms/system/install.sh
set -e
cd "$(dirname "$0")"

# Power-off hook: cuts the board power at the end of "poweroff"
install -m 755 pistorms-poweroff /usr/lib/systemd/system-shutdown/pistorms-poweroff

# Battery monitor: LEDs blink, then shutdown (thresholds per battery type:
# pistorms/scripts/batteries_monitor.py). It runs as user robot, which may
# then power the Pi off without a password (polkit rule, nothing else)
install -m 644 psm_batt_monitor.service /etc/systemd/system/
install -m 644 50-robot-poweroff.rules /etc/polkit-1/rules.d/

# LiDAR: /dev/rplidar + motor stopped while no program uses it
install -m 644 lidar-pause.service /etc/systemd/system/
install -m 644 99-rplidar.rules /etc/udev/rules.d/

systemctl daemon-reload
systemctl enable --now psm_batt_monitor.service
udevadm control --reload-rules
# Apply the rule to a LiDAR already plugged in (starts lidar-pause)
udevadm trigger --action=add --subsystem-match=tty --sysname-match='ttyUSB*'

echo "Installed. Check with: systemctl status psm_batt_monitor lidar-pause"
