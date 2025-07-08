#!/bin/bash
# Build and run on hardware
PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_30\:ED\:A0\:A8\:D7\:A8-if00
mosquitto_pub -h 192.168.68.137 -t cmnd/tasmota_71D51D/POWER -m OFF 
cp core1.cpp stash/core1.cpp.`date +%Y%m%d.%H%M%S`
make PORT=${PORT} upload
( sleep 3 && mosquitto_pub -h 192.168.68.137 -t cmnd/tasmota_71D51D/POWER -m ON ) &
make PORT=${PORT} cat | cat_until DONE


