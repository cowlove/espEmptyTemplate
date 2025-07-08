#!/bin/bash
# Build and run on hardware
cp core1.cpp stash/core1.cpp.`date +%Y%m%d.%H%M%S`   && make PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_30\:ED\:A0\:A8\:D7\:A8-if00 uc

