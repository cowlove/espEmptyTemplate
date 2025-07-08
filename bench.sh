#!/bin/bash -e 
stdbuf -o0 make \
    PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_B4\:3A\:45\:A5\:C4\:2C-if00 \
    "DEF=-DFAKE_CLOCK $1" \
    u1c | stdbuf -i0 -o0 cat_until "DONE" | tee timings.out

mv timing.txt timing.last.txt
grep HIST timings.out  > timing.txt

./compareTimings.sh timing.last.txt timing.txt 

#gnuplot -e "set term dumb; plot 'timing_committed.txt' u 1:2 w l, 'timing.txt' u 1:2 w l;"

