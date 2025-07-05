#!/bin/bash
stdbuf -o0 make \
    PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_B4\:3A\:45\:A5\:C4\:2C-if00 \
    DEF=-DFAKE_CLOCK \
    u1c | stdbuf -i0 -o0 cat_until "DONE" | tee cat.timings.out

grep HIST cat.timings.out  > timing.txt
gnuplot -e "set term dumb; plot 'timing_committed.txt' u 1:2 w l, 'timing_committed.txt' u 1:3 w l, 'timing.txt' u 1:2 w l, 'timing.txt' u 1:3 w l;"

#gnuplot -e "set term dumb; plot 'timing_committed.txt' u 1:2 w l, 'timing.txt' u 1:2 w l;"

