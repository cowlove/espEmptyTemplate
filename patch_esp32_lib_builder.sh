#!/bin/bash

# docker run -it jim-agit_saha_1 -v ${HOME}/src/esp32/:/arduino-esp32 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host  
# 
# docker cp patch_esp32_lib_builder.sh interesting_beaver:/opt/esp/lib-builder/

#how to patch docker container
cd /opt/esp/lib-builder/ 
git clone https://github.com/hathach/tinyusb.git /opt/esp/lib-builder/components/arduino_tinyusb/tinyusb
git -C /opt/esp/lib-builder/components/arduino_tinyusb/tinyusb checkout 72b1fc50e
sed -i  's/exit 1/exit 0/' ./tools/update-components.sh 
grep CONFIG_ESP_INT_WDT_CHECK_CPU1=n ./configs/defconfig.common || echo CONFIG_ESP_INT_WDT_CHECK_CPU1=n >> configs/defconfig.common



