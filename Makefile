BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
CHIP ?= esp32
VERBOSE=1
ALIBS=${HOME}/Arduino/libraries
EXCLUDE_DIRS=${ALIBS}/lvgl|${ALIBS}/LovyanGFX|${ALIBS}/esp32csim
PART_FILE=${ESP_ROOT}/tools/partitions/min_spiffs.csv
GIT_VERSION := "$(shell git describe --abbrev=6 --dirty --always)"

CDC_ON_BOOT=1
ifeq (${BOARD},esp32s3)
        CDC_ON_BOOT=1
        BUILD_MEMORY_TYPE=qio_opi
else
        BUILD_MEMORY_TYPE=qio_qspi
endif


#BUILD_EXTRA_FLAGS += -DARDUINO_PARTITION_huge_app 
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\" 
	
#sed  's|^\(.*/srmodels.bin\)|#\1|g' -i ~/.arduino15/packages/esp32/hardware/esp32/3.2.0/boards.txt  



UPLOAD_PORT ?= /dev/ttyACM0

ifeq ($(BOARD),csim)
CSIM_BUILD_DIR=./build/csim
CSIM_LIBS=Arduino_CRC32 ArduinoJson Adafruit_HX711 esp32jimlib
include ${ALIBS}/esp32csim/csim.mk
else
	include ~/Arduino/libraries/makeEspArduino/makeEspArduino.mk
endif

cat:    
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done  | tee ./cat.`basename ${PORT}`.out
socat:  
	socat udp-recvfrom:9000,fork - 
mocat:
	mosquitto_sub -h rp1.local -t "${MAIN_NAME}/#" -F "%I %t %p"   
uc:
	${MAKE} upload && ${MAKE} cat

backtrace:
	tr ' ' '\n' | addr2line -f -i -e ./build/${BOARD}/*.elf


