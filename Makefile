BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
CHIP ?= esp32

ALIBS=${HOME}/Arduino/libraries
EXCLUDE_DIRS=${ALIBS}/lvgl|${ALIBS}/LovyanGFX|${ALIBS}/esp32csim|${ALIBS}/PubSubClient/tests
PART_FILE=${ESP_ROOT}/tools/partitions/min_spiffs.csv
GIT_VERSION := "$(shell git describe --abbrev=6 --dirty --always)"

ESP_ROOT=${HOME}/src/arduino-esp32
#ESP_ROOT=${HOME}/src/esp32
ARDUINO_LIBS="${ESP_ROOT}/libraries ${HOME}/Arduino/libraries"



ifeq (${BOARD},esp32s3)
	# EventsCore=0,PSRAM=opi,CDCOnBoot=cdc
	CDC_ON_BOOT=1
	BUILD_EXTRA_FLAGS += -DBOARD_HAS_PSRAM 
	BUILD_MEMORY_TYPE=qio_opi
	PORT ?= /dev/ttyACM0
else
	BUILD_MEMORY_TYPE=qio_qspi
	#BUILD_EXTRA_FLAGS += -fpermissive
	PORT ?= /dev/ttyUSB0
endif
UPLOAD_PORT ?= ${PORT}

BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\" -O3
BUILD_EXTRA_FLAGS += -mno-serialize-volatile
BUILD_EXTRA_FLAGS += ${DEF}

#sed  's|^\(.*/srmodels.bin\)|#\1|g' -i ~/.arduino15/packages/esp32/hardware/esp32/3.2.0/boards.txt  

csim-build:
	${MAKE} BOARD=csim csim 

ifeq ($(BOARD),csim)
CSIM_BUILD_DIR=./build/csim
CSIM_LIBS=Arduino_CRC32 ArduinoJson Adafruit_HX711 esp32jimlib
include ${ALIBS}/esp32csim/csim.mk
else
include ${ALIBS}/makeEspArduino/makeEspArduino.mk
#include ./makeEspArduino/makeEspArduino.mk
endif

1cat:
	stty -F ${PORT} -echo raw 921600 && cat ${PORT} 
cat:    
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 921600 && cat ${PORT}; fi; done  | tee ./cat.`basename ${PORT}`.out
socat:  
	socat udp-recvfrom:9000,fork - 
mocat:
	mosquitto_sub -h rp1.local -t "${MAIN_NAME}/#" -F "%I %t %p"   
uc:		pbirom.h
	${MAKE} upload && ${MAKE} cat

u1c:		pbirom.h
	${MAKE} upload && sleep 1 && ${MAKE} 1cat

cuc:	pbirom.h
	${MAKE} clean && ${MAKE} upload && ${MAKE} cat

backtrace:
	tr ' ' '\n' | addr2line -f -i -e ./build/${BOARD}/*.elf

pbirom.h:  pbirom.o65
	./bin2src.py pbirom.o65 > pbirom.h

pbirom.o65:  pbirom.asm
	xa pbirom.asm -o pbirom.o65

