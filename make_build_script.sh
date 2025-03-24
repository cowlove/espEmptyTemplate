#!/bin/bash
BOARD=ttgo-t1
TMP=/tmp/arduino/$$.txt
PORT=/dev/ttyUSB0
BOARD_OPTS=PartitionScheme=min_spiffs

cd `dirname $0`
arduino-cli compile  -v \
    -b esp32:esp32:${BOARD} -u --port ${PORT} \
	 | tee $TMP

SKETCH=`basename \`pwd\``
SKETCHDIR=/tmp/arduino/sketches/`rematch '/tmp/arduino/sketches/([A-Z0-9]+)/' $TMP | head -1`
SKETCHCPP="${SKETCHDIR}/sketch/${SKETCH}.ino.cpp"
OUT=./build-${BOARD}.sh

# TODO: this is missing a link command        
COMPILE_CMD=`egrep "[-]o ${SKETCHCPP}.o" $TMP`
LINK_CMD1=`egrep " cr ${SKETCHDIR}/sketch/objs.a" $TMP`
LINK_CMD2=`egrep "[-]o ${SKETCHDIR}/${SKETCH}.ino.elf" $TMP`
LINK_CMD3=`grep esptool_py $TMP | grep -v ${PORT} | head -1 | tr '\n' ' '`
LINK_CMD4=`grep esptool_py $TMP | grep -v ${PORT} | tail -1 | tr '\n' ' '`
UPLOAD_CMD=`grep esptool_py $TMP | grep ${PORT} | tr '\n' ' '`

cat <<END > $OUT
#!/bin/bash 
OPT=\$1; if [ "\$OPT" == "" ]; then OPT="-clwum"; fi
set -e
if [[ \$OPT == *c* ]]; then
	echo -n Compiling...
	echo '#include <Arduino.h>' > ${SKETCHCPP}
	echo "#line 1 \"`pwd`/${SKETCH}.ino\"" >> ${SKETCHCPP}
	cat "${SKETCH}.ino" >> ${SKETCHCPP}
	time $COMPILE_CMD
fi

if [[ \$OPT == *l* ]]; then
	echo Linking...
	$LINK_CMD1
	$LINK_CMD2
	$LINK_CMD3
	$LINK_CMD4  
fi

if [[ \$OPT == *u* ]]; then
	echo Uploading... 
	if [[ \$OPT == *w* ]]; then echo -n Waiting for ${PORT}...; while [ ! -e ${PORT} ]; do sleep 1; done; echo OK; fi;
	$UPLOAD_CMD 
fi

if [[ \$OPT == *m* ]]; then
	echo Monitoring...
	if [[ \$OPT == *w* ]]; then echo -n Waiting for ${PORT}...; while [ ! -e ${PORT} ]; do sleep 1; done; echo OK; fi;
	stty -F ${PORT} 115200 raw -echo && cat ${PORT}
fi;


END

chmod 755 $OUT
#rm -f $TMP
