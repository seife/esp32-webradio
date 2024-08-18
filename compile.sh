#!/bin/bash

IAM=${0##*/}
IAM=${IAM%.sh}

PARAM=()
if [ "$IAM" = upload ]; then
	if [ -z "$1" ]; then
		echo "please give upload port!"
		exit 1
	fi
	PARAM=(-v -p "$1")
	shift
else
	# this avoids having to change the config of ESP32-audioI2S
	PARAM=(--build-property "build.extra_flags.esp32=-DARDUINO_USB_CDC_ON_BOOT=0 -DSBR_DEC=1")
	# use local deps subdirectory
	PARAM+=(--libraries deps)
fi
# for some reason, this is not always enough to make PSRAM work. Full params are extracted from IDE GUI build
#BOARD_PARAM=esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=min_spiffs
#BOARD_PARAM=esp32:esp32:esp32:PartitionScheme=min_spiffs
BOARD_PARAM=esp32:esp32:esp32wrover:PartitionScheme=min_spiffs
set -x
arduino-cli "$IAM" \
	-b ${BOARD_PARAM} "${PARAM[@]}" $@
