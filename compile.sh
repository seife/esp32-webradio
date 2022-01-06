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
	PARAM=(--build-property "build.defines=-DAAC_ENABLE_SBR=1")
	# use local deps subdirectory
	PARAM+=(--libraries deps)
fi
# for some reason, this is not always enough to make PSRAM work. Full params are extracted from IDE GUI build
# BOARD_PARAM=:PSRAM=enabled,PartitionScheme=min_spiffs
BOARD_PARAM=:PSRAM=enabled,PartitionScheme=min_spiffs,CPUFreq=240,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none
set -x
arduino-cli "$IAM" \
	-b esp32:esp32:esp32${BOARD_PARAM} "${PARAM[@]}" $@
