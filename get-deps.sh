#!/bin/bash

DEPS_DIR=deps/
DEPS=(
	https://github.com/schreibfaul1/ESP32-audioI2S.git
	https://github.com/maditnerd/es8388
	https://github.com/mathertel/RotaryEncoder.git
	https://github.com/ThingPulse/esp8266-oled-ssd1306
)

if ! [ -d "$DEPS_DIR" ]; then
	if ! mkdir "$DEPS_DIR"; then
		echo "cannot create $DEPS_DIR"
		exit 1
	fi
fi
if ! cd "$DEPS_DIR"; then
	echo "cannot chdir to $DEPS_DIR"
	exit 1
fi

set -e
set -x # debug
for i in "${DEPS[@]}"; do
	gname=${i##*/}
	gname=${gname%.git}
	if [ -d "$gname" ]; then
		( cd $gname && git pull --ff-only )
	else
		git clone $i
	fi
done
