#!/bin/bash

WORKSPACE=$PWD

source setup_env.sh


wget https://codeload.github.com/Makeblock-official/Makeblock-Libraries/zip/master -O $WORKSPACE/packages/Makeblock-Libraries-master.zip
export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=1
arduino-cli lib install --zip-path $WORKSPACE/packages/Makeblock-Libraries-master.zip

arduino-cli core install arduino:megaavr
arduino-cli core install arduino:avr