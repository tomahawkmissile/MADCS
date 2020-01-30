#!/bin/bash
echo 'Compiling....'

gcc main.c \
    driver/display/hx8357c/hx8357c.c \
    utils/colors/colors.c \
    utils/logic/swap/swap.c \
    driver/display/pcf8574at/pcf8574at.c \
    driver/display/ili9225c/ili9225c.c \
    utils/symbols.c \
    driver/display/oled/oled.c \
    utils/logic/stringUtils/stringUtils.c \
    utils/logic/graphics/image/image.c \
    driver/sensor/mpu9250/mpu9250.c \
    dss.c \
    displayAbstractionLayer.c \
    gui/vehicleGUI/vehicleGUI.c \
     -g -lwiringPi -lm -lpthread -o LRos.o