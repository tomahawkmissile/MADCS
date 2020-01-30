#include <wiringPi.h>

#include <mcp23017.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "utils/colors/colors.h"
#include "driver/display/pcf8574at/pcf8574at.h"
#include "driver/display/hx8357c/hx8357c.h"
#include "driver/display/oled/oled.h"
#include "driver/display/ili9225c/ili9225c.h"
#include "driver/sensor/mpu9250/mpu9250.h"
#include "utils/logic/stringUtils/stringUtils.h"
#include "utils/logic/graphics/image/image.h"

//Supported screens:
enum display_model {
    ILI9225C, //SPI
    HX8357C, //8-bit parallel
    OLED, //I2C
};

static enum display_model model;
static char buffering=1;

void DAL_setDisplayModel(enum display_model m) {
    model=m;
}
enum display_model DAL_getDisplayModel() {
    return model;
}
void DAL_setBuffering(char Nbuffering) {
    buffering=Nbuffering;
}
char DAL_getBuffering() {
    return buffering;
}

void DAL_writePixel(int x,int y,int color) {
    switch(model) {
        case ILI9225C:
            if(buffering) {
                ili9225c_drawPixel(x,y,color);
            } else {
                ili9225c_drawPixelDirect(x,y,color);
            }
            break;
        case HX8357C:
            if(buffering) {

            } else {
                hx8357c_drawPixel(x,y,color);
            }
            break;
        case OLED:
            if(buffering) {
                oled_writePixel(x,y,color);
            } else {
                oled_writePixel(x,y,color);
                oled_displayBuffer();
            }
            break;
        default: break;
    }
}
