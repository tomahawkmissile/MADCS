#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../utils/colors/colors.h"
#include "../../driver/display/pcf8574at/pcf8574at.h"
#include "../../driver/display/hx8357c/hx8357c.h"
#include "../../driver/display/oled/oled.h"
#include "../../driver/display/ili9225c/ili9225c.h"
#include "../../driver/sensor/mpu9250/mpu9250.h"
#include "../../utils/logic/stringUtils/stringUtils.h"
#include "../../utils/logic/graphics/image/image.h"

static int h=0;
//D0,D1,D2,D3,D4,D5,D6,D7,RST,CS,RS,WR,RD
static int pinout[13]={34,35,28,29,30,31,32,33,41,42,43,44,45};
static unsigned int altColor = 0xFFFF;

void VGUI_create() {
    hx8357c_setConsole(0,0,320,480);
    hx8357c_setRotation(2);
    memcpy(hx8357c_pins,pinout,sizeof(pinout));
    hx8357c_setPinStates();
    hx8357c_init();
    hx8357c_clear(0x0000);
    hx8357c_printConsole("Booting system...",altColor,THEME_BG);
    delay(1000);
    hx8357c_clear(0x0000);
    
}