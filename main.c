#include <wiringPi.h>

#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "driver/display/pcf8574at/pcf8574at.h"
#include "driver/display/hx8357c/hx8357c.h"
#include "driver/display/oled/oled.h"
#include "driver/display/ili9225c/ili9225c.h"
#include "driver/sensor/mpu9250/mpu9250.h"
#include "utils/logic/stringUtils/stringUtils.h"
#include "dss.h"
#include "gui/vehicleGUI/vehicleGUI.h"

void setup() {
    int fd_gpio=wiringPiSetupGpio();

    int led=18;
    pinMode(led,OUTPUT);
    digitalWrite(led,1);

    MPU9250_setupComs(0x69,0x0C);
	MPU9250_setupSensors();

    VGUI_create();
    dss_main();
}

void loop() {

}

int main() {
    setup();
    //while(1) loop();
    return 0;
}