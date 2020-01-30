#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "string.h"

#include "../../../utils/symbols.h"
#include "pcf8574at.h"

unsigned int pcf8574at_pins[8];

int fd;
//Pins in order from 0-7 : RS, RW, EN, BT, D4, D5, D6, D7
void init(char addr,unsigned int p[8]) {
  memcpy(pcf8574at_pins,p,sizeof(pcf8574at_pins));
  fd=wiringPiI2CSetup(addr);
}

void i2c_write(char data) {
  wiringPiI2CWrite(fd,data);
}

void setBacklight(unsigned char b) {
    if(b>0x1) b=0x1;
    //i2c_write(0x04);
    i2c_write(0x00);
}