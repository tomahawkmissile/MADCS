extern unsigned int pcf8574at_pins[8];

void init(char addr,unsigned int p[8]);
void i2c_write(char data);
void setBacklight(unsigned char b);