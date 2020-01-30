#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <string.h>
#include <stdlib.h>

#include "../../../utils/colors/colors.h"
#include "../../../utils/logic/swap/swap.h"
#include "../../../utils/symbols.h"
#include "../../../libs/stb/stb_image.h"

#define THEME_FG 0xFFFF
#define THEME_BG 0x0000

int SDI=0;
int CLK=0;
int DC=0;
int RESET=0;
int LED=0;
int CS=0;

int maxX=176;
int maxY=220;

int ili9225c_fd;

void ili9225c_setPins(int sdi,int clk,int dc,int rst,int led,int cs) {
	SDI=sdi;
	CLK=clk;
	DC=dc;
	RESET=rst;
	LED=led;
	CS=cs;
}

void ili9225c_setBacklight(int value) {
	digitalWrite(LED,value);
}
static void setDC(int value) {
	digitalWrite(DC,value);
}
static void setReset(int value) {
	digitalWrite(RESET,value);
}
static void write(unsigned char data) {
	if(SDI>0) {
		for(unsigned char c=0x80;c;c>>=1) {
			if(data&c) {
				digitalWrite(SDI,1);
			} else {
				digitalWrite(SDI,0);
			}
			digitalWrite(CLK,1);
			digitalWrite(CLK,0);
		}
	} else {
		wiringPiSPIDataRW(0,&data,sizeof(unsigned char));
	}
}

typedef struct loadstr loadstr;
struct loadstr {
  int width;
  int height;
  unsigned char* data;
};
static struct loadstr ili9225c_loadman(const char* file) {
	int width,height,bpp;
	unsigned char* pixels=stbi_load(file,&width,&height,&bpp,STBI_rgb);
	return (loadstr){ .width=width,.height=height,.data=pixels };
}

static void writeCommand(unsigned char cmd) {
	setDC(0);
	write(cmd);
}
static void writeData(unsigned char data) {
	setDC(1);
	write(data);
}
static void writeRegister(unsigned char reg,unsigned int value) {
	writeCommand(reg);
	writeData(value>>8);
	writeData(value&0xFF);
}

static void setWindow(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
	writeRegister(0x36,x1);
	writeRegister(0x37,x0);

	writeRegister(0x38,y1);
	writeRegister(0x39,y0);

	writeRegister(0x20,x0);
	writeRegister(0x21,y0);

	writeCommand(0x22);
}
static void setDisplay(int flag) {
	if (flag) {
		writeRegister(0x00FF, 0x0000);
		writeRegister(0x0010, 0x0000);
		delay(50);
		writeRegister(0x0007, 0x1017);
		delay(200);
	} else {
		writeRegister(0x00ff, 0x0000);
		writeRegister(0x0007, 0x0000);
		delay(50);
		writeRegister(0x0010, 0x0003);
		delay(200);
	}
}
static unsigned int displayBuffer[176][220];

static void initBuffer() {
	for(int i=0;i<176;i++) {
		for(int j=0;j<220;j++) {
			displayBuffer[i][j]=0x0000;
		}
	}
}
void ili9225c_pushBuffer() {
	for(int i=0;i<220;i++) {
		for(int j=0;j<176;j++) {
			setWindow(j,i,j,i);
			writeData(displayBuffer[j][i]>>8);
			writeData(displayBuffer[j][i]&255);
		}
	}
}
unsigned int ili9225c_getBufferPixel(unsigned int x,unsigned int y) {
	return displayBuffer[x][y];
}
void ili9225c_modBuffer(unsigned char x,unsigned char y,unsigned char val) {
	displayBuffer[x][y]=val;
}
//Unbuffered draw
void ili9225c_drawPixelDirect(unsigned char x,unsigned char y,unsigned int color) {
	if(x>175) return;
	if(y>219) return;
	setWindow(x,y,x,y);
	writeData(color>>8);
	writeData(color&0xFF);
}
//Buffered draw
void ili9225c_drawPixel(unsigned char x,unsigned char y,unsigned int color) {
	if(x>175) return;
	if(y>219) return;
	displayBuffer[x][y]=color;
}
void ili9225c_line(int x1, int y1, int x2, int y2, unsigned int c) {
  // Classic Bresenham algorithm
  signed int steep = abs((signed int)(y2 - y1)) > abs((signed int)(x2 - x1));
  signed int dx, dy;
  if (steep) {
    swap(&x1, &y1);
    swap(&x2, &y2);
  }
  if (x1 > x2) {
    swap(&x1, &x2);
    swap(&y1, &y2);
  }
  dx = x2 - x1;
  dy = abs((signed int)(y2 - y1));
  signed int err = dx / 2;
  signed int ystep;
  if (y1 < y2) ystep = 1;
  else ystep = -1;
  for (; x1 <= x2; x1++) {
    if (steep) ili9225c_drawPixel(y1, x1, c);
    else       ili9225c_drawPixel(x1, y1, c);
    err -= dy;
    if (err < 0) {
      y1 += ystep;
      err += dx;
    }
  }
}
void ili9225c_fillRectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned int color) {
	for(int i=y1;i<y2;i++) {
		for(int j=x1;j<x2;j++) {
			ili9225c_drawPixel(j,i,color);
		}
	}
}
void ili9225c_fill(unsigned int color) {
	ili9225c_fillRectangle(0,0,maxX,maxY,color);
}
void ili9225c_clear() {
	ili9225c_fill(0x0000);
}
void ili9225c_writeImage(const char* data,int x,int y) {
	loadstr r = ili9225c_loadman(data);
	for(int j=0;j<r.height;++j) {
		for(int i=0;i<r.width;++i) {
			int basenum=(i+(j*r.width))*3;
			ili9225c_drawPixel(175-(i+x),j+y,(getColor(r.data[basenum],r.data[basenum+1],r.data[basenum+2])));
		}
	}
}
void ili9225c_writeImageBuffer(unsigned int** arr,int arrx,int arry,int x,int y) {
	for(int j=0;j<arrx;j++) {
		for(int i=0;i<arry;i++) {
			ili9225c_drawPixel(j+x,i+y,arr[j][i]);
		}
	}
}
void ili9225c_writeChar(char c, int x, int y) {
  char alphabet[] = " !\"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}";
  char* alphaArray = {strtok(strdup(alphabet), "")};

  int width = 5;
  int indexX = 0;
  int indexY = 0;

  for (unsigned int i = 0; i < strlen(alphabet); i++) {
    if (c == alphaArray[i]) {
      unsigned char pixel_buffer[5] = {symbols_8x5[i * 5], symbols_8x5[(i * 5) + 1], symbols_8x5[(i * 5) + 2], symbols_8x5[(i * 5) + 3], symbols_8x5[(i * 5) + 4]};
      for (int j = 0; j < width; j++) {
        for (unsigned char bit = 0x01; bit; bit <<= 1) {
          int xN = x - indexX;
          int yN = y + indexY;
          if (bit & pixel_buffer[j]) {
            ili9225c_drawPixel(xN, yN, THEME_FG);
          } else {
            ili9225c_drawPixel(xN, yN, THEME_BG);
          }
          indexY++;
          if (indexY > 7) {
            indexX++;
            indexY = 0;
          }
        }
      }
	  ili9225c_line(x-5,y,x-5,y+7,THEME_BG);
    }
  }
}
void ili9225c_writeString(const char* data, int x, int y, int maxl, int* l, int* h) {

  char* array = {strtok(strdup(data), "")};

  int indexX = 0;
  int indexY = 0;

  for (unsigned int i = 0; i < strlen(data); i++) {
    ili9225c_writeChar(array[i], (maxX-x-3) - indexX, y + indexY);
    indexX += 6;
	*l+=6;
    if (indexX > maxl-3) {
      indexX = 0;
	  *l=0;
      indexY += 8;
      *h += 8;
    }
  }
}
void ili9225c_init() {
    ili9225c_fd=wiringPiSPISetup(0,62500000);
	wiringPiSetupGpio();
	pinMode(LED,OUTPUT);
	pinMode(DC,OUTPUT);
	pinMode(RESET,OUTPUT);

	ili9225c_setBacklight(0);
	
	delay(50);
	setReset(1);
	delay(50);
	setReset(0);
	delay(50);
	setReset(1);
	delay(50);
	
	writeRegister(0x01, 0x031C); // set SS and NL bit
	writeRegister(0x02, 0x0100); // set 1 line inversion
	writeRegister(0x03, 0x1030); // set GRAM write direction and BGR=1.
	writeRegister(0x08, 0x0808); // set BP and FP
	writeRegister(0x0C, 0x0000); // RGB interface setting R0Ch=0x0110 for RGB 18Bit and R0Ch=0111for RGB16Bit
	writeRegister(0x0F, 0x0801); // Set frame rate
	writeRegister(0x20, 0x0000); // Set GRAM Address
	writeRegister(0x21, 0x0000); // Set GRAM Address
	//*************Power On sequence ****************//
	delay(50); // delay 50ms
	writeRegister(0x10, 0x0A00); // Set SAP,DSTB,STB
	writeRegister(0x11, 0x1038); // Set APON,PON,AON,VCI1EN,VC
	delay(50); // delay 50ms
	writeRegister(0x12, 0x1121); // Internal reference voltage= Vci;
	writeRegister(0x13, 0x0066); // Set GVDD
	writeRegister(0x14, 0x5F60); // Set VCOMH/VCOML voltage
	//------------------------ Set GRAM area --------------------------------//
	writeRegister(0x30, 0x0000);
	writeRegister(0x31, 0x00DB);
	writeRegister(0x32, 0x0000);
	writeRegister(0x33, 0x0000);
	writeRegister(0x34, 0x00DB);
	writeRegister(0x35, 0x0000);
	writeRegister(0x36, 0x00AF);
	writeRegister(0x37, 0x0000);
	writeRegister(0x38, 0x00DB);
	writeRegister(0x39, 0x0000);
	// ----------- Adjust the Gamma Curve ----------//
	writeRegister(0x50, 0x0400);
	writeRegister(0x51, 0x060B);
	writeRegister(0x52, 0x0C0A);
	writeRegister(0x53, 0x0105);
	writeRegister(0x54, 0x0A0C);
	writeRegister(0x55, 0x0B06);
	writeRegister(0x56, 0x0004);
	writeRegister(0x57, 0x0501);
	writeRegister(0x58, 0x0E00);
	writeRegister(0x59, 0x000E);
	delay(50); // delay 50ms
	writeRegister(0x07, 0x1017);
	
	ili9225c_setBacklight(1);
	
	ili9225c_clear();
    initBuffer();
    ili9225c_pushBuffer();
}
void ili9225c_reset() {
	digitalWrite(RESET,0);
	delay(50);
	digitalWrite(RESET,1);
	delay(50);
	ili9225c_init();
	delay(10);
}
void ili9225c_shutdown() {
	setDisplay(0);
	ili9225c_setBacklight(0);
}
void ili9225c_start() {
	setDisplay(1);
	ili9225c_setBacklight(1);
}