#include <wiringPi.h>

#include <string.h>
#include <stdlib.h>

#include "../../../utils/colors/colors.h"
#include "../../../utils/logic/swap/swap.h"
#include "../../../utils/symbols.h"
#include "../../../libs/stb/stb_image.h"
#include "../../../utils/logic/graphics/image/image.h"

#define THEME_FG 0xFFFF
#define THEME_BG 0x0000

//D0,D1,D2,D3,D4,D5,D6,D7,RST,CS,RS,WR,RD
int hx8357c_pins[13];

int screen_height=320;
int screen_width=480;
//default rotation 4
char rotation=0;

void hx8357c_setPinStates(void) {
    for(int i=0;i<13;i++) {
        pinMode(hx8357c_pins[i],OUTPUT);
    }
}
static void write_bus(unsigned char c) {
    for(int i=0;i<8;i++) {
        digitalWrite(hx8357c_pins[i],0);
    }
    digitalWrite(hx8357c_pins[11],0);
    for(int i=0;i<8;i++) {
        digitalWrite(hx8357c_pins[i],(c&(1<<i)));
    }

    digitalWrite(hx8357c_pins[11],1);
}
static void write_command(unsigned char c) {
    digitalWrite(hx8357c_pins[10],0);
    write_bus(c);
}
static void write_data(unsigned char c) {
    digitalWrite(hx8357c_pins[10],1);
    write_bus(c);
}
static void write_data_16(unsigned int c) {
    digitalWrite(hx8357c_pins[10],1);
    write_bus(c>>8);
    write_bus(c);
}
static void write_command_data(unsigned char com,unsigned char dat) {
    write_command(com);
    write_data(dat);
}
static void write_register(unsigned char com,unsigned int dat) {
    write_command(com);
    write_data_16(dat);
}
static void address_set(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
    write_command(0x2a);
    write_data(x1 >> 8);
    write_data(x1);
    write_data(x2 >> 8);
    write_data(x2);

    write_command(0x2b);
    write_data(y1 >> 8);
    write_data(y1);
    write_data(y2 >> 8);
    write_data(y2);
    write_command(0x2c);
}
void hx8357c_init(void) {

  digitalWrite(hx8357c_pins[8],0);
  delay(150);
  digitalWrite(hx8357c_pins[8],1);
  delay(150); 

  for(int i=8;i<13;i++) {
      digitalWrite(hx8357c_pins[i],1);
  }

  digitalWrite(hx8357c_pins[8], HIGH);
  delay(50);
  digitalWrite(hx8357c_pins[8], LOW);
  delay(150);
  digitalWrite(hx8357c_pins[8], HIGH);
  delay(150);

  digitalWrite(hx8357c_pins[9], HIGH);
  digitalWrite(hx8357c_pins[11], HIGH);
  digitalWrite(hx8357c_pins[9], LOW); //CS

  write_command(0XF9);
  write_data(0x00);
  write_data(0x08);

  write_command(0xC0);
  write_data(0x19);//VREG1OUT POSITIVE
  write_data(0x1a);//VREG2OUT NEGATIVE

  write_command(0xC1);
  write_data(0x45);//VGH,VGL    VGH>=14V.
  write_data(0x00);

  write_command(0xC2);
  write_data(0x33);

  write_command(0XC5);
  write_data(0x00);
  write_data(0x28);//VCM_REG[7:0]. <=0X80.

  write_command(0xB1);//OSC Freq set.
  write_data(0x90);//0xA0=62HZ,0XB0 =70HZ, <=0XB0.
  write_data(0x11);

  write_command(0xB4);
  write_data(0x02); //2 DOT FRAME MODE,F<=70HZ.

  write_command(0xB6);
  write_data(0x00);
  write_data(0x42);//0 GS SS SM ISC[3:0];
  write_data(0x3B);

  write_command(0xB7);
  write_data(0x07);

  write_command(0xE0);
  write_data(0x1F);
  write_data(0x25);
  write_data(0x22);
  write_data(0x0B);
  write_data(0x06);
  write_data(0x0A);
  write_data(0x4E);
  write_data(0xC6);
  write_data(0x39);
  write_data(0x00);
  write_data(0x00);
  write_data(0x00);
  write_data(0x00);
  write_data(0x00);
  write_data(0x00);

  write_command(0XE1);
  write_data(0x1F);
  write_data(0x3F);
  write_data(0x3F);
  write_data(0x0F);
  write_data(0x1F);
  write_data(0x0F);
  write_data(0x46);
  write_data(0x49);
  write_data(0x31);
  write_data(0x05);
  write_data(0x09);
  write_data(0x03);
  write_data(0x1C);
  write_data(0x1A);
  write_data(0x00);

  write_command(0XF1);
  write_data(0x36);
  write_data(0x04);
  write_data(0x00);
  write_data(0x3C);
  write_data(0x0F);
  write_data(0x0F);
  write_data(0xA4);
  write_data(0x02);

  write_command(0XF2);
  write_data(0x18);
  write_data(0xA3);
  write_data(0x12);
  write_data(0x02);
  write_data(0x32);
  write_data(0x12);
  write_data(0xFF);
  write_data(0x32);
  write_data(0x00);

  write_command(0XF4);
  write_data(0x40);
  write_data(0x00);
  write_data(0x08);
  write_data(0x91);
  write_data(0x04);

  write_command(0XF8);
  write_data(0x21);
  write_data(0x04);

  write_command(0x36);
  write_data(0xC8);

  write_command(0x3A);
  write_data(0x55);

  write_command(0x11);
  delay(120);
  //write_command(0x21);
  write_command(0x29);
}
//Shapes
void hx8357c_drawPixel(unsigned int x, unsigned int y, unsigned int c) {
  if(rotation==2) {
    x=screen_height-x;
    y=screen_width-y;
  } else if(rotation==1) {
    x=screen_width-x;
    swap(&x,&y);
  } else if(rotation==3) {
    y=screen_height-y;
    swap(&x,&y);
  }
  write_command(0x02c); //write_memory_start
  digitalWrite(hx8357c_pins[10], HIGH);
  digitalWrite(hx8357c_pins[9], LOW);
  address_set(x, y, x, y);
  write_data(c >> 8);
  write_data(c);
  digitalWrite(hx8357c_pins[9], HIGH);
}
void hx8357c_line(int x1, int y1, int x2, int y2, unsigned int c) {
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
    if (steep) hx8357c_drawPixel(y1, x1, c);
    else       hx8357c_drawPixel(x1, y1, c);
    err -= dy;
    if (err < 0) {
      y1 += ystep;
      err += dx;
    }
  }
}
void hx8357c_rect(int x1, int y1, int x2, int y2, unsigned int c) {
  write_command(0x02c);
  digitalWrite(hx8357c_pins[10], HIGH);
  digitalWrite(hx8357c_pins[9], LOW);
  address_set(x1, y1, x2, y2);
  for (int i = y1; i <= y2; i++) {
    for (int j = x1; j <= x2; j++) {
      write_data(c >> 8);
      write_data(c);
    }
  }
  digitalWrite(hx8357c_pins[9], HIGH);
}
void hx8357c_box(int x1, int y1, int x2, int y2, unsigned int c) {
  hx8357c_line(x1, y1, x2, y1, c);
  hx8357c_line(x1, y1, x1, y2, c);
  hx8357c_line(x2, y2, x1, y2, c);
  hx8357c_line(x2, y2, x2, y1, c);
}

void hx8357c_clear(unsigned int j) {
  unsigned int i, m;
  digitalWrite(hx8357c_pins[9], LOW);
  address_set(0, 0, 320 - 1, 480 - 1);
  //write_command(0x02c); //write_memory_start
  //digitalWrite(hx8357c_pins[10],HIGH);

  for (i = 0; i < 480; i++)
    for (m = 0; m < 320; m++)
    {
      write_data(j >> 8);
      write_data(j);

    }
  digitalWrite(hx8357c_pins[9], HIGH);
}

typedef struct loadstr loadstr;
struct loadstr {
  int width;
  int height;
  unsigned char* data;
};
static struct loadstr hx8357c_loadman(const char* file) {
	int width,height,bpp;
	unsigned char* pixels=stbi_load(file,&width,&height,&bpp,STBI_rgb);
	return (loadstr){ .width=width,.height=height,.data=pixels };
}
void hx8357c_writeImage(const char* data,int x,int y) {
	loadstr r = hx8357c_loadman(data);
	for(int j=0;j<r.height;++j) {
		for(int i=0;i<r.width;++i) {
			int basenum=(i+(j*r.width))*3;
		  hx8357c_drawPixel(i+x,j+y,(getColor(r.data[basenum],r.data[basenum+1],r.data[basenum+2])));
		}
	}
}
void hx8357c_fast_writeImageBuffer(unsigned int** arr,int arrx,int arry,int x,int y) {
  write_command(0x02c);
  digitalWrite(hx8357c_pins[10], HIGH);
  digitalWrite(hx8357c_pins[9], LOW);
  if(arrx>320) arrx=320;
  if(arry>480) arry=480;
  address_set(x, y, x+arrx, y+arry);
  for (int j = arry-1; j >= 0; j--) {
    for (int i = arrx-1; i >= 0 ; i--) {
      write_data(arr[i][j] >> 8);
      write_data(arr[i][j]);
    }
  }
  digitalWrite(hx8357c_pins[9], HIGH);
}
void hx8357c_writeImageBuffer(unsigned int** arr,int arrx,int arry,int x,int y) {
	for(int j=0;j<arry;j++) {
		for(int i=0;i<arrx;i++) {
			hx8357c_drawPixel(i+x,j+y,arr[i][j]);
		}
	}
}

void hx8357c_writeChar(char c, int x, int y, unsigned int color, unsigned int bgColor) {
  char alphabet[] = " !\"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}";
  char* alphaArray = {strtok(strdup(alphabet), "")};

  int width = 5;
  int indexX = 0;
  int indexY = 0;

  int xN=0;
  int yN=0;

  for (unsigned int i = 0; i < strlen(alphabet); i++) {
    if (c == alphaArray[i]) {
      unsigned char buffer[5] = {symbols_8x5[i * 5], symbols_8x5[(i * 5) + 1], symbols_8x5[(i * 5) + 2], symbols_8x5[(i * 5) + 3], symbols_8x5[(i * 5) + 4]};
      for (int j = 0; j < width; j++) {
        for (unsigned char bit = 0x01; bit; bit <<= 1) {
          xN = x + indexX;
          yN = y + indexY;
          if (bit & buffer[j]) {
            hx8357c_drawPixel(xN, yN, color);
          } else {
            hx8357c_drawPixel(xN, yN, bgColor);
          }
          indexY++;
          if (indexY > 7) {
            indexX++;
            indexY = 0;
          }
        }
      }
      hx8357c_line(xN+1,yN-7,xN+1,yN,bgColor);
    }
  }
}
void hx8357c_writeString(const char* data, int x, int y, unsigned int color, unsigned int bgColor, int maxl, int* l, int* h) {

  char* array = {strtok(strdup(data), "")};

  int indexX = 0;
  int indexY = 0;

  for (unsigned int i = 0; i < strlen(data); i++) {
    hx8357c_writeChar(array[i], x + indexX, y + indexY, color, bgColor);
    indexX += 6;
    *l+=6;
    if (indexX > maxl-5) {
      indexX = 0;
      *l=0;
      indexY += 8;
      *h += 8;
    }
  }
}
int yi = 0;
int consoleX,consoleY,consoleWidth,consoleHeight;
scaledImageData iData;

void hx8357c_clearConsole() {
  hx8357c_rect(consoleX, consoleY, consoleWidth, consoleHeight, 0x0000);
}
void hx8357c_setConsole(int cx,int cy,int cw,int ch) {
  consoleX=cx;
  consoleY=cy;
  consoleWidth=cw;
  consoleHeight=ch;
  iData=scaleImage("resources/images/logo.jpg",320,480);
}
void hx8357c_printConsole(const char data[], unsigned int color, unsigned int bgColor) {
  int h=0,l=0;
  hx8357c_writeString(data, consoleX+1, (consoleY+1) + yi, color, bgColor, consoleWidth, &l, &h);
  yi += h + 8;
  if (yi > consoleHeight-8) {
    //hx8357c_clearConsole();
    hx8357c_fast_writeImageBuffer(iData.array,iData.arrx,iData.arry,0,0);
    yi=0;
  }
}
void hx8357c_setRotation(int r) {
  if(r>4) r=4;
  if(r<1) r=1;
  rotation=r;
}
int hx8357c_getConsoleWidth() {
  return consoleWidth;
}
