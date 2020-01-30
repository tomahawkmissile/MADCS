#include "colors.h"

//Colors
int RGB(int r, int g, int b)
{ return r << 16 | g << 8 | b;
}
void splitColor(unsigned int rgb, unsigned char* red, unsigned char* green, unsigned char* blue) {
  *red = (rgb & 0b1111100000000000) >> 11 << 3;
  *green = (rgb & 0000011111100000) >> 5 << 2;
  *blue = (rgb & 0b0000000000011111) << 3;
}
unsigned int getColor(unsigned char r, unsigned char g, unsigned char b) {
  return (r >> 3) << 11 | (g >> 2) << 5 | (b >> 3);
}