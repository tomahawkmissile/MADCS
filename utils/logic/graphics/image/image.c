#include <stdio.h>
#include <stdlib.h>

#include "../../../../libs/stb/stb_image.h"
#include "../../../colors/colors.h"

typedef struct loadstr loadstr;
struct loadstr {
  int width;
  int height;
  unsigned char* data;
};
static struct loadstr loadman(const char* file) {
	int width,height,bpp;
	unsigned char* pixels=stbi_load(file,&width,&height,&bpp,STBI_rgb);
	return (loadstr){ .width=width,.height=height,.data=pixels };
}
typedef struct {
    unsigned int** array;
    int arrx,arry;
} scaledImageData;
scaledImageData scaleImage(const char* data,int newX,int newY) {
    newX++;
    newY++;
	loadstr r = loadman(data);
    int x=r.width;
    int y=r.height;
    unsigned int unscaled_data[x][y];
	for(int j=0;j<y;++j) {
		for(int i=0;i<x;++i) {
			int basenum=(i+(j*r.width))*3;
			unscaled_data[i][j]=(getColor(r.data[basenum],r.data[basenum+1],r.data[basenum+2]));
		}
	}
    unsigned int** scaled_data;
    scaled_data=(unsigned int**)malloc(x*sizeof(unsigned int*));
    for(int k=0;k<newX;k++) {
        scaled_data[k]=(unsigned int*)malloc(newY*sizeof(unsigned int));
    }
    double xM=r.width/(double)newX;
    double yM=r.height/(double)newY;
    for(int j=0;j<newX;j++) {
        for(int i=0;i<newY;i++) {
            scaled_data[j][i]=unscaled_data[(int)(j*xM)][(int)(i*yM)];
        }
    }
    return (scaledImageData){.array=scaled_data,.arrx=newX,.arry=newY};
}