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

#include "dss.h"

static int h=0;
//D0,D1,D2,D3,D4,D5,D6,D7,RST,CS,RS,WR,RD
static int pinout[13]={34,35,28,29,30,31,32,33,41,42,43,44,45};
//unsigned int altColor = 0xC7BF;
static unsigned int altColor = 0xFFFF;

void* dss_run_hx8357c(void* args) {
    hx8357c_setConsole(0,0,320,480);
    hx8357c_setRotation(2);
    memcpy(hx8357c_pins,pinout,sizeof(pinout));
    hx8357c_setPinStates();
    hx8357c_init();
    hx8357c_clear(0x0000);
    hx8357c_printConsole("Booting system...",altColor,THEME_BG);
    delay(1000);
    scaledImageData iData=scaleImage("resources/images/logo.jpg",320,480);
    hx8357c_fast_writeImageBuffer(iData.array,iData.arrx,iData.arry,0,0);
    FILE* fp;
    char path[1035];
    fp=popen("journalctl -fxe -n 50","r");

    if(fp==NULL) {
        hx8357c_printConsole("Boot failed. System failure.",altColor,THEME_BG);
    }
    while(fgets(path,sizeof(path),fp)!=NULL) {
        hx8357c_printConsole(path,altColor,THEME_BG);
    }
    pclose(fp);
    return NULL;
}

void* dss_run_oled(void* args) {
    oled_initOLED(0x3C);
    oled_writeString("Booting system...",0,0,127,&h);
    oled_displayBuffer();
    while(1) {
        mpu9250_data mpuData = MPU9250_readAllData();
        h=0;
        oled_clearBuffer();

        char axStr[32];
        char ayStr[32];
        char azStr[32];
        char gxStr[32];
        char gyStr[32];
        char gzStr[32];
        char mxStr[32];
        char myStr[32];
        char mzStr[32];
        char tmpStr[32];

        const char* format = "%s%.5f";
        sprintf(axStr,format,"A X ",mpuData.axx);
        sprintf(ayStr,format,"A Y ",mpuData.ayy);
        sprintf(azStr,format,"A Z ",mpuData.azz);
        sprintf(gxStr,format,"G X ",mpuData.gxx);
        sprintf(gyStr,format,"G Y ",mpuData.gyy);
        sprintf(gzStr,format,"G Z ",mpuData.gzz);
        sprintf(mxStr,format,"Roll ",mpuData.mxx);
        sprintf(myStr,format,"Pitch ",mpuData.myy);
        sprintf(mzStr,format,"Yaw ",mpuData.mzz);
        sprintf(tmpStr,"%s%i","T ",mpuData.tempDataA);

        oled_writeString(mxStr,0,0,127,&h);
        oled_writeString(myStr,0,8,127,&h);
        oled_writeString(mzStr,0,16,127,&h);

        oled_writeString(axStr,0,24,127,&h);
        oled_writeString(ayStr,0,32,127,&h);
        oled_writeString(azStr,0,40,127,&h);
        /*
        oled_writeString(gxStr,0,0,127,&h);
        oled_writeString(gyStr,0,0,127,&h);
        oled_writeString(gzStr,0,0,127,&h);
        */
        oled_writeString(tmpStr,0,56,127,&h);

        oled_displayBuffer();
    }
    return NULL;
}
typedef struct matrixPixel matrixPixel;
struct matrixPixel {
    unsigned int x;
    unsigned int y[25];
    unsigned int speed;
    unsigned char visible;
    unsigned int color;
};
void* dss_run_ili9225c(void* args) {
    mcp23017Setup(100,0x21);
    //int sdi,int clk,int dc,int rst,int led,int cs
    ili9225c_setPins(-1,-1,5,6,102,-1);
    ili9225c_init();
    ili9225c_fill(0x0000);
    ili9225c_pushBuffer();
    int h=0;
    //ili9225c_writeString(" !\"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}",0,0,170,&h);
	//ili9225c_pushBuffer();
    //scaledImageData iData=scaleImage("poog.jpg",176,220);
    //ili9225c_writeImageBuffer(iData.array,iData.arrx,iData.arry,0,0);
	//ili9225c_pushBuffer();
    //ili9225c_fill(0x0000);
    //ili9225c_pushBuffer();
    
    unsigned int colorGradient[20];
    unsigned int cgSize=sizeof(colorGradient)/sizeof(colorGradient[0]);
    for(int i=0;i<cgSize;i++) {
        double mult=220/(double)cgSize;
        int g=(int)(i*mult);
        colorGradient[cgSize-i-1]=getColor(0,g,0);
    }
    matrixPixel pan[350];
    while(1) {
        for(int i=0;i<sizeof(pan)/sizeof(pan[0]);i++) {
            unsigned int arrSize=sizeof(pan[i].y)/sizeof(pan[i].y[0]);
            unsigned int rN=rand()%10;
            if(pan[i].visible) {
                if(rN>7) {
                    unsigned int oldY=pan[i].y[0];
                    if(oldY<0) oldY=0;
                    //unsigned int oldPixel=ili9225c_getBufferPixel(pan[i].x,oldY);
                    /*
                    int exists=0;
                    for(unsigned int l=0;l<arrSize;l++) {
                        if(pan[i].x==pan[l].x) {
                            if(abs(pan[i].y[0]-pan[l].y[0])<cgSize-3) {
                                exists=1;
                            }
                        }
                    }
                    if((pan[i].y[0]>0)&&!exists) ili9225c_drawPixelDirect(pan[i].x,oldY,0x0000);
                    */
                    if(pan[i].y[0]>0) ili9225c_drawPixelDirect(pan[i].x,oldY,0x0000);
                    for(int k=0;k<cgSize;k++) {
                        pan[i].y[k]++;
                        ili9225c_drawPixelDirect(pan[i].x,pan[i].y[k],colorGradient[(cgSize-1)-k]);
                    }
                    if(pan[i].y[arrSize-1]>220+cgSize) {
                        pan[i].visible=0;
                        for(unsigned int del=0;del<cgSize;del++) {
                            ili9225c_drawPixelDirect(pan[i].x,pan[i].y[del],0x0000);
                        }
                    }
                }
            } else {
                pan[i].x=rand()%177;
                unsigned int yI=rand()%221;
                for(int j=0;j<cgSize;j++) {
                    pan[i].y[j]=yI+j;
                }
                pan[i].visible=1;
            }
            //delay(1);
        }
    }

    return NULL;
}

void dss_main() {
    pthread_t run_oled_p,run_hx8357c_p,run_ili9225c_p;
    pthread_create(&run_oled_p,NULL,dss_run_oled,NULL);
    pthread_create(&run_hx8357c_p,NULL,dss_run_hx8357c,NULL);
    //pthread_create(&run_ili9225c_p,NULL,dss_run_ili9225c,NULL);
    pthread_join(run_oled_p,NULL);
    pthread_join(run_hx8357c_p,NULL);
    //pthread_join(run_ili9225c_p,NULL);
}
