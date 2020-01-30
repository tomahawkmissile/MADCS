#include "stringUtils.h"

#include <stdio.h>

const char* str_concat(const char* a,const char* b) {
    int sizeA=0;
    int sizeB=0;
    while(a[sizeA]!='\0') sizeA++;
    while(b[sizeB]!='\0') sizeB++;
    const char* newStr[sizeA+sizeB];
    for(int i=0;i<(sizeA+sizeB);i++) {
        newStr[i]=(i<sizeA?&a[i]:(&b[i-sizeA]));
    }
    return *newStr;
}