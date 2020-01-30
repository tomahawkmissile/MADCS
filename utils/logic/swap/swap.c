#include "swap.h"

void swap(int* a,int* b) {
    int w=*a;
    *a=*b;
    *b=w;
}