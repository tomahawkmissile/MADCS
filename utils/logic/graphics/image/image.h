struct loadstr;
static struct loadstr loadman(const char* file);
typedef struct {
    unsigned int** array;
    int arrx,arry;
} scaledImageData;
scaledImageData scaleImage(const char* data,int newX,int newY);