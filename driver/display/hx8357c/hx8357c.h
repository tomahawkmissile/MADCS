#define THEME_FG 0xFFFF
#define THEME_BG 0x0000

extern int hx8357c_pins[13];

void hx8357c_setPinStates(void);
static void write_bus(unsigned char c);
static void write_command(unsigned char c);
static void write_data(unsigned char c);
static void write_data_16(unsigned int c);
static void write_command_data(unsigned char com,unsigned char dat);
static void write_register(unsigned char com,unsigned int dat);
static void address_set(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void hx8357c_init(void);
void hx8357c_drawPixel(unsigned int x, unsigned int y, unsigned int c);
void hx8357c_line(int x1, int y1, int x2, int y2, unsigned int c);
void hx8357c_rect(int x1, int y1, int x2, int y2, unsigned int c);
void hx8357c_box(int x1, int y1, int x2, int y2, unsigned int c);
void hx8357c_clear(unsigned int j);
struct loadstr;
static struct loadstr hx8357c_loadman(const char* file);
void hx8357c_writeImage(const char* data,int x,int y);
void hx8357c_fast_writeImageBuffer(unsigned int** arr,int arrx,int arry,int x,int y);
void hx8357c_writeImageBuffer(unsigned int** arr,int arrx,int arry,int x,int y);
void hx8357c_writeChar(char c, int x, int y, unsigned int color, unsigned int bgColor);
void hx8357c_writeString(const char* data, int x, int y, unsigned int color, unsigned int bgColor, int maxl, int* h);
void hx8357c_clearConsole();
void hx8357c_setConsole(int cx,int cy,int cw,int ch);
void hx8357c_printConsole(const char data[], unsigned int color, unsigned int bgColor);
void hx8357c_setRotation(int r);
int hx8357c_getConsoleWidth();