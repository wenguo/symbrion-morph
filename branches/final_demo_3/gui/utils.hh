#ifndef UTILS_HH
#define UTILS_HH


typedef unsigned char uint8_t;

uint8_t* Quartile(uint8_t * data, int size);
uint8_t Avg(uint8_t * data, int size);
uint8_t Min(uint8_t * data, int size);
uint8_t Max(uint8_t * data, int size);

#define clip(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

void ppm16Write(char * ppmFilename, unsigned char* img, int width, int height);
void flip(int width, int height, unsigned char * data);
void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst);

void PrintBuffer(uint8_t *data, int size);

#endif
