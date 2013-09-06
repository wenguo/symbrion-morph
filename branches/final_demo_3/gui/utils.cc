#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <algorithm>
#include <string.h>

#include "utils.hh"


uint8_t Avg(uint8_t * data, int size)
{
    assert(data!=NULL);
    assert(size>0);
    int sum = 0;
    for(int i=0;i<size;i++)
        sum += data[i];
    return sum/size;
}

uint8_t Min(uint8_t * data, int size)
{
    assert(data!=NULL);
    assert(size>0);
    uint8_t ret = 0xFF;
    for(int i=0;i<size;i++)
    {
        if(data[i] < ret)
            ret = data[i];
    }
    return ret;
}

uint8_t Max(uint8_t * data, int size)
{
    assert(data!=NULL);
    assert(size>0);
    uint8_t ret = 0;
    for(int i=0;i<size;i++)
    {
        if(data[i] > ret)
            ret = data[i];
    }
    return ret;
}

uint8_t* Quartile(uint8_t * data, int size)
{
    assert(data!=NULL);
    assert(size>0);
    //sort data
    std::vector<uint8_t> data_copy(data, data + size);
    sort(data_copy.begin(), data_copy.end());

    uint8_t * temp = new uint8_t[7];
    //calculate median
    int res = size % 2;
    int quotient = (int)(size/2);
    if(res == 1)
        temp[0] = data_copy[quotient];
    else
        temp[0] = (data_copy[quotient-1] + data_copy[quotient]) / 2;

    res = quotient % 2;
    quotient = (int)(quotient/2);
 //calculate low median and uppper median
    if( res == 1)
    {
        temp[1] = data_copy[quotient];
        temp[2] = data_copy[size - quotient - 1];
    }
    else
    {
        temp[1] = (data_copy[quotient-1] + data_copy[quotient]) / 2;
        temp[2] = (data_copy[size - quotient - 1] + data_copy[size - quotient]) / 2;
    }

    //fill min and max
    temp[3]=data_copy[0];
    temp[4]=data_copy[size-1];

    //fill the lower and upper limits
    temp[5] = temp[1] - 1.5 * (temp[2] - temp[1]);
    temp[6] = temp[2] + 1.5 * (temp[2] - temp[1]);

    printf("Quartile Result: %d\t%d\t%d\t%d\t%d\t%d\t%d\n",temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6]);

    return temp;

}

void flip(int width, int height, unsigned char *data)
{
    int size = width*height;
    uint8_t *src = new uint8_t[width*height*3];
    memcpy(src, data, size * 3);
    for(int i=0;i<size;i++)
    {
        data[i * 3]  = src[(size - i) * 3 - 3];
        data[i * 3 + 1]  = src[(size - i) * 3 -2];
        data[i * 3 + 2]  = src[(size - i) * 3 -1];
    }

}

void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
  int i;
  int y1, y2, u, v;
  int nr_pixels = width*height;


  for (i = 0; i < nr_pixels; i += 2)
  {
    /* Input format is Cb(i)Y(i)Cr(i)Y(i+1) */
#ifdef LAPTOP
    y1 = *src++;
    u = *src++;
    y2 = *src++;
    v = *src++;
#else
    u = *src++;
    y1 = *src++;
    v = *src++;
    y2 = *src++;
#endif
    y1 -= 16;
    u -= 128;
    v -= 128;
    y2 -= 16;

    *dst++ = clip(( 298 * y1           + 409 * v + 128) >> 8);
    *dst++ = clip(( 298 * y1 - 100 * u - 208 * v + 128) >> 8);
    *dst++ = clip(( 298 * y1 + 516 * u           + 128) >> 8);

    *dst++ = clip(( 298 * y2           + 409 * v + 128) >> 8);
    *dst++ = clip(( 298 * y2 - 100 * u - 208 * v + 128) >> 8);
    *dst++ = clip(( 298 * y2 + 516 * u           + 128) >> 8);
  }

}

void ppm16Write(char * ppmFilename, unsigned char* img, int width, int height)
{
    FILE *stream;

    stream = fopen(ppmFilename, "wb" );                     // Open the file for write

    fprintf( stream, "P6\n%d %d\n255\n", width, height );        // Write the file header information

    unsigned char *buf = img;

    char b[4];
    int c;
    for (c = 0 ; c < width*height; c++) 
    {
		b[0]= buf[0];//(buf[0]& 0xF8);	/* r */
		b[1]= buf[1];//((buf[0]<<5 | buf[1]>>3)&0xF8);	/* g */
		b[2]= buf[2];//((buf[1]<<2)&0xF8);	/* b */
		buf +=3;
		fwrite(&b[0], 1, 3, stream);
    }

    fclose( stream );

}

void PrintBuffer(uint8_t *data, int size)
{
    int i;
    printf(" -- buffer:");
    if(!data)
        printf("empty!");
    else
    {
        for(i=0; i<size; i++)
            printf("%#x\t", data[i]);
    }
    printf("\n");
}

