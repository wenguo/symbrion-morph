#ifndef VISION_DEF_H
#define VISION_DEF_H

#include <sys/time.h>

#define PORT_SERVER 10000
#define PORT_CLIENT_START 11000

#define RGB2YUV(r, g, b, y, u, v)\
	y = (306*r + 601*g + 117*b)  >> 10;\
u = ((-172*r - 340*g + 512*b) >> 10)  + 128;\
v = ((512*r - 429*g - 83*b) >> 10) + 128;\
y = y < 0 ? 0 : y;\
u = u < 0 ? 0 : u;\
v = v < 0 ? 0 : v;\
y = y > 255 ? 255 : y;\
u = u > 255 ? 255 : u;\
v = v > 255 ? 255 : v

#define HEADER_SIZE 1
#define CRC_SIZE 2
#define CHANNEL_INFO_SIZE 10 //[CHANNEL, RED, GREE, BLUE, YMIN, YMAX, UMIN, UMAX, VMIN, VMAX]
#define CHANNEL_BUFFER_SIZE HEADER_SIZE + 10 + CRC_SIZE //10 bytes to be  
#define BLOB_INFO_SIZE 4 //[x1,y1,x2,y2]
#define MAX_BLOBS_PER_CHANNEL 4
#define BLOB_BUFFER_SIZE 2 + MAX_BLOBS_PER_CHANNEL * BLOB_INFO_SIZE +CRC_SIZE // to be [channel, num_valid_blob, blob1, blob2 ... blob5]
#define NUM_BLOB_CHANNELS               3
#define MAX_OBJECTS_TRACKED             3

enum tracking_channel_t {BODY=0, LEDS, MAX_COLORS_TRACKED};

struct vect_2
{
    int16_t x;
    int16_t y;
};

struct blob_rect {
    vect_2 offset;
    vect_2 size;
    uint8_t id;
}__attribute__ ((__packed__));

struct blob_info_t{
	unsigned char channel;
	unsigned char num_blobs;
	blob_rect blobs[MAX_BLOBS_PER_CHANNEL];
        unsigned int timestamp;
	uint16_t crc;
}__attribute__ ((__packed__));

enum comm_status_t {
        REQ_SUBSCRIPTION = 0X10,
        REQ_SUBSCRIPTION_ACK = 0X11,
        REQ_UNSUBSCRIPTION = 0X12,
        REQ_UNSUBSCRIPTION_ACK = 0X13,
	REQ_CHANNEL_INFO = 0X14,
	REQ_CHANNEL_INFO_ACK = 0X15,
	SET_CHANNEL_INFO = 0X16,
	SET_CHANNEL_INFO_ACK = 0X17,
	REQ_IMAGE_FRAME = 0X18,
	REQ_IMAGE_FRAME_ACK = 0X19,
	REQ_BLOB_INFO = 0X1A,
	REQ_BLOB_INFO_ACK = 0X1B,
        REQ_ID = 0X1C,
        REQ_ID_ACK = 0X1D,
	UNKNOWN = 0XEF};

class RawImageFrame{
    public:
        enum ImageType{
            ImageTypeUnknown = 0,
            ImageTypeRawYUV  = 1,
        };

        class RawImageFileHdr
        { 
            public:
                uint16_t type;         // from ImageType enum
                uint8_t  reserved;     // reserved, default to zero
                uint8_t  field;        // which field of video this is from {0,1}
                uint16_t width,height; // image dimensions
                timeval  timestamp;
        };

        RawImageFileHdr hdr;
        uint8_t *data;
};




#endif
