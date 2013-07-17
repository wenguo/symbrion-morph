#ifndef IMAGEBOX_HH
#define IMAGEBOX_HH
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string>
#include <list>

#include "../src/base/vision_def.h"
#include "utils.hh"
#include "ethlolmsg.h"
#include "ipc.hh"

using namespace std;

enum slider_index{RED=0, GREEN, BLUE, YMIN, UMIN, VMIN, YMAX, UMAX, VMAX};

class Canvas;
class iDisplay;

struct rgb{
    unsigned char red,green,blue;
};
struct color_info{
    rgb color;          // example color (such as used in test output)
    char *name;         // color's meaninful name (e.g. ball, goal)
    double merge;       // merge density threshold
    int expected_num;   // expected number of regions (used for merge)
    int min_area;
    int max_area;
    int y_low,y_high;   // Y,U,V component thresholds
    int u_low,u_high;
    int v_low,v_high;
};

class ImageBox
{
    public:
        ImageBox(int port, int width=320, int height=240, iDisplay*parent=NULL);
        ~ImageBox();


        bool Connected();
        bool RequestInfo(comm_status_t req_type);
        bool SetChannelInfo(color_info ch_info, int index);

        void SetREQType(comm_status_t req_type);
        void SetParent(iDisplay* parent) {display = parent;};
        void Snapshot(unsigned int ts);
        static void * Capturing(void *ptr);

        bool WriteFrame(const RawImageFrame * frame, FILE * fd);
        bool InitCapturing(const char * folder);


        //    private:
        iDisplay* display;
        int port;

        pthread_t capture_thread;
        pthread_mutex_t mutex;
        pthread_cond_t image_ready_cond;
        bool image_ready;

        list<blob_info_t*> blobs;
        blob_info_t blob_info[MAX_COLORS_TRACKED];

        static int index;
        int id;

        bool connected;
        char name[64];
        char logfolder[64];
        FILE * logfile;

        int img_width;
        int img_height;
        unsigned char * img_yuv_buffer;
        RawImageFrame frame;
        unsigned char *img_rgb_buffer;

        unsigned int timestamp;
        bool userQuit;

        int channel_index;
        color_info channel_info;

        comm_status_t comm_rx_status;
        comm_status_t comm_tx_status;

        static bool flipped_image;
        bool capturing;

        IPC::IPC monitoringIPC;
        static void Monitoring(const ELolMessage *msg, void*connection, void *ptr);
};

class iDisplay
{
    public:
        iDisplay(Canvas *parent, int x, int y, int w=320, int h=240);
        bool SetImageBox(ImageBox *box);
        bool InRegion(int x, int y);//check if mouse clicked 

        static int index;
        int id;

        int width;
        int height;

        double scale;

        int region_x1; //selection region
        int region_y1;
        int region_x2;
        int region_y2;

        int clicked;

        int startx;
        int starty;
        Canvas * canvas;
        ImageBox *imagebox;
};

#endif
