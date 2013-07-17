#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <signal.h>

#include "capture.h"
#include "cmvision.h"
#include "VisualMemory.hh"
#include "../src/base/vision_def.h"

#include "ipc.hh"
#include "hist.hh"

#ifdef LAPTOP
#define PIXEL_FORMAT V4L2_PIX_FMT_YUYV
const char *video_device = "/dev/video0";
#else
#define PIXEL_FORMAT V4L2_PIX_FMT_UYVY
const char *video_device = "/dev/video1";
#endif

uint32_t currentTime = 0;
uint32_t lastupdateTime = 0;
int userQuit = 0;
void signalHandler(int dummy);
void timerHandler(int dymmy);

class Robot
{
    public:
        Robot(){};
        ~Robot(){};

    pthread_mutex_t vision_mutex;
    pthread_cond_t vision_cond;
    pthread_t vision_thread;
    Capture cap;
    CMVision::CMVision vision;
    const Capture::Image *img;
    RawImageFrame frame;
    int img_width;
    int img_height;
    Blob_info blob_data[MAX_OBJECTS_TRACKED];
    Hist blob_hist;

    VisualMemory *vm;

    IPC::IPC monitoringIPC;
    IPC::IPC subscriptionIPC;
    char * monitoring_host;
    static void Monitoring(const ELolMessage*msg, void* connection, void *user_ptr);
    static void Subscribe(const ELolMessage*msg, void* connection, void *user_ptr);
    static void *BlobDetection(void * ptr);
    bool InitVision();
    void addBlob(int x1, int y1, int x2, int y2, unsigned char * img, int width, int height, CMVision::rgb color);

    void Update(uint32_t ts);

    bool subscribed;

    uint32_t timestamp;
};


int main(int argc, char *argv[])
{
    //set signal handler to capture "ctrl+c" event
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        printf("signal(2) failed while setting up for SIGINT");
        return -1;
    }

    if (signal(SIGTERM, signalHandler) == SIG_ERR)
    {
        printf("signal(2) failed while setting up for SIGTERM");
        return -1;
    }

    if (signal(SIGALRM, timerHandler)==SIG_ERR)
    {
        printf("signal(2) failed while setting up for SIGALRM");
        return -1;
    }

    Robot robot;
    robot.InitVision();
    //set timer to be every 100 ms
    struct itimerval tick;
    memset(&tick, 0, sizeof(tick));
    tick.it_value.tv_sec = 0;
    tick.it_value.tv_usec = 100000;
    tick.it_interval.tv_sec = 0;
    tick.it_interval.tv_usec = 100000;

    //set timer
    if (setitimer(ITIMER_REAL, &tick, NULL))
    {
        printf("Set timer failed!!\n");
    }


    //main loop
    while (userQuit != 1)
    {
        //block until timer event occurs
        while(currentTime==lastupdateTime)
        {
            usleep(5000);
        }
        lastupdateTime = currentTime;
        //robot update
        robot.Update(currentTime);
    }
    printf("clean up\n");

    return 0;

}
void signalHandler(int dummy)
{
    printf("Ctrl+C captured, exit program!\n");
    userQuit = 1;
}
void timerHandler(int dummy)
{
    currentTime++;
}

bool Robot::InitVision()
{
    const char *video_device = "/dev/video0";
    const char *color_filename = "colors.txt";
    monitoring_host = strdup("localhost");

    //#define PIXEL_FORMAT V4L2_PIX_FMT_UYVY

    img_width=320;
    img_height=240;

    frame.hdr.type = RawImageFrame::ImageTypeRawYUV;
    frame.hdr.width = img_width;
    frame.hdr.height = img_height;


    if(!cap.init(video_device,0,img_width,img_height,PIXEL_FORMAT))
    {
        printf("no success to load the camera driver, quit the program\n");
        return false;
    }


    if(!vision.initialize(img_width, img_height))
    {
        printf("Error initializing vision");
        return false;
    }
    if(!vision.loadOptions(color_filename))
    {
        printf("Error loading color file");
        return false;
    }

    vm = new VisualMemory(img_width, img_height);
    subscribed = false;

    pthread_mutex_init(&vision_mutex, NULL);
    pthread_cond_init(&vision_cond, NULL);

    pthread_create(&vision_thread, NULL, BlobDetection, this);

    subscriptionIPC.SetCallback(Subscribe, this);
    subscriptionIPC.Start(monitoring_host, PORT_SERVER, false);
    subscriptionIPC.Name("subscription");
    //delay 2 seconds
    usleep(2000000);
    char hostname[64];
    gethostname(hostname,64);
    subscriptionIPC.SendData(REQ_SUBSCRIPTION, (uint8_t*)hostname, strlen(hostname));

}

void Robot::Subscribe(const ELolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection ||!user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    int monitoring_port = 0;
    switch(msg->command)
    {
        case REQ_SUBSCRIPTION_ACK:
            {
                monitoring_port = msg->data[1];
                monitoring_port = monitoring_port << 8 | msg->data[0];
                robot->monitoringIPC.SetCallback(Monitoring, (void*)robot);
                robot->monitoringIPC.Name("Monitoring");
                robot->monitoringIPC.Start(robot->monitoring_host, monitoring_port, false);
                char hostname[64];
                gethostname(hostname,64);
                usleep(1000000);
                robot->monitoringIPC.SendData(REQ_ID_ACK, (uint8_t*)hostname, strlen(hostname));
                robot->subscribed = true;
            }     
        default:
            break;
    }
}

void Robot::Monitoring(const ELolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !user_ptr || !connection)
        return;

    Robot *robot = (Robot*)user_ptr;

    int image_size = robot->img_width * robot->img_height * 2;
    int header_size =sizeof(RawImageFrame::RawImageFileHdr);
    unsigned char *tx_buffer=new unsigned char[image_size + header_size];

    switch(msg->command)
    {
        case REQ_IMAGE_FRAME:
            {
                printf("recieved request for new image %d\n", image_size + header_size);
                pthread_mutex_lock(&robot->vision_mutex);
                memcpy(tx_buffer, (unsigned char*)&robot->frame.hdr, header_size);
                memcpy(tx_buffer + header_size, (unsigned char*)robot->img->data, image_size);				
                pthread_mutex_unlock(&robot->vision_mutex);


                for(int ch=0; ch<MAX_COLORS_TRACKED; ch++)
                {
                    pthread_mutex_lock(&robot->vision_mutex);
                    CMVision::rgb c = robot->vision.getColorVisual(ch);
                    CMVision::CMVision::region* reg = robot->vision.getRegions(ch);
                    pthread_mutex_unlock(&robot->vision_mutex);

                    while(reg)
                    {
                        robot->addBlob(reg->x1, reg->y1, reg->x2, reg->y2, tx_buffer + header_size, robot->img_width, robot->img_height, c);
                        reg = reg->next;
                    }

                }

                printf("send new image\n");
                robot->monitoringIPC.SendData(REQ_IMAGE_FRAME_ACK, tx_buffer, image_size + header_size);
            }
            break;
        case REQ_CHANNEL_INFO:
            {
                printf("recieved request for channel_info\n");
                int ymin,ymax,umin,umax,vmin,vmax;
                CMVision::rgb color=robot->vision.getColorVisual(msg->data[0]);
                robot->vision.getThreshold(msg->data[0], ymin,ymax,umin,umax,vmin,vmax);
                tx_buffer[0] = msg->data[0];
                tx_buffer[1] = color.red;
                tx_buffer[2] = color.green;
                tx_buffer[3] = color.blue;
                tx_buffer[4] = ymin;
                tx_buffer[5] = ymax;
                tx_buffer[6] = umin;
                tx_buffer[7] = umax;
                tx_buffer[8] = vmin;
                tx_buffer[9] = vmax;


                robot->monitoringIPC.SendData(REQ_CHANNEL_INFO_ACK, tx_buffer, 10);

            }
            break;

        case SET_CHANNEL_INFO:
            {
                printf("Received request to set channel %d info\n", msg->data[0]);
                CMVision::CMVision::color_info *colorinfo = robot->vision.getColorInfo(msg->data[0]);
                colorinfo->color.red = msg->data[1];
                colorinfo->color.green  = msg->data[2];
                colorinfo->color.blue  = msg->data[3];

                int ymin,ymax,umin,umax,vmin,vmax;
                ymin = msg->data[4];
                ymax = msg->data[5];
                umin = msg->data[6];
                umax = msg->data[7];
                vmin = msg->data[8];
                vmax = msg->data[9];

                robot->vision.setThreshold(msg->data[0], ymin,ymax,umin,umax, vmin,vmax);

                tx_buffer[0] = 0xAA;
                robot->monitoringIPC.SendData(SET_CHANNEL_INFO_ACK, tx_buffer, 1);


                printf("red: %d\ngreen: %d\nblue: %d\nymin: %d\nymax: %d\numin: %d\numax: %d\nvmin: %d\nvmax: %d\n", 
                        msg->data[1], msg->data[2], msg->data[3],
                        ymin,ymax,umin,umax,vmin,vmax);
            }
            break;
        case REQ_BLOB_INFO:
            {
                printf("recieved request for blob_info\n");
                blob_info_t blob_info[MAX_COLORS_TRACKED];
                memset(&blob_info, 0, MAX_COLORS_TRACKED*sizeof(blob_info_t));

                std::list<Object*>::iterator it;
                if(robot->vm)
                {
                    std::list<Object*> objects = robot->vm->Objects();
                    int num_blobs[MAX_COLORS_TRACKED] = {0};
                    for(it = objects.begin();it!=objects.end();it++)
                    {
                        Object * obj = *it;
                        if(obj)
                        {
                            for(int ch=0;ch<MAX_COLORS_TRACKED;ch++)
                            {
                                Blob_info blob = obj->GetLatestAveragedBlobInfo(robot->timestamp, ch);
                                blob_info[ch].channel = ch;
                                if(blob.sys_timestamp!=-1)
                                {
                                    if(num_blobs[ch] >= MAX_BLOBS_PER_CHANNEL)
                                        break;
                                    blob_info[ch].blobs[num_blobs[ch]].offset.x = blob.offset.x;
                                    blob_info[ch].blobs[num_blobs[ch]].offset.y = blob.offset.y;
                                    blob_info[ch].blobs[num_blobs[ch]].size.x = blob.size.x;
                                    blob_info[ch].blobs[num_blobs[ch]].size.y = blob.size.y;
                                    blob_info[ch].blobs[num_blobs[ch]].id = obj->id;
                                    num_blobs[ch]++;
                                }
                            }
                        }
                    }
                    for(int ch=0;ch<MAX_COLORS_TRACKED;ch++)
                        blob_info[ch].num_blobs=num_blobs[ch];

                }
                //memcpy(tx_buffer, blob_info, MAX_COLORS_TRACKED * sizeof(blob_info_t));
                robot->monitoringIPC.SendData(REQ_BLOB_INFO_ACK, (uint8_t*)blob_info, MAX_COLORS_TRACKED * sizeof(blob_info_t));
            }
            break;
        case REQ_ID:
            {
                printf("received request for ID\n");
                char hostname[64];
                gethostname(hostname,64);
                int size=strlen(hostname);
                robot->monitoringIPC.SendData(REQ_ID_ACK, (uint8_t*)hostname, size);
            }
            break;
        case UNKNOWN:
        default:
            break;

    }


}

void *Robot::BlobDetection(void * ptr)
{
    printf("Blob detection thread is running\n");

    Robot *robot = (Robot*)ptr;
    static int count = 0;

    timeval starttime, sys_time;
    gettimeofday(&starttime, NULL);

    while(1)
    {
        //#if !defined(LAPTOP) 
        count++;
        pthread_mutex_lock(&robot->vision_mutex);
        robot->img = robot->cap.captureFrame();
        if(robot->img !=NULL)
        {
            robot->frame.hdr.timestamp = robot->img->timestamp;
            robot->frame.data = robot->img->data;
            robot->vision.processFrame(reinterpret_cast<CMVision::image_pixel*>(robot->img->data));
            /*if(robot->timestamp %10 ==0)
              {
              printf("processed frame @ %d fps\n", count);
              count=0;
              }*/
        }
        robot->cap.releaseFrame(robot->img);

        gettimeofday(&sys_time, NULL);

        for(int ch=0;ch<MAX_COLORS_TRACKED;ch++)
        {
            CMVision::rgb c = robot->vision.getColorVisual(ch);
            CMVision::CMVision::region* reg = robot->vision.getRegions(ch);
            int index=0;

            while(reg)
            {
                robot->vm->AddTrackingData(reg->x1, reg->y1, reg->x2, reg->y2, robot->timestamp, (sys_time.tv_sec - starttime.tv_sec)*1000 + sys_time.tv_usec/1000 , ch);
                reg = reg->next;
                index++;
                if(index >= MAX_OBJECTS_TRACKED) 
                    break;
            }
        }

        pthread_mutex_unlock(&robot->vision_mutex);
        //#endif
    }

    printf("Blob detection thread is exiting\n");
    return NULL;
}

void Robot::addBlob(int x1, int y1, int x2, int y2, unsigned char * img, int width, int height, CMVision::rgb color)
{
    int y,u,v;
    if(x1<0)
        x1=0;
    if(x2<0)
        x2=0;
    if(y1<0)
        y1=0;
    if(y2<0)
        y2=0;
    if(x1>=width)
        x1=width-1;
    if(x2>=width)
        x2=width-1;
    if(y1>=height)
        y1=height-1;
    if(y2>=height)
        y2=height-1;

    RGB2YUV(color.red, color.green, color.blue, y,u,v);

    for (int i=x1; i<=x2;i++)
    {
        img[(y1*width  + i )*2] = u;
        img[(y1*width  + i)*2 + 1] = y;
        img[(y1*width  + i)*2 + 2] = v;
        img[(y1*width  + i)*2 + 3] = y;
    }
    for (int i=x1; i<=x2;i++)
    {
        img[(y2*width  + i )*2] = u;
        img[(y2*width  + i)*2 + 1] = y;
        img[(y2*width  + i)*2 + 2] = v;
        img[(y2*width  + i)*2 + 3] = y;
    }

    for (int i=y1; i<=y2;i++)
    {
        img[(i*width  + x1)*2] = u;
        img[(i*width  + x1)*2 + 1] = y;
        img[(i*width  + x1)*2 + 2] = v;
        img[(i*width  + x1)*2 + 3] = y;
    }
    for (int i=y1; i<=y2;i++)
    {
        img[(i*width  + x2)*2] = u;
        img[(i*width  + x2)*2 + 1] = y;
        img[(i*width  + x2)*2 + 2] = v;
        img[(i*width  + x2)*2 + 3] = y;
    }
}

void Robot::Update(uint32_t ts)
{
    timestamp = ts;
}
