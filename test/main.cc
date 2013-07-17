#include "IRobot.h"
#include "comm/IRComm.h"
#include "comm/Ethernet.h"
#include "ipc.hh"
#include "ethlolmsg.h"
#include "ipc_interface.hh"
#include <pthread.h>
#include <signal.h>
#include <cstdio>
#include <string.h>
#include <sys/time.h>


#include "capture.h"
#include "cmvision.h"
#include "global.h"

Capture cap;
CMVision::CMVision vision;
const Capture::Image *img;
RawImageFrame frame;
int img_width;
int img_height;
void * BlobDetection(void * ptr);
#define PIXEL_FORMAT V4L2_PIX_FMT_UYVY
const char *video_device = "/dev/video0";
const char *color_filename = "colors.txt";
pthread_t vision_thread;

bool initVision();

void process_message(const ELolMessage*msg, void* connection, void *user_ptr);
void update(int64_t timestamp);
void testlolmsg();

bool processing_done = false;

bool userQuit = false;
int64_t currentTime=0;
int64_t lastupdateTime=0;
//define the IPC communication socket
IPC::IPC master_IPC;
Ethernet::IP neighbours_IP[4];

void signalHandler(int); 
void timerHandler(int); 

int main(int argc, char** args) {

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

     //set signal handler to capture timer out event  
    if (signal(SIGALRM, timerHandler) == SIG_ERR)
    {
        printf("signal(2) failed while setting up for SIGALRM");
        return -1;
    }

    testlolmsg();

    return 0;

/*
    RobotBase::RobotType robot_type = RobotBase::Initialize("test");

    IRComm::Initialize();
//    Ethernet::Initialize();

    // Enable power for the motors
    switch (robot_type) {
        case RobotBase::ACTIVEWHEEL:
            ((ActiveWheel*) RobotBase::Instance())->EnableMotors(true);
            break;
        case RobotBase::KABOT:
            ((KaBot*) RobotBase::Instance())->EnableMotors(true);
            break;
        case RobotBase::SCOUTBOT:
            ((ScoutBot*) RobotBase::Instance())->EnableMotors(true);
            break;
        default:
            break;
    }

    master_IPC.Name("test");
    master_IPC.SetCallback(process_message, NULL);
    master_IPC.Start("localhost", 40000, false);


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

    initVision();
    pthread_create(&vision_thread, NULL, BlobDetection, NULL);
    //main loop
    while (!userQuit) 
    {
        //block until timer event occurs
        while(currentTime==lastupdateTime)
        {
            //need to put some code here, other wise the main thread will not run
            usleep(5000);
        } 

        lastupdateTime = currentTime;

        //this will be called every 100ms
        update(currentTime);

    }

    RobotBase::MSPReset();
*/
    return 0;
}

void seeding(uint8_t *seq, uint32_t size)
{
    uint8_t cmd_data[size + 1];
    cmd_data[0] = size;
    if(seq != NULL)
    {
        for(int i=0;i<size;i++)
            cmd_data[i+1]=seq[i];
    }
    master_IPC.SendData(DAEMON_MSG_SEEDING, cmd_data, sizeof(cmd_data));
}

void recruiting(uint8_t recruiting_side, uint8_t required_robot_type, uint8_t required_robot_side)
{
    uint8_t cmd_data[3];
    cmd_data[0] = recruiting_side; //recruiting side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
    cmd_data[1] = required_robot_type; //recruited robot type: 1 -- KIT, 2 -- Scout, 3 -- ActiveWheel
    cmd_data[2] = required_robot_side; //recruited robot side: 0 -- front, 1 -- left, 2 -- back, 3 -- right
    master_IPC.SendData(DAEMON_MSG_RECRUITING, cmd_data, sizeof(cmd_data));
}

void docking()
{
    master_IPC.SendData(DAEMON_MSG_DOCKING, NULL, 0);
}

void force_quit()
{
    master_IPC.SendData(DAEMON_MSG_FORCE_QUIT, NULL, 0);
}

void query_progress()
{
    int size=2046;
    uint8_t buf[size];
    for(int i=0;i< size;i++)
        buf[i] = i;
    master_IPC.SendData(DAEMON_MSG_PROGRESS_REQ, buf, size);
}

void query_seed_ip()
{
    master_IPC.SendData(DAEMON_MSG_SEED_IP_REQ, NULL, 0);
}

void query_allrobots_ip()
{
    master_IPC.SendData(DAEMON_MSG_ALLROBOTS_IP_REQ, NULL, 0);
}

void query_neighbours_IP(uint8_t side)
{
    uint8_t cmd_data;
    master_IPC.SendData(DAEMON_MSG_NEIGHBOUR_IP_REQ, &cmd_data, 1);
}


void update(int64_t timestamp)
{
    if(timestamp == 20)
    {
        //recruiting(0, 1, 0);
       docking();
        //uint8_t buf[2] = {25, 0}; //test organism "KBKF0000"
        //seeding(buf, 2);

        RobotBase::pauseSPI(true);
    }
    else if(timestamp > 20 && timestamp %20 == 0 && !processing_done)
    {
        query_progress();
    }

    if(processing_done)
    {
        query_neighbours_IP(0);
        printf("robot's docked! %d\n", neighbours_IP[2].i32>>24 & 0xFF);

        RobotBase::pauseSPI(false);

        if(timestamp % 10 ==0)
            query_seed_ip();
        else if(timestamp % 10 ==5)
            query_allrobots_ip();
            
    }

    /*
    else if(timestamp == 100)
    {
        force_quit();
    }
    else if(timestamp == 120)
    {
        docking();
    }*/
}

void process_message(const ELolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection)
        return;

    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    bool valid = true;

    switch(msg->command)
    {
        case DAEMON_MSG_NEIGHBOUR_IP:
            memcpy(&(neighbours_IP[msg->data[0]].i32), msg->data + 1, 4);
            break;
        case DAEMON_MSG_ALLROBOTS_IP:
            if(msg->data[0] == 0)
                printf("%d: can not get all robot's IP as organism is not formed yet, or I am not a seed robot\n", currentTime);
            else
            {
                uint32_t ip[msg->data[0]];
                memcpy((uint8_t*)ip, (uint8_t*)(&msg->data[1]), msg->data[0] * sizeof(uint32_t));
                for(int i=0; i< msg->data[0];i++)
                    printf("%d: %d (%d.%d.%d.%d)\n", i, ip[i], 
                            ip[i] & 0xFF, 
                            (ip[i] >> 8) & 0xFF,
                            (ip[i] >> 16) & 0xFF,
                            (ip[i] >> 24) & 0xFF);
            }
            break;
        case DAEMON_MSG_SEED_IP:
            {
                uint32_t ip;
                memcpy((uint8_t*)&ip, (uint8_t*)(msg->data), sizeof(uint32_t));
                printf("seed's IP: %d (%d.%d.%d.%d)\n", ip, 
                            ip & 0xFF, 
                            (ip >> 8) & 0xFF,
                            (ip >> 16) & 0xFF,
                            (ip >> 24) & 0xFF);

            }
            break;
        case DAEMON_MSG_PROGRESS:
            printf("%d: received progress ack %d\n", currentTime, msg->data[0]);
            processing_done = msg->data[0];
            break;
        case DAEMON_MSG_ACK:
            if(msg->data[0] ==0 )
                printf("%d: request %s %s\n", currentTime, daemon_message_names[msg->data[0]], msg->data[0] == 0 ? "rejected" : "accepted");
        default:
            break;
    }

}

void signalHandler(int signal) 
{
    printf("user quit signals captured: ctrl + c \n");
    userQuit = 1;
}

void timerHandler(int dummy)
{
    currentTime++;
}


void testlolmsg()
{
#define BUFFERSIZE 640*480*3
#define PARSE_BUFFERSIZE 640*480*3
    uint8_t buf[BUFFERSIZE];
    ByteQueue bq;


    ELolParseContext parseContext;
    BQInit(&bq, buf, BUFFERSIZE);
    uint8_t *lolbuffer = new uint8_t[PARSE_BUFFERSIZE];
    ElolmsgParseInit(&parseContext, lolbuffer, PARSE_BUFFERSIZE);


    printf("size LolMessage --- %d\n", sizeof(ELolMessage));
    printf("size uint8_t* --- %d\n", sizeof(uint8_t *));

    int size=65534;
    uint8_t *data = new uint8_t[size];
    for(int i=0;i<size;i++) 
        data[i]=0x30 + i;

    ELolMessage msg;
    ElolmsgInit(&msg, 0x1, data, size);

    int size2 = ElolmsgSerializedSize(&msg);
    uint8_t *outbytes = new uint8_t[size2];
    ElolmsgSerialize(&msg, outbytes);

    printf("data size: %d, lolmsg size: %d\n", size, size2);

    /*
    printf("");
    for(int i=0;i<size2;i++)
        printf("%#x\t", outbytes[i]);
    printf("\n");
    */

    int parsed=0,read=size2 -5;
    while (parsed < read)
    {
        printf("--- parsed %d --\n", parsed);
        parsed += ElolmsgParse(&parseContext, outbytes + parsed, size2 - 5);
        printf("--- parsed %d --\n", parsed);
        parsed += ElolmsgParse(&parseContext, outbytes + parsed, size2 - 5);
        printf("--- parsed %d --\n", parsed);
        ELolMessage* msg = ElolmsgParseDone(&parseContext);
        if(msg!=NULL)
        {
            ELolMessage*tt=(ELolMessage*)parseContext.buf;
            printf("length: %d %#x %d\n", msg->length, msg->data, sizeof(ParseState));
            //for(int i=0;i<msg->length;i++)
            //    printf("%#x\n", msg->data[i]);
        }

    }
};

void * BlobDetection(void * ptr)
{
    printf("Blob detection thread is running\n");

    static int count = 0;

    timeval starttime, sys_time;
    gettimeofday(&starttime, NULL);

    while(userQuit!=1)
    {
//#if !defined(LAPTOP) 
        count++;
      //  pthread_mutex_lock(&vision_mutex);
        img = cap.captureFrame();
        if(img !=NULL)
        {
            frame.hdr.timestamp = img->timestamp;
            frame.data = img->data;
            vision.processFrame(reinterpret_cast<CMVision::image_pixel*>(img->data));
            if(currentTime %10 ==0)
            {
                printf("processed frame @ %d fps\n", count);
                count=0;
            }
        }
        cap.releaseFrame(img);

        gettimeofday(&sys_time, NULL);

        for(int ch=0;ch<MAX_COLORS_TRACKED;ch++)
        {
            CMVision::CMVision::region* reg = vision.getRegions(ch);
            int index=0;

            while(reg)
            {
                reg = reg->next;
                index++;
                //if(index >= MAX_OBJECTS_TRACKED) 
                //    break;
            }
        }

        //pthread_mutex_unlock(&vision_mutex);
//#endif
    }

    printf("Blob detection thread is exiting\n");
    return NULL;
}

bool initVision()
{
    img_width=640;
    img_height=480;

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
}
