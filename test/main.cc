#include "IRobot.h"
#include "comm/IRComm.h"
#include "comm/Ethernet.h"
#include "ipc.hh"
#include "ipc_interface.hh"
#include <pthread.h>
#include <signal.h>
#include <cstdio>
#include <string.h>
#include <sys/time.h>

void process_message(const LolMessage*msg, void* connection, void *user_ptr);
void update(int64_t timestamp);

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
    master_IPC.SendData(DAEMON_MSG_PROGRESS_REQ, NULL, 0);
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
        //recruiting(2, 1, 0);
        //docking();
        uint8_t buf[2] = {25, 0}; //test organism "KBKF0000"
        seeding(buf, 2);

        RobotBase::pauseSPI(true);
    }
    else if(timestamp > 20 && timestamp %5 == 0 && !processing_done)
    {
        query_progress();
    }

    if(processing_done)
    {
        query_neighbours_IP(2);
        printf("robot's docked! %d\n", neighbours_IP[2].i32>>24 & 0xFF);

        RobotBase::pauseSPI(false);
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

void process_message(const LolMessage*msg, void* connection, void *user_ptr)
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
        case DAEMON_MSG_PROGRESS:
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

