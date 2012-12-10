#include "robot_KIT.hh"
#include "robot_AW.hh"
#include "robot_SCOUT.hh"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

int64_t currentTime=0;
int64_t lastupdateTime=0;
int userQuit = 0;


void timerHandler(int dummy);
void signalHandler(int dummy);

int main(int argc, char * argv[])
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

    //set signal handler to capture timer out event  
    if (signal(SIGALRM, timerHandler) == SIG_ERR)
    {
        printf("signal(2) failed while setting up for SIGALRM");
        return -1;
    }

    RobotBase::RobotType robot_type = RobotBase::Initialize("morph");

    //create robot object
    Robot * robot;
    char cf_path[64];
    char cf_name[128];
    if(argc == 2)
        sprintf(cf_path,"%s", argv[1]);
    else
        sprintf(cf_path,"/flash/morph");


    if(robot_type == RobotBase::KABOT)
    {
        robot = new RobotKIT((KaBot*)RobotBase::Instance());
        sprintf(cf_name,"%s/kit_option.cfg", cf_path);
    }
    else if(robot_type == RobotBase::ACTIVEWHEEL)
    {
        printf("create new robots\n");
        robot = new RobotAW ((ActiveWheel *)RobotBase::Instance());
        printf("create new robots done\n");
        sprintf(cf_name,"%s/aw_option.cfg", cf_path);
    }
    else if(robot_type == RobotBase::SCOUTBOT)
    {
        robot = new RobotSCOUT((ScoutBot *)RobotBase::Instance());
        sprintf(cf_name,"%s/scout_option.cfg", cf_path);
    }
    else
    {
        printf("unknow robot type, quit\n");
        return -1;
    }
    
    if(!robot->Init(cf_name))
        return -1;


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

    //main loop until user quit signals captured
    while (userQuit!=1)
    {
        //block until timer event occurs
        while(currentTime==lastupdateTime)
        {
            //need to put some code here, other wise the main thread will not run
            usleep(5000);
        } 

        lastupdateTime = currentTime;
        robot->Update(currentTime);
    }

    robot->Stop();
    delete robot;

    return 0;
}

void timerHandler(int dummy)
{
    currentTime++;
}

void signalHandler(int dummy)
{
    printf("user quit signals captured: ctrl + c \n");
    userQuit = 1;
}
