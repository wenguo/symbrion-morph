#include "robot.hh"
#include <pthread.h>
#include <signal.h>
#include <cstdio>
#include <string.h>

using namespace std;
int userQuit = 0;

void signalHandler(int signal) {
        userQuit = 1;
}

int main(int argc, char** args) 
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


    RobotBase::RobotType robot_type = RobotBase::Initialize("IRDocking");

   
    Robot::Initialise(RobotBase::Instance());

    Robot *robot = Robot::Instance();
    
    printf("Initialised\n");

    Robot::Symbol r;
    r.type1 = RobotBase::SCOUTBOT;
    r.side1 = ScoutBot::REAR;
    r.type2 = RobotBase::SCOUTBOT;
    r.side2 = ScoutBot::FRONT;

    if(argc ==2)
    {

        if(strcmp(args[1], "r")==0)
        {
            printf("As recruiter\n");
            robot->Recruiting(r);

            usleep(1000000);

            while (!userQuit && robot->GetRecruitingStatus(ScoutBot::REAR) != Robot::RECRUITING_DONE)
            {
                printf("checking\n");
               // robot->Recruiting(r);
                usleep(1000000);
            }

            printf("Recuriting done!\n");
        }
        else if(strcmp(args[1],"d")==0)
        {
            printf("As recruitee\n");
            robot->Docking(r);
            usleep(1000000);

            while (!userQuit && robot->GetDockingStatus() != Robot::DOCKING_DONE)
            {
                usleep(1000000);
            }

            printf("Docking done!\n");

        }
    }

    // Main loop
    while (!userQuit) 
    {

        //sleep for 1 second
                printf("mainloop\n");
        usleep(1000000);
    }

    RobotBase::MSPReset();
    printf("Quit\n");

    return 0;
}
