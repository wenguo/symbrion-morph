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

    SPIVerbose = DEBUGALL;

    //Initialise IRobot instance
    RobotBase::Initialize("IRDocking");

    //Initialise Robot instance for IR docking, it will create the right type of robot automatically
    Robot::Initialise(RobotBase::Instance());

    //Get the robot pointer 
    Robot *robot = Robot::Instance();
    

    //Define recruiting types: from which robot, which side, recruits which robot, which side
    Robot::Symbol r;
    r.type1 = RobotBase::SCOUTBOT;
    r.side1 = ScoutBot::REAR;
    r.type2 = RobotBase::SCOUTBOT;
    r.side2 = ScoutBot::FRONT;

    if(argc ==2)
    {

        if(strcmp(args[1], "r")==0)
        {
            //Using Recruiting(Symbol r) to set robot to be recruiter, you must decide 'r' in your own controller
            robot->Recruiting(r);

            //Using GetRecruitingStatus to check if recruiting is done. If not, your controller should not access Motors + IR sensors
            while (!userQuit && robot->GetRecruitingStatus(ScoutBot::REAR) != Robot::RECRUITING_DONE)
            {
                printf("*\n");
                usleep(1000000);
            }

            printf("Recuriting done!\n");
        }
        else if(strcmp(args[1],"d")==0)
        {
            //Using Docking(Symbol r) to set robot to be recruitee, you must set 'r' in your own controller
            robot->Docking(r);

            //Using GetDockingStatus to check if docking is done. If not, your controller should not access Motors + IR sensors
            while (!userQuit && robot->GetDockingStatus() != Robot::DOCKING_DONE)
            {
                printf("*\n");
                usleep(1000000);
            }

            printf("Docking done!\n");

        }
    }

    // Main loop
    while (!userQuit) 
    {

        printf("Do something else\n");
        usleep(1000000);
    }

    RobotBase::MSPReset();
    printf("Quit\n");

    return 0;
}
