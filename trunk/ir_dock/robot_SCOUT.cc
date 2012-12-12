#include "robot.hh"

void RobotSCOUT::Recruiting()
{
    printf("\tScoutBot Recruiting\n");
    for(int i=0;i<4;i++)
    {
        switch(recruiting_status[i])
        {
            case STAGE1:
                break;
            case STAGE2:
                break;
            case STAGE3:
                break;
            default:
                break;
        }
    }
}

void RobotSCOUT::LocateBeacon()
{
    printf("\tScoutBot LocateBeaon\n");
}

void RobotSCOUT::Aligning()
{
    printf("\tScoutBot Aligning\n");
}

void RobotSCOUT::Locking()
{
    printf("\tScoutBot Locking\n");
}

void RobotSCOUT::UpdateSensors()
{
    printf("\tScout Update Sensor\n");
}

void RobotSCOUT::UpdateActuators()
{
    printf("\tScout Update Actuators\n");
}
