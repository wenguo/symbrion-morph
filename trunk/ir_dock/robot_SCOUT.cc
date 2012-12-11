#include "robot.hh"

void RobotSCOUT::Recruiting()
{
    printf("\t Scout Recruiting\n");
    printf("ScoutBot Recruiting\n");
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

void RobotSCOUT::Docking()
{
    printf("\t Scout Docking\n");
    printf("ScoutBot Docking\n");
    switch(docking_status)
    {
        case LOCATEBEACON:
            break;
        case ALIGNING:
            break;
        case LOCKING:
            break;
        default:
            break;
    }

}

void RobotSCOUT::UpdateSensors()
{
    printf("Scout Update Sensor\n");
}

void RobotSCOUT::UpdateActuators()
{
    printf("Scout Update Actuators\n");
}
