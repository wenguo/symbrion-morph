#include "robot.hh"

void Robot::RemoteDebugging(char * data)
{
    if(!data)
        return;

    printf("processing remote cmd %d %d %d %d\n", data[0], data[1], data[2], data[3]);
    //message to all or to me?
    if((uint8_t)data[0] !=0 && (uint8_t)data[0] != id)
        return;
    printf("processing remote cmd %#x %#x %#x %#x\n", data[0], data[1], data[2], data[3]);


    bool valid = true;
    switch(data[1])
    {
        case  CMD_LOCKING_MOTOR:
            {
                int channel = data[2];
                if(channel>=0 && channel<NUM_DOCKS)
                {
                    int status =  data[3] - 1;
                    //already closed?
                    if(locking_motors_status[channel] == CLOSED)
                    {
                        if(status == CLOSE)
                            locking_motors_status[channel]=OPENED;
                        SetDockingMotor(channel, status);
                    }
                    else if(locking_motors_status[channel] == OPENED)
                    {
                        if(status == OPEN)
                            locking_motors_status[channel]=CLOSED;
                        SetDockingMotor(channel, status);
                    }
                    else
                        printf("channel: %d locking motor is engaged\n", channel);

                }
            }
            break;
        case CMD_2D_LOCOMOTION:
            break;
        case CMD_3D_HINGE:
            break;
        default:
            valid = false;
            break;
    }

    if(valid)
        printf("%d: remote control command received\n", remote_cmd_names[data[1]]);

}
