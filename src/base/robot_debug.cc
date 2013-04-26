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
        printf("%d: remote control command received [%s]\n",timestamp, remote_cmd_names[data[1]]);

}

bool Robot::ParseCMD(char * buf)
{
        char cmd[16];
    int arg1=0,arg2=0, arg3, arg4;
    int res=0;
    int i=0;
    //cmd id para1 para2 para3
    //locking 115 0 3; locking robot115's front side, open
    //move 115 speed1 speed2 speed3
    //hinge 115 up
    if((res=sscanf(buf, "%s %d %d %d %d\n", cmd, &arg1, &arg2 , &arg3, &arg4))!=EOF)
    {
        switch(res)
        {
            case 1:
                if(strcmp(cmd,"exit")==0)
                {
                    return false;
                }
                else if(strcmp(cmd, "help")==0)
                {
                    printf("\t lock robot_id robot_side[0-3] open/close[2 or 0]\n");
                    printf("\t move robot_id speed0 speed1 speed2\n");
                    printf("\t hing robot_id speed angle\n");
                }
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                if(strcmp(cmd, "lock")==0)
                {
                    uint8_t data[4];
                    data[0] = arg1; //id
                    data[1] = CMD_LOCKING_MOTOR; //cmd
                    data[2] = arg2; //side
                    data[3] = arg3 > 0 ? 0 : (arg3 < 0 ? 2 : 1); //open or close
                    BroadcastIRMessage(0, IR_MSG_TYPE_REMOTE_DEBUG, data, 4, 0);
                    if(data[3] == 0)
                    {
                        printf("   unlocking robot%d's side [%d]\n", arg1, arg2);
                    }
                    else if(data[3] == 1)
                    {
                        printf("   stop robot%d's side [%d] locking motor\n", arg1, arg2);
                    }
                    else if(data[3] == 2)
                    {
                        printf("   locking robot%d's side [%d]\n", arg1, arg2);
                    }
                }
                else if(strcmp(cmd, "hinge")==0)
                {
                    printf("   move robot%d's hinge to [%d] at speed [%d]\n", arg1, arg2, arg3);
                }
                break;
            case 5:
                if(strcmp(cmd, "move")==0)
                {
                    printf("   move robot%d at speed [%d %d %d]\n", arg1, arg2, arg3, arg4);

                }
                break;
            default:
                break;
        }
    }

    return true;
}

