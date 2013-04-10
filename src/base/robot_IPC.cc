#include "robot.hh"
void Robot::Process_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;

    bool ack_required = true;

    switch(msg->command)
    {       
        case IPC_MSG_HINGE_3D_MOTION_REQ:
            {
                //followed by speed, angle, rotation angle, [int, int, int]
                memcpy((uint8_t*)&(robot->hinge_command), msg->data, sizeof(robot->hinge_command));
            }
            break;
        case IPC_MSG_LOCOMOTION_2D_REQ:
            {
                //followed by speed[0], speed[1], speed[2]
            }
            break;
        case IPC_MSG_PROXIMITY_DATA_REQ:
            {
            }
            break;
        case IPC_MSG_BEACON_DATA_REQ:
            {
            }
            break;
        case IPC_MSG_REFLECTIVE_DATA_REQ:
            {
            }
            break;
        case IPC_MSG_RAISING_START:
            robot->msg_raising_start_received = true;
            break;
        case IPC_MSG_RAISING_STOP:
            robot->msg_raising_stop_received = true;
            break;
        case IPC_MSG_LOWERING_START:
//            msg_lowering_start_received = true;
            break;
        case IPC_MSG_LOWERING_STOP:
  //          msg_lowering_stop_received = true;
            break;
        case IPC_MSG_ACK:
            {
                ack_required = false;
                robot->commander_acks[conn->addr.sin_addr.s_addr]++;
            }
            break;
        default:
            ack_required = false;
            printf("unknow command\n");
            break;
    }

    if(msg->command == IPC_MSG_ACK)
        printf("%d: received [%s - %s]\n", robot->timestamp, ipc_message_names[msg->command], ipc_message_names[msg->data[0]] );
    else
    {
        printf("%d: received [%s]\n", robot->timestamp, ipc_message_names[msg->command]);
    }

    if(ack_required)
    {
        uint8_t command = msg->command; 
        conn->SendData(IPC_MSG_ACK, &command, 1);
    }
}
