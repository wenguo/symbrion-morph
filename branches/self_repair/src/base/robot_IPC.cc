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
                //followed by speed, count, rotation, angle, [int, int, int, int]
                memcpy((uint8_t*)robot->hinge_command, msg->data, sizeof(robot->hinge_command));
                uint8_t command = msg->command; 
                conn->SendData(IPC_MSG_ACK, &command, 1);
            }
            break;
        case IPC_MSG_LOCOMOTION_2D_REQ:
            {
                //followed by speed[0], speed[1], speed[2]
                memcpy((uint8_t*)robot->locomotion_command, msg->data, sizeof(robot->locomotion_command));
                uint8_t command = msg->command; 
                conn->SendData(IPC_MSG_ACK, &command, 1);
                printf("%d: received [%s] %d %d %d %d\n", robot->timestamp, ipc_message_names[msg->command],
                        robot->locomotion_command[0],
                        robot->locomotion_command[1],
                        robot->locomotion_command[2],
                        robot->locomotion_command[3]
                        );
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
                int reflective_calibrated[NUM_IRS];
                uint8_t data[sizeof(reflective_calibrated) + 3];
                data[0] = msg->command;
                data[1] = msg->data[0];
                data[2] = msg->data[1];
                for(int i=0;i<NUM_IRS;i++)
                    reflective_calibrated[i]=robot->reflective[i] - robot->para.reflective_calibrated[i];
                memcpy(data+3, (uint8_t*)reflective_calibrated, sizeof(reflective_calibrated));
                
                conn->SendData(IPC_MSG_ACK, data, sizeof(data));
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
                pthread_mutex_lock(&robot->IPC_data_mutex);
                robot->commander_acks[conn->addr.sin_addr.s_addr]++;
                pthread_mutex_unlock(&robot->IPC_data_mutex);
                //for sensor data msg->data: 
                // 0 -- REQ type
                // 1 -- organism side; front, left, back, right
                // 2 -- index, for left side and right side
                // 3 -- sensor mask, each bit correspond to one sensor
                // 2 - 34, sensor data
                switch(msg->data[0])
                {
                    case IPC_MSG_PROXIMITY_DATA_REQ:
                        break;
                    case IPC_MSG_BEACON_DATA_REQ:
                        break;
                    case IPC_MSG_REFLECTIVE_DATA_REQ:
                        uint8_t config[2];
                        int data[NUM_IRS];
                        memcpy(config, msg->data + 1, 2);
                        memcpy((uint8_t*)data, msg->data + 3, sizeof(data));
                        robot->UpdateOGIRSensors(config, data, 0);
                        break;
                    default:
                        break;
                }
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

}
