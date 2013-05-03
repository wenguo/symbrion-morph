#include "robot.hh"
void Robot::Process_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;

    uint8_t receiver = msg->data[0];

    //do I need to relay the message
    if(receiver != 0 && receiver != ((robot->my_IP.i32 >> 24)&0xFF))
    {
        if(!robot->commander_IPC.Server())
            printf("This shouldn't happen, please check! data: %d %d\n", msg->data[0], msg->data[1]);
        else
        {
            robot->commander_IPC.SendData(robot->getFullIP(receiver).i32, msg->command, (uint8_t*)msg->data, msg->length);
        }

    }
    else
    {

        bool ack_required = true;

        uint8_t* data = (uint8_t*)msg->data + 2;

        switch(msg->command)
        {       
            case MSG_TYPE_ORGANISM_FORMED:
                robot->organism_formed = true;
                break;
            case MSG_TYPE_LOWERING:
                robot->msg_lowering_received = true;
                break;
            case MSG_TYPE_DISASSEMBLY:
                robot->msg_disassembly_received = true;
                break;
            case IPC_MSG_HINGE_3D_MOTION_REQ:
                {
                    //followed by speed, count, rotation, angle, [int, int, int, int]
                    memcpy((uint8_t*)robot->hinge_command, data, sizeof(robot->hinge_command));
                    uint8_t command = msg->command; 
                    robot->IPCSendMessage(IPC_MSG_ACK, &command, 1);
                    robot->timestamp_motors_cmd_received  = robot->timestamp;
                }
                break;
            case IPC_MSG_LOCOMOTION_2D_REQ:
                {
                    //followed by speed[0], speed[1], speed[2]
                    memcpy((uint8_t*)robot->locomotion_command, data, sizeof(robot->locomotion_command));
                    uint8_t command = msg->command; 
                    robot->IPCSendMessage(IPC_MSG_ACK, &command, 1);
                    robot->timestamp_motors_cmd_received  = robot->timestamp;
                    printf("%d: received [%s] %d %d %d %d\n", robot->timestamp, message_names[msg->command],
                            robot->locomotion_command[0],
                            robot->locomotion_command[1],
                            robot->locomotion_command[2],
                            robot->locomotion_command[3]
                          );
                }
                break;
            case IPC_MSG_IRSENSOR_DATA_REQ:
                {
                    int irsensor_data[NUM_IRS];
                    uint8_t buf[sizeof(irsensor_data) + 4];
                    buf[0] = msg->command;
                    buf[1] = data[0]; //ir sensor type
                    buf[2] = (robot->my_IP.i32>>24)&0xFF; //my ip
                    buf[3] = 0;//reserved
                    if(buf[1] == IR_REFLECTIVE_DATA)
                    {
                        for(int i=0;i<NUM_IRS;i++)
                            irsensor_data[i]=robot->reflective[i] - robot->para.reflective_calibrated[i];
                    }
                    else if(buf[1] == IR_PROXIMITY_DATA)
                    {
                        for(int i=0;i<NUM_IRS;i++)
                            irsensor_data[i]=robot->reflective[i] - robot->para.reflective_calibrated[i];
                    }
                    else if(buf[1] == IR_BEACON_DATA)
                    {
                        for(int i=0;i<NUM_IRS;i++)
                            irsensor_data[i]=robot->reflective[i] - robot->para.reflective_calibrated[i];
                    }

                    memcpy(buf+4, (uint8_t*)irsensor_data, sizeof(irsensor_data));

                    robot->IPCSendMessage(IPC_MSG_ACK, buf, sizeof(buf));
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
                    // 1 --2 -- configuration tag in the organism 
                    // 3 - 34, sensor data
                    switch(data[0])
                    {
                        case IPC_MSG_IRSENSOR_DATA_REQ:
                            uint8_t config[2];
                            int buf[NUM_IRS];
                            memcpy(config, data + 2, 2);
                            memcpy((uint8_t*)buf, data + 4, sizeof(data));
                            robot->UpdateOGIRSensors(config, buf, data[1]);
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
            printf("%d: received [%s - %s]\n", robot->timestamp, message_names[msg->command], message_names[msg->data[2]] );
        else
        {
            printf("%d: received [%s]\n", robot->timestamp, message_names[msg->command]);
        }

    }

}
