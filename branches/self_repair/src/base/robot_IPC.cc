#include "robot.hh"
void Robot::Process_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;

    uint8_t receiver = msg->data[0];
    uint8_t sender = msg->data[1];

    //do I need to relay the message
    if(receiver != 0 && receiver != ((robot->my_IP.i32 >> 24)&0xFF))
    {
        if(robot->commander_IPC.Server())
            printf("This shouldn't happen, please check! data: %d %d\n", msg->data[0], msg->data[1]);
        else
        {
            robot->master_IPC.SendData(robot->getFullIP(receiver).i32, msg->command, (uint8_t*)msg->data, msg->length);
        }

    }
    else
    {

        bool ack_required = true;

        uint8_t command = msg->command; 
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
            case MSG_TYPE_IP_ADDR_COLLECTION:
                {
                    uint8_t channel = robot->getEthChannel(getFullIP(sender));
                    if(channel < SIDE_COUNT)
                    {
                        //first fill in the ip_list in the right position of 'mytree' using received data
                        std::vector<uint8_t> branch_IPs;
                        printf("IPC received IPs (%d):", data[0]);
                        for(int i=0;i<data[0];i++)
                        {
                            printf("%d ", (uint8_t)data[i+1]);
                            branch_IPs.push_back(data[i+1]);
                        }
                        printf("\n");
                        if(branch_IPs.size() > 0)
                            robot->mytree.setBranchIPs(robot_side(channel), branch_IPs);

                        robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, (uint8_t*)&msg->command, 1);

                    }
                    else
                    {
                        printf("%d: message is not from my neighbour ip %s\n", robot->timestamp, IPToString(getFullIP(sender)));

                    }
                }
                break;
            case IPC_MSG_HINGE_3D_MOTION_REQ:
                {
                    //followed by speed, count, rotation, angle, [int, int, int, int]
                    memcpy((uint8_t*)robot->hinge_command, data, sizeof(robot->hinge_command));
                    robot->IPCSendMessage(IPC_MSG_ACK, &command, 1);
                    robot->timestamp_hinge_motor_cmd_received  = robot->timestamp;
                }
                break;
            case IPC_MSG_LOCOMOTION_2D_REQ:
                {
                    //followed by speed[0], speed[1], speed[2]
                    memcpy((uint8_t*)robot->locomotion_command, data, sizeof(robot->locomotion_command));
                    robot->IPCSendMessage(IPC_MSG_ACK, &command, 1);
                    robot->timestamp_locomotion_motors_cmd_received  = robot->timestamp;
                    //printf("%d: received [%s] %d %d %d %d\n", robot->timestamp, message_names[msg->command],
                    //        robot->locomotion_command[0],
                    //        robot->locomotion_command[1],
                    //        robot->locomotion_command[2],
                    //        robot->locomotion_command[3]
                    //      );
                }
                break;
            case IPC_MSG_DOCKING_ROTATION_REQ:
                {
                    if(robot->type == ROBOT_AW)
                    {
                        uint8_t channel = robot->getEthChannel(getFullIP(sender));
                        int8_t angle = data[0];
                        robot->RotateDockingUnit(channel, angle);
                        robot->IPCSendMessage(getFullIP(sender).i32, IPC_MSG_ACK, &command, 1);
                    }

                }
                break;
            case IPC_MSG_RESET_POSE_REQ:
                {
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        printf("%d: process side %d\n", robot->timestamp, i);
                        OrganismSequence::Symbol sym = OrganismSequence::Symbol(robot->docked[i]);
                        if(sym.type2 == ROBOT_AW)
                        {
                            printf("%d request to rotate docking angle %d %s\n", robot->timestamp, i, IPToString(robot->neighbours_IP[i]));
                            int8_t angle = 90;
                            robot->IPCSendMessage(robot->neighbours_IP[i].i32,IPC_MSG_DOCKING_ROTATION_REQ, (uint8_t*)&angle, 1);
                            break;
                        }
                    }
                    robot->IPCSendMessage(getFullIP(sender).i32, IPC_MSG_ACK, &command, 1);
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
                {
                    if(!robot->seed)
                        robot->msg_raising_start_received = true;
                }
                break;
            case IPC_MSG_RAISING_STOP:
                {
                    if(!robot->seed)
                        robot->msg_raising_stop_received = true;
                }
                break;
            case IPC_MSG_LOWERING_START:
                //            msg_lowering_start_received = true;
                break;
            case IPC_MSG_LOWERING_STOP:
                //          msg_lowering_stop_received = true;
                break;
            case IPC_MSG_CLIMBING_START:
                {
                    if(!robot->seed)
                        robot->msg_climbing_start_received = true;
                }
                break;
            case IPC_MSG_CLIMBING_STOP:
                {
                    if(!robot->seed)
                        robot->msg_climbing_stop_received = true;
                }
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
                        case MSG_TYPE_IP_ADDR_COLLECTION:
                            printf("%d: MSG_TYPE_IP_ADDR_COLLECTION acknowleged %d %d\n", robot->timestamp, sender, receiver);
                            robot->RemoveFromQueue(robot->getEthChannel(getFullIP(sender)),MSG_TYPE_IP_ADDR_COLLECTION);
                            printf("%d: removed any message in IR txQue from  channel %d\n", robot->timestamp, robot->getEthChannel(getFullIP(sender)));
                            printf("%d: received [%s - %s]\n", robot->timestamp, message_names[msg->command], message_names[msg->data[2]] );
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

        /*
        if(msg->command == IPC_MSG_ACK)
            printf("%d: received [%s - %s]\n", robot->timestamp, message_names[msg->command], message_names[msg->data[2]] );
        else
        {
            printf("%d: received [%s]\n", robot->timestamp, message_names[msg->command]);
        }*/

    }

}

//send data to specified address, it may be relayed by the server
void Robot::IPCSendMessage(uint32_t dst,  uint8_t type, const uint8_t *data, int size)
{
    uint8_t buf[size + 2];
    buf[0] = (dst >> 24) & 0xFF;
    buf[1] = (my_IP.i32 >> 24) & 0xFF;
    memcpy(buf+2, data, size);

    if(seed) 
        master_IPC.SendData(dst, type, buf, sizeof(buf));
    else
        commander_IPC.SendData(type, buf, sizeof(buf));
}

//send data to all from the server, or send back to server from the client
void Robot::IPCSendMessage(uint8_t type, const uint8_t *data, int size)
{
    uint8_t buf[size + 2];
    buf[0] = 0;
    buf[1] = (my_IP.i32 >> 24) & 0xFF;
    memcpy(buf+2, data, size);

    if(seed) 
        master_IPC.SendData(type, buf, sizeof(buf));
    else
        commander_IPC.SendData(type, buf, sizeof(buf));
}


