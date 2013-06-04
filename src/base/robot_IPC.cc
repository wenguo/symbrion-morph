#include "robot.hh"
void Robot::Relay_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    uint8_t receiver = msg->data[0];
    uint8_t sender = msg->data[1];

    if(receiver != 0)// && receiver != ((robot->my_IP.i32 >> 24)&0xFF))
    {
       // printf("%d: relay message %s, from %d to %d\n", robot->timestamp, message_names[msg->command], (int)sender, (int)receiver);
        robot->master_IPC.SendData(robot->getFullIP(receiver).i32, msg->command, (uint8_t*)msg->data, msg->length);
    }
    else
    {
       // printf("%d: relay message %s, from %d to everyone\n", robot->timestamp, message_names[msg->command], (int)sender);
        robot->master_IPC.SendData(msg->command, (uint8_t*)msg->data, msg->length);
    }

}
void Robot::Process_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    uint8_t receiver = msg->data[0];
    uint8_t sender = msg->data[1];

    if(receiver != 0 && receiver != ((robot->my_IP.i32 >> 24)&0xFF))
    {
            printf("This shouldn't happen, please check! data: %d %d\n", (int)receiver, (int)sender);
    }
    else
    {
        //msg->data[0] -- reciver
        //msg->data[1] -- sender

        bool ack_required = true;

        uint8_t command = msg->command; 
        uint8_t* data = (uint8_t*)msg->data + 2;

        switch(msg->command)
        {       
            case MSG_TYPE_ORGANISM_FORMED:
                robot->organism_formed = true;
                break;
            case MSG_TYPE_LOWERING:
                {
                    if(!robot->seed)
                        robot->msg_lowering_received = true;
                }
                break;
            case MSG_TYPE_DISASSEMBLY:
                {
                    if(!robot->seed)
                        robot->msg_disassembly_received = true;
                }
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
                    robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, &command, 1);
                    robot->timestamp_hinge_motor_cmd_received  = robot->timestamp;
                }
                break;
            case IPC_MSG_LOCOMOTION_2D_REQ:
                {
                    //followed by speed[0], speed[1], speed[2]
                    memcpy((uint8_t*)robot->locomotion_command, data, sizeof(robot->locomotion_command));
                    robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, &command, 1);
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
                        printf("%d received request to rotate docking angle %d from side %d\n", robot->timestamp, (int)angle, (int)channel);
                    }

                }
                break;
            case IPC_MSG_RESET_POSE_REQ:
                {
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
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
                            irsensor_data[i]=robot->proximity[i];
                    }
                    else if(buf[1] == IR_BEACON_DATA)
                    {
                        for(int i=0;i<NUM_IRS;i++)
                            irsensor_data[i]=robot->beacon[i];
                    }
                    else if(buf[1] == IR_AUX_REFLECTIVE_DATA)
                    {
                        for(int i=0;i<NUM_IRS;i++)
                            irsensor_data[i]=robot->get_aux_reflective(i) - robot->para.aux_reflective_calibrated[i];
                    }

                    memcpy(buf+4, (uint8_t*)irsensor_data, sizeof(irsensor_data));

                    robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, buf, sizeof(buf));
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
            case IPC_MSG_OPAQUE:
                {
                    robot->user_input += data[0] - 1;
                    printf("%d: opaque message received from %s: user_input %d (%#x)\n", robot->timestamp, IPToString(getFullIP(sender)), robot->user_input, data[0]);
                }
                break;
            case IPC_MSG_ORGANISM_SEQ:
                {
                    robot->msg_organism_seq_received = true;
                    robot->mytree.Clear();
                    robot->mybranches.clear();
                    //data[0] -- seeds' ip
                    //data[1] -- seeds' port
                    //data[2] -- size of branch 
                    //data[3]-data[x] -- branch shape 
                    robot->commander_IP = getFullIP(data[0]);
                    robot->commander_port = COMMANDER_PORT_BASE + (uint8_t)data[1];

                    //fill mytree, removing the parental info
                    rt_status ret = robot->mytree.reBuild((uint8_t*)&(data[4]), data[2]-2);

                    robot->parent_side = robot->getEthChannel(getFullIP(sender));
                    robot->reshaping_processed |= 1<<robot->parent_side;
                    printf("%d: received msg_organism_seq from %s\n", robot->timestamp, IPToString(getFullIP(sender)));
                    std::cout<<"new seq: "<<robot->mytree<<std::endl;
                }
                break;
            case IPC_MSG_RESHAPING_START:
                {
                    robot->msg_reshaping_start_received = true;
                    //clear my tree
                    robot->mytree.Clear();
                    robot->mybranches.clear();
                    //data[0] -- new seeds' ip
                    //data[1] -- new seeds' port
                    //data[2] -- size of new shape
                    //data[3]-data[x] -- new shape
                    robot->commander_IP = getFullIP(data[0]);
                    robot->commander_port = COMMANDER_PORT_BASE + (uint8_t)data[1];
                    if(robot->my_IP == robot->commander_IP)
                    {
                        robot->reshaping_seed = true;
                        //rebuild mytree
                        rt_status ret = robot->mytree.reBuild((uint8_t*)&(data[3]), data[2]);
                        std::cout<<robot->ClockString()<<":receive new subtree: "<<robot->mytree<<std::endl;
                        robot->parent_side = SIDE_COUNT;
                    }
                }
                break;
            case IPC_MSG_RESHAPING_DONE:
                {
                    robot->msg_reshaping_done_received = true;
                }
                break;
            case IPC_MSG_ACK:
                {
                    ack_required = false;
                    pthread_mutex_lock(&robot->IPC_data_mutex);
                    robot->commander_acks[getFullIP(sender).i32]++;
                    pthread_mutex_unlock(&robot->IPC_data_mutex);
                    switch(data[0])
                    {
                        case IPC_MSG_IRSENSOR_DATA_REQ:
                            //for sensor data msg->data: 
                            // 1 -- sensor data type
                            // 2 -- 3 -- configuration tag in the organism 
                            // 4 - 35, sensor data
                            uint8_t config[2];
                            int32_t buf[NUM_IRS];
                            memcpy(config, data + 2, 2);
                            memcpy((uint8_t*)buf, data + 4, sizeof(buf));
                            robot->UpdateOGIRSensors(config, buf, data[1]);
                            break;
                        case MSG_TYPE_IP_ADDR_COLLECTION:
                            robot->RemoveFromQueue(robot->getEthChannel(getFullIP(sender)),MSG_TYPE_IP_ADDR_COLLECTION);
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

//send data to specified address, it will be relayed by the server
void Robot::IPCSendMessage(uint32_t dst,  uint8_t type, const uint8_t *data, int size)
{
    uint8_t buf[size + 2];
    buf[0] = (dst >> 24) & 0xFF;
    buf[1] = (my_IP.i32 >> 24) & 0xFF;
    memcpy(buf+2, data, size);

    commander_IPC.SendData(type, buf, sizeof(buf));
}

//send data to all from the master_IPC, or send back to master from the client, exclude the seed
void Robot::IPCSendMessage(uint8_t type, const uint8_t *data, int size)
{
    IPCSendMessage(0, type, data, size);
}


