#include "robot.hh"
void Robot::Relay_Organism_command(const ELolMessage*msg, void* connection, void *user_ptr)
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
void Robot::Process_Organism_command(const ELolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    uint8_t receiver = msg->data[0];
    uint8_t sender = msg->data[1];
    uint8_t channel = robot->getEthChannel(getFullIP(sender));

    if(receiver != 0 && receiver != ((robot->my_IP.i32 >> 24)&0xFF))
    {
            printf("This shouldn't happen, please check! data: %d %d\n", (int)receiver, (int)sender);
    }
    else
    {
        //msg->data[0] -- reciver
        //msg->data[1] -- sender

        bool ack_required = false;

        uint8_t command = msg->command; 
        uint8_t* data = (uint8_t*)msg->data + 2;

        switch(msg->command)
        {       
            case MSG_TYPE_ORGANISM_FORMED:
                {
                    robot->organism_formed = true;
                    robot->target.Clear();
                    rt_status ret = robot->target.reBuild((uint8_t*)&(data[1]), data[0]);
                    std::cout<<"target seq: "<<robot->target<<std::endl;
                }
                break;
            case MSG_TYPE_LOWERING:
                {
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
                    {
                        printf("%d: received lowering\n", robot->timestamp);
                        robot->msg_lowering_received = true;
                    }
                }
                break;
            case MSG_TYPE_DISASSEMBLY:
                {
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
                        robot->msg_disassembly_received = true;
                }
                break;
            case MSG_TYPE_IP_ADDR_COLLECTION:
                {
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

                        ack_required = true;
                    }
                    else
                        printf("%d: message is not from my neighbour ip %s\n", robot->timestamp, IPToString(getFullIP(sender)));
                }
                break;
            case MSG_TYPE_FAILED:
                {
                    robot->msg_failed_received |= 1<<channel;
                    robot->subog_id = data[0];
                    robot->parent_side = channel;
                    robot->heading = (robot->parent_side + 2) % 4;
                    ack_required = true;
                }
                break;

            case MSG_TYPE_SUB_OG_STRING:
                {
                    if(channel < SIDE_COUNT)
                    {
                        robot->msg_subog_seq_expected &= ~(1<<channel);
                        robot->msg_subog_seq_received |= 1<<channel;
                        memcpy(robot->subog_str,data,data[0]+1); //???dont' quiet understand as data[0] stores the seq.size()+1 already

                        printf("%d parent_side: %d type: %d channel: %d\n", robot->timestamp, robot->parent_side,robot->type,channel);

                        // if module has not yet entered a repair state
                        if(  robot->current_state != REPAIR && robot->current_state != LEADREPAIR )
                        {
                            robot->parent_side = channel;
                            robot->subog_str[robot->subog_str[0]] |= robot->type<<4;     // 4:5
                            robot->subog_str[robot->subog_str[0]] |= channel<<6;  // 5:6
                        }

                        int ind = (int)(((uint8_t*)data)[0])+1;	 // get heading index
                        robot->heading = ((uint8_t*)data)[ind];	 // get heading
                        robot->commander_IP = getFullIP(data[data[0] + 2]);
                        robot->commander_port = COMMANDER_PORT_BASE + (uint8_t)data[data[0]+3];
                        printf("%d My heading is: %d @ %d, commander_ip is: %s:%d\n",robot->timestamp,(int)robot->heading,ind, IPToString(robot->commander_IP), robot->commander_port);

                        printf("%d Sub-organism string received\n",robot->timestamp);
                        robot->PrintSubOGString(robot->subog_str);

                        ack_required = true;
                    }
                    else
                        printf("%d: message is not from my neighbour ip %s\n", robot->timestamp, IPToString(getFullIP(sender)));

                }
                break;
            case MSG_TYPE_SCORE_STRING:
                {
                    if(channel < SIDE_COUNT)
                    {
                        robot->msg_score_seq_expected &= ~(1<<channel);
                        robot->msg_score_seq_received |= 1<<channel;

                        memcpy(robot->subog_str,data,data[0]+1);
                        robot->best_score = data[(int)(data[0])+1];

                        std::cout << "Score received: " << (int) robot->best_score << std::endl;
                        robot->PrintSubOGString(robot->subog_str);
                        
                        ack_required = true;
                    }
                    else
                        printf("%d: message is not from my neighbour ip %s\n", robot->timestamp, IPToString(getFullIP(sender)));
                }
                break;
            case IPC_MSG_HINGE_3D_MOTION_REQ:
                {
                    //followed by speed, count, rotation, angle, [int, int, int, int]
                    memcpy((uint8_t*)robot->hinge_command, data, sizeof(robot->hinge_command));
                    robot->timestamp_hinge_motor_cmd_received  = robot->timestamp;
                    ack_required = true;
                }
                break;
            case IPC_MSG_LOCOMOTION_2D_REQ:
                {
                    //followed by speed[0], speed[1], speed[2]
                    memcpy((uint8_t*)robot->locomotion_command, data, sizeof(robot->locomotion_command));
                    robot->timestamp_locomotion_motors_cmd_received  = robot->timestamp;
                    ack_required = true;
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
                        int8_t angle = data[0];
                        robot->RotateDockingUnit(channel, angle);
                        ack_required = true;
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
                    ack_required = true;
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
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
                        robot->msg_raising_start_received = true;
                }
                break;
            case IPC_MSG_RAISING_STOP:
                {
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
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
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
                        robot->msg_climbing_start_received = true;
                }
                break;
            case IPC_MSG_CLIMBING_STOP:
                {
                    if(sender != ((robot->my_IP.i32 >> 24)& 0xFF))
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
                    robot->msg_subog_seq_expected |= (1<<robot->parent_side);
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
                        std::cout<<robot->ClockString()<<": receive new target tree: "<<robot->mytree<<std::endl;
                        robot->parent_side = SIDE_COUNT;
                    }
                    printf("%d: reshaping start, new seed @ %s\n", robot->timestamp, IPToString(robot->commander_IP));
                }
                break;
            case IPC_MSG_RESHAPING_DONE:
                {
                    robot->msg_reshaping_done_received = true;
                }
                break;
            case MSG_TYPE_PROPAGATED:
                {
                    bool valid = true;
                    int data_size=0;
                    switch(data[0])
                    {
                        case MSG_TYPE_RETREAT:
                            {
                                robot->msg_retreat_received = true;
                                // Robot no longer needs to send sub_og_string
                                robot->RemoveFromQueue(channel,MSG_TYPE_SUB_OG_STRING, 0);
                                CPrintf1(SCR_GREEN,"%d -- retreating !", robot->timestamp);
                            }
                            break;
                        case MSG_TYPE_STOP:
                            {
                                robot->msg_stop_received = true;
                                CPrintf1(SCR_GREEN,"%d -- stopping !", robot->timestamp);
                            }
                            break;

                        case MSG_TYPE_RESHAPING_SCORE:
                            {
                                robot->msg_reshaping_score_received |= 1<<channel;
                                data_size = 0;
                                CPrintf1(SCR_GREEN,"%d -- reshaping score!", robot->timestamp);
                            }
                            break;
                        case MSG_TYPE_SCORE:
                            {
                                robot->msg_score_received |= 1<<channel;

                                robot->new_id[channel] = data[1];
                                robot->new_score[channel] = data[2];

                                CPrintf1(SCR_GREEN,"%d -- score !", robot->timestamp);
                                data_size = 2;
                                printf("%d: side %d(%#x) received id score %d %d\n",robot->timestamp,channel, robot->docked[channel],robot->new_id[channel],robot->new_score[channel]);
                            }
                            break;
                        default:
                            valid = false;
                            break;
                    }

                    if(valid)
                    {
                        robot->PropagateIRMessage(data[0], data+1, data_size, channel);
                        robot->IPCPropagateMessage(data[0], data+1, data_size, getFullIP(sender));
                        robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, (uint8_t*)&(data[0]), 1);
                    }
                }
                break;
            case IPC_MSG_ACK:
                {
                    //                    printf("%d: received [%s - %s]\n", robot->timestamp, message_names[msg->command], message_names[data[0]] );
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
                        case MSG_TYPE_FAILED:
                        case MSG_TYPE_SUB_OG_STRING:
                        case MSG_TYPE_SCORE_STRING:
                            robot->RemoveFromQueue(channel,data[0],MSG_TYPE_UNKNOWN);
                            break;
                        case MSG_TYPE_RETREAT:
                        case MSG_TYPE_STOP:
                        case MSG_TYPE_SCORE:
                        case MSG_TYPE_RESHAPING_SCORE:
                            CPrintf2(SCR_RED,"%d received message ack for %s\n", robot->timestamp, message_names[data[0]]);
                            robot->RemoveFromQueue(channel,MSG_TYPE_PROPAGATED, data[0]);
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

        if(ack_required)
        {
            robot->IPCSendMessage(getFullIP(sender).i32,IPC_MSG_ACK, (uint8_t*)&msg->command, 1);
        }

        /*
        if(msg->command == IPC_MSG_ACK)
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

void Robot::IPCPropagateMessage(uint8_t type, uint8_t *data, uint32_t size, Ethernet::IP sender)
{
    uint8_t buf[size+1];
    buf[0] = type;
    memcpy(buf+1, data, size);
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && neighbours_IP[i] != sender)
        {
            CPrintf2(SCR_RED,"%d propagate IPC message %s\n", timestamp, message_names[type]);
            IPCSendMessage(neighbours_IP[i].i32, MSG_TYPE_PROPAGATED, buf, size + 1);
        }
    }
}



