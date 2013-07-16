#include <comm/IRComm.h>
#include "robot.hh"
#include "utils/support.hh"

void * Robot::IRCommRxThread(void * para)
{
    printf("IRCommRxThread is running.\n");
    Robot * robot = (Robot*) para;
    while(1)
    {
        pthread_mutex_lock(&robot->ir_rx_mutex);
        while(IRComm::HasMessage())
        {
            robot->ProcessIRMessage(IRComm::ReadMessage());
        }
        pthread_mutex_unlock(&robot->ir_rx_mutex);

        usleep(20000);
    }
    printf("IRCommThread is quiting.\n");
    return NULL;
}

void * Robot::IRCommTxThread(void * para)
{
    printf("IRCommTxThread is running.\n");
    Robot * robot = (Robot*) para;
    while(1)
    {
        for(unsigned int i=0;i<NUM_DOCKS;i++)
        {
            pthread_mutex_lock(&robot->ir_txqueue_mutex);
            if(robot->IR_TXMsgQueue[i].size()>0)
            {
                IRMessage &msg = robot->IR_TXMsgQueue[i].front();

                if(msg.ack_required==0)
                {
                    if(msg.type == IR_MSG_TYPE_DOCKING_SIGNALS_REQ)
                    {
                        if(((robot->timestamp - msg.timestamp) % 10) == 2 * i)
                        {
                            robot->SendIRMessage(msg);
                            robot->IR_TXMsgQueue[i].erase(robot->IR_TXMsgQueue[i].begin());
                        }
                    }
                    else if(msg.type != MSG_TYPE_ACK || robot->timestamp - msg.timestamp > robot->para.ir_msg_ack_delay)
                    {
                        robot->SendIRMessage(msg);
                        robot->IR_TXMsgQueue[i].erase(robot->IR_TXMsgQueue[i].begin());
                    }
                }
                else if(msg.repeated < msg.ack_required)//robot->para.ir_msg_repeated_num)
                {
                    //delay a second for acknowledgement
                    if(robot->timestamp - msg.timestamp > robot->para.ir_msg_repeated_delay)
                    {
                        msg.repeated++;
                        robot->SendIRMessage(msg);
                        msg.timestamp = robot->timestamp;
                    }
                }
                else 
                {
                    printf("%d: Message %s has been repeated %d times, but no Ack received, remove it now!\n", robot->timestamp,
                            message_names[msg.type], msg.repeated);
                    robot->IR_TXMsgQueue[i].erase(robot->IR_TXMsgQueue[i].begin());
                }
            }
            pthread_mutex_unlock(&robot->ir_txqueue_mutex);

        }
        usleep(20000);
    }
    printf("IRCommThread is quiting.\n");
    return NULL;
}


void Robot::ProcessIRMessage(std::auto_ptr<Message> msg)
{
    int size = msg.get()->GetDataLength();
    uint8_t channel = robot_side_dev_num[msg.get()->GetSender()];
    char * data = (char *)msg.get()->GetData();
    // printf("%d received message %d: ", channel, size);
    //   for(int i=0;i<size;i++)
    //   printf("%#x\t",data[i]);
    //   printf("\n");
    bool ack_required = false;
    bool valid_message = true;

    //if received message has a specified receiver, but it doesn't match with mine, skip it
    if(data[0] !=0 && ((data[0] & 0xF) != ((docked[channel] >>4) & 0xF) ||  (data[0] >> 4 & 0xF) != ((docked[channel]) & 0xF)))
    {
        printf("%d channel %d received message %s, expected receiver is %#x(%#x) but I have %#x(%#x), skip it\n",
                timestamp, channel, message_names[data[1]], data[0]&0xF, (uint8_t)data[0], (docked[channel]>>4) & 0xF, docked[channel]);
        return;
    }

    //first byte indicates the msg type
    switch(data[1])
    {
        case IR_MSG_TYPE_REMOTE_DEBUG:
            RemoteDebugging(data+2);
            break;
        //broadcast, no ack required
        case IR_MSG_TYPE_RECRUITING:
            {
                OrganismSequence::Symbol sym(data[2]);
                if(current_state==FORAGING || current_state == WAITING || current_state == ASSEMBLY)
                {
                    //if(sym.type2 == type)
                    {
                        comm_status[channel] = 1;

                        if (!organism_found)
                        {
                            organism_found = true;
                            assembly_count = 0;
                        }

                        if(assembly_info!=0 && assembly_info!=sym)
                            printf("Received conflicting assembly info, may need to check !!!\n");

                        assembly_info = sym;
                        printf("assembly_info: %c%c\n", robottype_names[assembly_info.type2], side_names[assembly_info.side2]);
                    }
                }
                else if(current_state == LOCATEBEACON && sym.type2 == type)
                {
                    BroadcastIRMessage(channel, IR_MSG_TYPE_DOCKING_SIGNALS_REQ, 0);
                }
            }
            break;
        case IR_MSG_TYPE_RECRUITING_REQ:
            msg_recruiting_req_received |= 1<<channel;
            break;
        case IR_MSG_TYPE_EXPELLING:
            expelling_signals_detected |=1<<channel;
            break;
        case IR_MSG_TYPE_GUIDEME:
            msg_guideme_received |=1<<channel;
            break;
        case IR_MSG_TYPE_DOCKING_SIGNALS_REQ:
            msg_docking_signal_req_received |=1<<channel;
            break;
        case IR_MSG_TYPE_POWERSOURCE_FOUND:
            break;
        case IR_MSG_TYPE_OBJECTTYPE: 
            break;

            //broadcast, ack required
        case IR_MSG_TYPE_LOCKED:
            if(msg_locked_expected & 1<<channel)
            {
                msg_locked_expected &= ~(1<<channel);
                msg_locked_received |= 1<<channel;
                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_UNLOCKED:
            if(msg_unlocked_expected & 1<< channel)
            {
                msg_unlocked_expected &= ~(1<<channel);
                msg_unlocked_received |= 1<<channel;
                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_LOCKME:
            if(msg_lockme_expected & 1<< channel)
            {
                msg_lockme_expected &= ~(1<<channel);
                msg_lockme_received |= 1<<channel;
                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_UNLOCKME:
            if(msg_unlockme_expected & 1<< channel)
            {
                msg_unlockme_expected &= ~(1<<channel);
                msg_unlockme_received |=1<<channel;
                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_ORGANISM_SEQ:
            {
                if(channel == assembly_info.side2)
                {
                    ack_required = true;
                    //only rebuild organism info when it is expected
                    if(msg_organism_seq_expected)
                    {
                        msg_organism_seq_received = true;
                        //fill mytree, removing the parental info
                        rt_status ret = mytree.reBuild((uint8_t*)&(data[4]), data[2]-2);

                        //fill branches sequence
                        if(ret.status < RT_ERROR)
                            ret=OrganismSequence::fillBranches(mytree, mybranches);

                        if(ret.status >= RT_ERROR)
                            printf("%d : Error in filling branches!\n", timestamp);

                        std::cout<<ClockString()<<": "<<name<<" receive new subtree: "<<mytree<<std::endl;

                        // for reshaping
                        parent_side = channel;
                        msg_subog_seq_expected |= 1<<channel;
                    }
                }
            }
            break;
        case IR_MSG_TYPE_ASSEMBLY_INFO:
            if(msg_assembly_info_expected & (1<< channel))
            {
                printf("%d received assemby_info %#x\n", timestamp, data[2]);
                msg_assembly_info_received |=1<<channel;
                //msg_assembly_info_expected &=~(1<<channel);
                OrganismSequence::Symbol sym(data[2]);
                if(sym.type2 != type || sym.side2 != assembly_info.side2)
                    assembly_info = OrganismSequence::Symbol(0);
                else
                    assembly_info = sym; //it is ok to update assembly_info, even they are not the same

                //get commander_IP of the seed
                commander_IP = getFullIP(data[3]);
                commander_port = COMMANDER_PORT_BASE + (uint8_t)data[4];
                std::cout<<"commander_IP: "<<IPToString(commander_IP)<<" port: "<<commander_port<<std::endl;

                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_ASSEMBLY_INFO_REQ:
            if(msg_assembly_info_req_expected & 1<< channel)
            {
                printf("%d received assemby_info_req \n", timestamp);
                msg_assembly_info_req_expected &= ~(1<<channel);
                msg_assembly_info_req_received |= 1<<channel;
                OrganismSequence seq;
                rt_status ret = mytree.getBranch(seq, (robot_side)channel);
                std::cout<<seq<<std::endl;
                printf("assembly_info %#x\n", seq.getSymbol(0).data);
                uint8_t data[3];
                data[0] = seq.getSymbol(0).data;
                data[1] = (commander_IP.i32 >> 24 ) & 0xFF;
                data[2] = (commander_port - COMMANDER_PORT_BASE) & 0xFF;
                if(ret.status == RT_OK)
                    SendIRMessage(channel, IR_MSG_TYPE_ASSEMBLY_INFO, data, 3, 5);
                else
                    printf("Error in getting branch, no Assembly info be sent\n");
            }
            break;
        case IR_MSG_TYPE_IP_ADDR:
            {
                OrganismSequence::Symbol sym = OrganismSequence::Symbol(data[2]);
                if(channel == sym.side1 && type == sym.type1)
                {
                    if(msg_ip_addr_expected & (1<<channel))
                    {
                        msg_ip_addr_expected &= ~(1<<channel);
                        msg_ip_addr_received |= 1<<channel;
                        memcpy((uint8_t*)&neighbours_IP[channel], (uint8_t*)&data[3], 4);
                    }
                    ack_required = true;
                }
            }
            break;
        case IR_MSG_TYPE_IP_ADDR_REQ:
            {
                //REQ: data[1] -- child_side:child_type:parent_side:parent_type 
                //only respond to the valid request
                OrganismSequence::Symbol sym = OrganismSequence::Symbol(data[2]);
                if( channel == sym.side2 && type == sym.type2)
                {
                    uint8_t replied_data[5];
                    replied_data[0] = data[2];
                    memcpy((uint8_t*)&replied_data[1], (uint8_t*)&my_IP, 4);
                    SendIRMessage(channel, IR_MSG_TYPE_IP_ADDR, replied_data, 5, para.ir_msg_repeated_num);
                    memcpy((uint8_t*)&neighbours_IP[channel], (uint8_t*)&data[3], 4);
                }
            }
            break;
        case MSG_TYPE_ACK:
            {
                std::vector<IRMessage>::iterator it=IR_TXMsgQueue[channel].begin();
                while(it!=IR_TXMsgQueue[channel].end())
                {
                    IRMessage& msg = *it;
                    if(msg.ack_required && msg.type == data[2]
                            && (msg.type != MSG_TYPE_PROPAGATED ||
                                msg.data[0] == data[3]))
                        it = IR_TXMsgQueue[channel].erase(it);
                    else
                        ++it;
                }
            }
            break;
        case MSG_TYPE_FAILED:
            {
                msg_failed_received |= 1<<channel;
                subog_id = data[2];
                parent_side = channel;
                heading = (parent_side + 2) % 4;
                ack_required = true;
            }
            break;
        case MSG_TYPE_SUB_OG_STRING:
            {
                // only copy string when it is expected
                if( msg_subog_seq_expected & 1<<channel )
                {
                    msg_subog_seq_expected &= ~(1<<channel);
                    msg_subog_seq_received |= 1<<channel;
                    memcpy(subog_str,data+2,data[2]+1);

                    //printf("%d parent_side: %d type: %d channel: %d\n", timestamp,parent_side,type,channel);
                    // if module has not yet entered a repair state
                    //if( parent_side >= SIDE_COUNT )
                    if(  current_state != REPAIR && current_state != LEADREPAIR )
                    {
                        parent_side = channel;
                        subog_str[subog_str[0]] |= type<<4;     // 4:5
                        subog_str[subog_str[0]] |= channel<<6;  // 5:6
                    }


                    int ind = (int)(((uint8_t*)data)[2])+3;	 // get heading index
                    heading = ((uint8_t*)data)[ind];	 	// get heading
                    commander_IP = getFullIP(data[data[2] + 4]);
                    commander_port = COMMANDER_PORT_BASE + (uint8_t)data[data[2]+5];
                    printf("%d My heading is: %d @ %d, commander_ip is: %s:%d\n",timestamp,(int)heading,ind, IPToString(commander_IP), commander_port);

                    printf("%d Sub-organism string received\n",timestamp);
                    PrintSubOGString(subog_str);
                }

                if( docked[channel] )
                    ack_required = true;
            }
            break;
        case MSG_TYPE_SCORE_STRING:
            {
                if( msg_score_seq_expected & 1<<channel )
                {
                    msg_score_seq_expected &= ~(1<<channel);
                    msg_score_seq_received |= 1<<channel;

                    memcpy(subog_str,data+2,data[2]+1);
                    best_score = data[(int)(data[2])+3];

                    std::cout << "Score received: " << (int) best_score << std::endl;
                    PrintSubOGString(subog_str);
                }

                if( docked[channel] )
                    ack_required = true;
            }
            break;
        case MSG_TYPE_SCORE:
            {
                msg_score_received |= 1<<channel;

                new_id[channel] = data[2];
                new_score[channel] = data[3];

                printf("%d: side %d(%#x) received id score %d %d\n",timestamp,channel, docked[channel],new_id[channel],new_score[channel]);

                // only acknowledge messages sent by
                // other members of the sub-organism
                if( docked[channel] )
                    ack_required = true;
            }
            break;
        case MSG_TYPE_RESHAPING:
            {
                if( msg_reshaping_expected & 1<<channel )
                {
                    msg_reshaping_expected &= ~(1<<channel);
                    msg_reshaping_received |= 1<<channel;

                    // if message received whilst in the repair state, expect
                    // it to contain the best_score value, used to determine
                    // which robot should become the next seed module.
                    if( current_state == REPAIR )
                    {
                        best_score = data[2];
                        if( best_score == own_score )
                        {
                            seed = true;
                            mytree = target;
                        }
                    }
                }

                if( docked[channel] )
                    ack_required = true;

            }
            break;
        case MSG_TYPE_IP_ADDR_COLLECTION:
            {
                if(docked[channel])
                {
                    if(channel == parent_side)
                    {
                        printf("%d: I shouldn't receive this message from my parent %d\n", timestamp, channel);
                    }
                    else
                    {
                        //first fill in the ip_list in the right position of 'mytree' using received data
                        std::vector<uint8_t> branch_IPs;
                        printf("IR received IPs (%d):", data[2]);
                        for(int i=0;i<data[2];i++)
                        {
                            printf("%#x ", data[i+3]);
                            branch_IPs.push_back(data[i+3]);
                        }
                        printf("\n");
                        if(branch_IPs.size() > 0)
                            mytree.setBranchIPs(robot_side(channel), branch_IPs);

                        ack_required = true;
                    }
                }
            }
            break;
        case MSG_TYPE_PROPAGATED:
            {
                //data[1] IR_MSG_TYPE_PROPAGATED
                //data[2] Propagated message type
                //data[3 - 6] timestamp
                //data[7] data size
                //data[8-x] data
                // if(docked[channel])
                {
                    ack_required = true;
                    uint32_t ts = data[3] | data[4] << 8 | data[5] << 16 | data[6] << 24; //using 24bits for timestamp, 8bits for message type, so different message can be queued at the same time TODO: msg1, msg2, msg1 in queue with the same timestamp(real) will cause problems
                    //check if the same message received, using timestamp
                    if(ts != timestamp_propagated_msg_received)
                    {
                        timestamp_propagated_msg_received = ts;
                        bool valid=true;
                        switch(data[2])
                        {
                            case MSG_TYPE_DISASSEMBLY:
                                {
                                    msg_disassembly_received = true;
                                    CPrintf1(SCR_GREEN,"%d -- organism starts to disassemble !", timestamp);
                                }
                                break;
                            case MSG_TYPE_NEWROBOT_JOINED:
                                {
                                    num_robots_inorganism++;
                                    CPrintf1(SCR_GREEN,"%d -- new robot joined !", timestamp);
                                }
                                break;
                            case MSG_TYPE_ORGANISM_FORMED:
                                {
                                    organism_formed = true;
                       //             commander_IP = getFullIP(data[8]);
                       //             commander_port = COMMANDER_PORT_BASE + (uint8_t)data[9];
                                    std::cout<<"commander_IP: "<<IPToString(commander_IP)<<" port: "<<commander_port<<std::endl;
                                    CPrintf1(SCR_GREEN,"%d -- organism formed !", timestamp);
                                }
                                break;
                            case MSG_TYPE_RAISING:
                                {
                                    msg_raising_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to raise !", timestamp);
                                }
                                break;
                            case MSG_TYPE_RAISING_START:
                                {
                                    msg_raising_start_received = true;
                                    CPrintf1(SCR_GREEN,"%d -- start to raise !", timestamp);
                                }
                                break;
                            case MSG_TYPE_RAISING_STOP:
                                {
                                    msg_raising_stop_received = true;;
                                    CPrintf1(SCR_RED,"%d -- stopping raise !", timestamp);
                                }
                                break;
                            case MSG_TYPE_LOWERING:
                                {
                                    msg_lowering_received = true;
                                    CPrintf1(SCR_GREEN,"%d -- start to lower !", timestamp);
                                }
                                break;
                            case MSG_TYPE_RESHAPING_SCORE:
                                {
                                    msg_reshaping_score_received |= 1<<channel;
                                }
                                break;
                            case MSG_TYPE_RETREAT:
                                {
                                    msg_retreat_received = true;
                                    CPrintf1(SCR_BLUE,"%d -- retreating !", timestamp);
                                }
                                break;
                            case MSG_TYPE_STOP:
                                {
                                    msg_stop_received = true;
                                    CPrintf1(SCR_RED,"%d -- stopping !", timestamp);
                                }
                                break;
                            case MSG_TYPE_SCORE:
                                {
                                    msg_score_received |= 1<<channel;

                                    new_id[channel] = data[8];
                                    new_score[channel] = data[9];

                                    printf("%d: side %d(%#x) received id score %d %d\n",timestamp,channel, docked[channel],new_id[channel],new_score[channel]);

                                }
                                break;

                            default:
                                valid = false;
                                CPrintf2(SCR_GREEN, "%d -- received unknow message %#x", timestamp, data[2]);
                                break;
                        }

                        if(valid)
                        {
                            printf("propagated message size: %d\n", size);
                            PropagateIRMessage(data[2], (uint8_t*)&data[8], data[7], channel);
                          //  PropagateEthMessage(data[2], (uint8_t*)&data[8], data[7], neighbours_IP[channel]);
                        }
                    }
                }
            }
            break;
        case MSG_TYPE_UNKNOWN:
        default:
            valid_message = false;
            break;

    }

    if(valid_message)
    {
        printf("%d: channel %d docked %#x [%#x] recevied message %s", timestamp, channel, docked[channel], data[0], message_names[data[1]]);
        if(data[1]==MSG_TYPE_ACK || data[1]==MSG_TYPE_PROPAGATED)
            printf("(%s)\n", message_names[data[2]]);
        else
            printf("\n");
    }

    //send acknowledgement
    if(ack_required)
    {
        if(data[1]==MSG_TYPE_PROPAGATED)
        {
            printf("%d: channel %d Send ack %s (%s)\n", timestamp, channel, message_names[data[1]], message_names[data[2]]);
            SendIRAckMessage(channel, data[1], (uint8_t*)&(data[2]),1);
        }
        else
        {
            printf("%d: channel %d Send ack %s\n", timestamp, channel, message_names[data[1]]);
            SendIRAckMessage(channel, data[1]);
        }
    }

    //flash led briefly
    if(RGBLED_status[channel] ==0)
    {
        SetRGBLED(channel, 0,0, GREEN, GREEN);
        RGBLED_flashing |=1<<channel;
    }


}

//don't call this function directly, it is used in IRCommTxThread only
void Robot::SendIRMessage(const IRMessage& msg)
{
    uint8_t buf[MAX_IR_MESSAGE_SIZE];
    buf[0]=msg.data_len + 3;          //has to set the length explicityly for IR comm, this will be ignored by receiver's MSP level code
    buf[1]=msg.receiver;
    buf[2]=msg.type;
    for(int i=0;i<msg.data_len;i++)
        buf[i+3]=msg.data[i];
    IRComm::SendMessage(board_dev_num[msg.channel], buf, std::min(msg.data_len+3, MAX_IR_MESSAGE_SIZE));

    //flash led briefly
    if(RGBLED_status[msg.channel] ==0)
    {
        SetRGBLED(msg.channel, GREEN, GREEN, 0, 0);
        RGBLED_flashing |=1<<msg.channel;
    }

    if(msg.type == MSG_TYPE_PROPAGATED || msg.type == MSG_TYPE_ACK)
        printf("%d-%d: %s send message %s (%s) via channel %d (%#x)\n",timestamp, msg.timestamp, name, message_names[msg.type],message_names[msg.data[0]],msg.channel, board_dev_num[msg.channel]);
    else
        printf("%d-%d: %s send message %s via channel %d (%#x)\n",timestamp, msg.timestamp, name, message_names[msg.type],msg.channel, board_dev_num[msg.channel]);
}

void Robot::SendIRMessage(int channel, uint8_t type, const uint8_t *data, int size, uint8_t ack_required)
{
    //all messages are pushed into a queue waiting to be processed by TxThread
    //This function should be called only when robots are docked, so docked[channel]!=0 TODO: check this
    //docked[channel] stores the encoded sequence information such as "KFSF", 'KF' is my type and my docking side
    //'SF' is the connected robot's type and side
    if(channel >= NUM_DOCKS || channel <0)
        return;

    pthread_mutex_lock(&ir_txqueue_mutex);
    if(type == MSG_TYPE_ACK)
        IR_TXMsgQueue[channel].insert(IR_TXMsgQueue[channel].begin(),IRMessage(channel, timestamp, docked[channel], type, data, size, ack_required));
    else    
        IR_TXMsgQueue[channel].push_back(IRMessage(channel, timestamp, docked[channel], type, data, size, ack_required));
    pthread_mutex_unlock(&ir_txqueue_mutex);
}

void Robot::SendIRMessage(int channel, uint8_t type, uint8_t ack_required)
{
    SendIRMessage(channel, type, NULL, 0, ack_required);
}

void Robot::SendIRMessage(int channel, uint8_t type, const uint8_t data, uint8_t ack_required)
{
    SendIRMessage(channel, type, &data, 1, ack_required);
}

void Robot::SendIRAckMessage(int channel, uint8_t type)
{
    SendIRMessage(channel, MSG_TYPE_ACK, (const uint8_t*)&type, 1, 0);
}

void Robot::SendIRAckMessage(int channel, uint8_t type, uint8_t *data, int size)
{
    uint8_t dst_data[size+1];
    dst_data[0] = type;
    memcpy(dst_data+1, data, size);
    SendIRMessage(channel, MSG_TYPE_ACK, dst_data, size+1, 0);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, uint8_t ack_required)
{
    BroadcastIRMessage(channel, type, NULL, 0, ack_required);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t data, uint8_t ack_required)
{
    BroadcastIRMessage(channel, type, &data, 1, ack_required);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t *data, int size, uint8_t ack_required)
{
    if(channel >= NUM_DOCKS || channel <0)
        return;

    //push broadcast message into a queue, the receiver is set to 0 to be distinguished from SendIRMessages
    pthread_mutex_lock(&ir_txqueue_mutex);
    IR_TXMsgQueue[channel].push_back(IRMessage(channel, timestamp, 0, type, data, size, ack_required));
    pthread_mutex_unlock(&ir_txqueue_mutex);
}

// Propagates IR message on single channel
void Robot::PropagateSingleIRMessage(uint8_t type, int channel, uint8_t *data, uint32_t len )
{
    if( docked[channel] )
    {
        uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
        buf[0] = type;
        buf[1] = timestamp & 0xFF;
        buf[2] = (timestamp >>8) & 0xFF;
        buf[3] = (timestamp >>16) & 0xFF;
        buf[4] = (timestamp >>24) & 0xFF; //not used at moment
        int data_size = len;
        buf[5] = data_size;
        if(data_size > MAX_IR_MESSAGE_SIZE - 7 )
        {
            printf("Warning: only %d of %d bytes will be sent\n", MAX_IR_MESSAGE_SIZE - 6, data_size);
            data_size = MAX_IR_MESSAGE_SIZE - 7;
        }
        memcpy(buf + 6, data, data_size);
        SendIRMessage(channel, MSG_TYPE_PROPAGATED, buf, data_size + 6, para.ir_msg_repeated_num);
    }
}

void Robot::PropagateIRMessage(uint8_t type, uint8_t *data, uint32_t len, int excluded_channel)
{
    for(uint8_t i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && i != excluded_channel)
        {
            PropagateSingleIRMessage( type, i, data, len );

            // TODO: remove, no longer needed
            // 			  MOVED TO 'PropagateSingleIRMessage'
            //
            //            uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
            //            buf[0] = type;
            //            buf[1] = timestamp;
            //            buf[2] = timestamp;
            //            buf[3] = timestamp;
            //            buf[4] = timestamp;
            //            int data_size = len;
            //            if(data_size > MAX_IR_MESSAGE_SIZE - 6 )
            //            {
            //                printf("Warning: only %d of %d bytes will be sent\n", MAX_IR_MESSAGE_SIZE - 5, data_size);
            //                data_size = MAX_IR_MESSAGE_SIZE - 6;
            //            }
            //            memcpy(buf + 5, data, data_size);
            //            SendIRMessage(i, IR_MSG_TYPE_PROPAGATED, buf, data_size + 5, true);
        }
    }
}

/*
 * Sends a message containing the side from which the
 * message was sent. Used to determine 'sub_og_id'.
 * At a later stage more information may be sent.
 */
void Robot::SendFailureMsg( int channel )
{
    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = channel;

    SendIRMessage(channel, MSG_TYPE_FAILED, buf, 1, para.ir_msg_repeated_num);
    IPCSendMessage(neighbours_IP[channel].i32, MSG_TYPE_FAILED, buf, 1);

    printf("%d Sending failure message %d %#x\n",timestamp,(int)buf[0],docked[channel]);
}







void Robot::PropagateReshapeScore( uint8_t score, int ignore_side )
{
    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = score;

    printf("%d Propagating reshaping score: %d\n",timestamp, score);
    PropagateIRMessage(MSG_TYPE_RESHAPING_SCORE,buf,1,ignore_side);
    IPCPropagateMessage(MSG_TYPE_RESHAPING_SCORE,buf,1,neighbours_IP[ignore_side]);


    //    for( int i=0; i<SIDE_COUNT; i++ )
    //    {
    //        if( docked[i] && i != ignore_side )
    //        {
    //            Pro(i, IR_MSG_TYPE_RESHAPING, buf, 1, true);
    //            printf("%d Propagating reshaping score: %d on side %d\n",timestamp, score,i);
    //        }
    //    }


}


void Robot::PropagateSubOrgScore( uint8_t id, uint8_t score, int ignore_side )
{

    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = id;
    buf[1] = score;

    PropagateIRMessage(MSG_TYPE_SCORE, buf, 2, ignore_side);
   IPCPropagateMessage(MSG_TYPE_SCORE,buf,2,neighbours_IP[ignore_side]);
    //IPCSendMessage(0, MSG_TYPE_SCORE, buf, 2);

    printf("%d Propagating id score: %d %d\n",timestamp,id,score);

    //    for( int i=0; i<SIDE_COUNT; i++ )
    //    {
    //        if( docked[i] && i != ignore_side )
    //        {
    //            SendIRMessage(i, IR_MSG_TYPE_SCORE, buf, 2, true);
    //            printf("%d Propagating id score: %d %d on side %d\n",timestamp,id,score,i);
    //        }
    //    }

}
void Robot::BroadcastScore( int i, uint8_t score, uint8_t id )
{

    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = id;
    buf[1] = score;

    printf("%d Broadcasting id score %d %d\n",timestamp,id,score);
    BroadcastIRMessage( i, MSG_TYPE_SCORE, buf, 2, 0);

}

void Robot::SendBranchTree(int channel, const OrganismSequence& seq)
{
    if(seq.Size()<=0)
        return;

    OrganismSequence::Symbol connection_info = seq.getSymbol(0);

    if(channel != (int) connection_info.side1)
    {
        std::cout<<"ERROR!!! sending branch tree ( "<<seq<<" ) in wrong channel" << channel<<std::endl;
        return;
    }

    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = seq.Size();

    if(seq.Size() > MAX_IR_MESSAGE_SIZE-2)
    {
        printf("Warning: only %d of %d bytes will be sent (SendBranchTree)\n", MAX_IR_MESSAGE_SIZE-2, seq.Size());
        buf[0] = MAX_IR_MESSAGE_SIZE - 2;
    }

    for(unsigned int i=0; i < buf[0];i++)
        buf[i+1] =seq.Encoded_Seq()[i].data;

    SendIRMessage(channel, IR_MSG_TYPE_ORGANISM_SEQ, buf, buf[0] + 1, para.ir_msg_repeated_num);
    std::cout<<timestamp<<": "<<name<<" send branch:"<<seq<<std::endl;

}

// Messages that have not been acknowledged can sometimes
// block the queue - if a message no longer requires ack-
// nowledgement, this method can remove it from the queue
void Robot::RemoveFromQueue(int channel, uint8_t type, uint8_t subtype)
{
    std::vector<IRMessage>::iterator it = IR_TXMsgQueue[channel].begin();
    pthread_mutex_lock(&ir_txqueue_mutex);
    while(it!=IR_TXMsgQueue[channel].end())
    {
        std::cout<<*it<<std::endl;
        if((*it).type == type && (subtype == MSG_TYPE_UNKNOWN || subtype == (*it).data[0]))
        {
            if((*it).type == MSG_TYPE_PROPAGATED)
                printf("%d Ack no longer needed for message %s [%s], removing it from queue.\n", timestamp, message_names[(*it).type], message_names[(*it).data[0]]);
            else
                printf("%d Ack no longer needed for message %s, removing it from queue.\n", timestamp, message_names[(*it).type]);
            it = IR_TXMsgQueue[channel].erase(it);
        }
        else
        {
            it++;
        }
    }
    pthread_mutex_unlock(&ir_txqueue_mutex);
}

void Robot::RemoveFromAllQueues( uint8_t type, uint8_t subtype)
{
    for(int i=0;i<NUM_DOCKS;i++)
    {
        RemoveFromQueue(i,type, subtype);
    }
}

bool Robot::MessageWaitingAck(int channel, uint8_t type)
{
    std::vector<IRMessage>::iterator it = IR_TXMsgQueue[channel].begin();
    while(it!=IR_TXMsgQueue[channel].end())
    {
        if((*it).type == type)
            return true;
        it++;
    }
    return false;
}
bool Robot::MessageWaitingAck(uint8_t type)
{
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(MessageWaitingAck(i, type))
            return true;
    }
    return false;
}