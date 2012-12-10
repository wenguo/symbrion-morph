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
        for(int i=0;i<NUM_DOCKS;i++)
        {
            pthread_mutex_lock(&robot->ir_txqueue_mutex);
            if(robot->IR_TXMsgQueue[i].size()>0)
            {
                IRMessage &msg = robot->IR_TXMsgQueue[i].front();

                if(!msg.ack_required)
                {
                    //delay a second for acknowledgement
                    if(msg.type != IR_MSG_TYPE_ACK || robot->timestamp - msg.timestamp > robot->para.ir_msg_ack_delay)
                    {
                        robot->SendIRMessage(msg);
                        robot->IR_TXMsgQueue[i].erase(robot->IR_TXMsgQueue[i].begin());
                    }
                }
                else if(msg.repeated < robot->para.ir_msg_repeated_num)
                {
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
                            irmessage_names[msg.type], msg.repeated);
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
    /* printf("%d received message %d: ", channel, size);
       for(int i=0;i<size;i++)
       printf("%#x\t",data[i]);
       printf("\n");
       */
    bool ack_required = false;
    bool valid_message = true;

    //if received message has a specified receiver, but it doesn't match with mine, skip it
    if(data[0] !=0 && ((data[0] & 0xF) != ((docked[channel] >>4) & 0xF) ||  (data[0] >> 4 & 0xF) != ((docked[channel]) & 0xF)))
    {
        printf("%d channel %d received message %s, expected receiver is %#x(%#x) but I have %#x(%#x), skip it\n",
                timestamp, channel, irmessage_names[data[1]], data[0]&0xF, data[0], (docked[channel]>>4) & 0xF, docked[channel]);
        return;
    }

    //first byte indicates the msg type
    switch(data[1])
    {
        //broadcast, no ack required
        case IR_MSG_TYPE_RECRUITING:
            {
                if(current_state!=INORGANISM)
                {
                    comm_status[channel] = 1;

                    if (!organism_found)
                    {
                        organism_found = true;
                        assembly_count = DEFAULT_ASSEMBLY_COUNT;
                    }
                    assembly_info = OrganismSequence::Symbol(data[2]);
                }
            }
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
            {
                assembly_info_checked = true;
                assembly_info = OrganismSequence::Symbol(data[3]);
            }
            break;
        case IR_MSG_TYPE_ASSEMBLY_INFO_REQ:
            {
                OrganismSequence seq;
                rt_status ret = mytree.getBranch(seq, (robot_side)channel);
                if(ret.status == RT_OK)
                    SendIRMessage(channel, IR_MSG_TYPE_ASSEMBLY_INFO, &(seq.getSymbol(0).data), 1, true);
                else
                    printf("Error in getting branch, no Assembly info be sent\n");
            }
            break;
        case IR_MSG_TYPE_IP_ADDR:
            {
                OrganismSequence::Symbol sym = OrganismSequence::Symbol(data[2]);
                if(channel == sym.side1 && type == sym.type1)
                {
                    msg_ip_addr_received |= 1<<channel;
                    memcpy((uint8_t*)&neighbours_IP[channel], (uint8_t*)&data[3], 4);
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
                    SendIRMessage(channel, IR_MSG_TYPE_IP_ADDR, replied_data, 5, true);
                    memcpy((uint8_t*)&neighbours_IP[channel], (uint8_t*)&data[3], 4);
                }
            }
            break;
        case IR_MSG_TYPE_ACK:
            {
                std::vector<IRMessage>::iterator it=IR_TXMsgQueue[channel].begin();
                while(it!=IR_TXMsgQueue[channel].end())
                {
                    IRMessage& msg = *it;
                    if(msg.ack_required && msg.type == data[2]
                       && (msg.type != IR_MSG_TYPE_PROPAGATED ||
                       msg.data[0] == data[3]))
                        it = IR_TXMsgQueue[channel].erase(it);
                    else
                        ++it;
                }
            }
            break;
        case IR_MSG_TYPE_FAILED:
            {
                msg_failed_received |= 1<<channel;
                subog_id = data[2];
                parent_side = channel;
                heading = (parent_side + 2) % 4;
                ack_required = true;
            }
            break;
        case IR_MSG_TYPE_SUB_OG_STRING:
            {
                // only copy string when it is expected
                if( msg_subog_seq_expected & 1<<channel )
                {
                    msg_subog_seq_expected &= ~(1<<channel);
                    msg_subog_seq_received |= 1<<channel;
                    memcpy(subog_str,data+2,data[2]+1);

                    printf("%d parent_side: %d type: %d channel: %d\n", timestamp, parent_side,type,channel);
                    // if module has not yet entered a repair state
                    //if( parent_side >= SIDE_COUNT )
                    if(  current_state != REPAIR && current_state != LEADREPAIR )
                    {
                        parent_side = channel;
                        subog_str[subog_str[0]] |= type<<4;     // 4:5
                        subog_str[subog_str[0]] |= channel<<6;  // 5:6

            			int ind = (int)((uint8_t*)data)[0]+1;	 // get heading index
            			heading = (int)((uint8_t*)data)[ind];	 // get heading
                    }

                    printf("%d Sub-organism string received\n",timestamp);
                    PrintSubOGString(subog_str);
                }

                if( docked[channel] )
                    ack_required = true;
            }
            break;
        case IR_MSG_TYPE_SCORE_STRING:
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
        case IR_MSG_TYPE_SCORE:
            {
                msg_score_received |= 1<<channel;

                new_id[channel] = data[2];
                new_score[channel] = data[3];
                
                printf("%d Received id score %d %d\n",timestamp,new_id[channel],new_score[channel]);

                // only acknowledge messages sent by
                // other members of the sub-organism
                if( docked[channel] )
                    ack_required = true;
            }
            break;
        case IR_MSG_TYPE_RESHAPING:
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
        case IR_MSG_TYPE_PROPAGATED:
            {
                //data[1] IR_MSG_TYPE_PROPAGATED
                //data[2] Propagated message type
                //data[3 - 6] timestamp
                //data[7] data size
                //data[8-x] data
               // if(docked[channel])
                {
                    ack_required = true;
                    uint32_t ts = data[2] | data[3] << 8 | data[4] << 16 | data[5] << 24; //using 24bits for timestamp, 8bits for message type, so different message can be queued at the same time TODO: msg1, msg2, msg1 in queue with the same timestamp(real) will cause problems
                    //check if the same message received, using timestamp
                    if(ts != timestamp_propagated_msg_received)
                    {
                        timestamp_propagated_msg_received = ts;
                        bool valid=true;
                        switch(data[2])
                        {
                            case IR_MSG_TYPE_DISASSEMBLY:
                                {
                                    msg_disassembly_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- organism starts to disassemble !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_NEWROBOT_JOINED:
                                {
                                    num_robots_inorganism++;
                                    CPrintf1(SCR_GREEN,"%d -- new robot joined !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_ORGANISM_FORMED:
                                {
                                    organism_formed = true;
                                    CPrintf1(SCR_GREEN,"%d -- organism formed !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_RAISING:
                                {
                                    msg_raising_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to raise !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_LOWERING:
                                {
                                    msg_lowering_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to lower !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_RESHAPING:
                                {
                                    msg_reshaping_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to reshaping !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_RETREAT:
                            	{
                            		msg_retreat_received = true;
                                    CPrintf1(SCR_BLUE,"%d -- retreating !", timestamp);
                            	}
                            	break;
                            case IR_MSG_TYPE_STOP:
								{
									msg_stop_received = true;
                                    CPrintf1(SCR_RED,"%d -- stopping !", timestamp);
								}
								break;
                            default:
                                valid = false;
                                CPrintf1(SCR_GREEN, "%d -- received unknow message", timestamp);
                                break;
                        }

                        if(valid)
                            PropagateIRMessage(data[2], NULL, 0, channel);
                    }
                }
            }
            break;
        case IR_MSG_TYPE_UNKNOWN:
        default:
            valid_message = false;
            break;

    }

    if(valid_message)
    {
        printf("%d: channel %d docked %#x recevied message %s", timestamp, channel, data[0], irmessage_names[data[1]]);
        if(data[1]==IR_MSG_TYPE_ACK || data[1]==IR_MSG_TYPE_PROPAGATED)
            printf("(%s)\n", irmessage_names[data[2]]);
        else
            printf("\n");
    }

    //send acknowledgement
    if(ack_required)
    {
        if(data[1]==IR_MSG_TYPE_PROPAGATED)
        {
            printf("%d: channel %d Send ack %s (%s)\n", timestamp, channel, irmessage_names[data[1]], irmessage_names[data[2]]);
            SendIRAckMessage(channel, data[1], (uint8_t*)&(data[2]),1);
        }
        else
        {
            printf("%d: channel %d Send ack %s\n", timestamp, channel, irmessage_names[data[1]]);
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
    
    if(msg.type == IR_MSG_TYPE_PROPAGATED)
        printf("%d: %s send message %s (%s) via channel %d (%#x)\n",msg.timestamp, name, irmessage_names[msg.type],irmessage_names[msg.data[0]],msg.channel, board_dev_num[msg.channel]);
    else
        printf("%d: %s send message %s via channel %d (%#x)\n",msg.timestamp, name, irmessage_names[msg.type],msg.channel, board_dev_num[msg.channel]);
}

void Robot::SendIRMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{
    //all messages are pushed into a queue waiting to be processed by TxThread
    //This function should be called only when robots are docked, so docked[channel]!=0 TODO: check this
    //docked[channel] stores the encoded sequence information such as "KFSF", 'KF' is my type and my docking side
    //'SF' is the connected robot's type and side
    pthread_mutex_lock(&ir_txqueue_mutex);
    IR_TXMsgQueue[channel].push_back(IRMessage(channel, timestamp, docked[channel], type, data, size, ack_required));
    pthread_mutex_unlock(&ir_txqueue_mutex);
}

void Robot::SendIRMessage(int channel, uint8_t type, bool ack_required)
{
    SendIRMessage(channel, type, NULL, 0, ack_required);
}

void Robot::SendIRMessage(int channel, uint8_t type, const uint8_t data, bool ack_required)
{
    SendIRMessage(channel, type, &data, 1, ack_required);
}

void Robot::SendIRAckMessage(int channel, uint8_t type)
{
    SendIRMessage(channel, IR_MSG_TYPE_ACK, (const uint8_t*)&type, 1, false);
}

void Robot::SendIRAckMessage(int channel, uint8_t type, uint8_t *data, int size)
{
    uint8_t dst_data[size+1];
    dst_data[0] = type;
    memcpy(dst_data+1, data, size);
    SendIRMessage(channel, IR_MSG_TYPE_ACK, dst_data, size+1, false);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, bool ack_required)
{
    BroadcastIRMessage(channel, type, NULL, 0, ack_required);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t data, bool ack_required)
{
    BroadcastIRMessage(channel, type, &data, 1, ack_required);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{
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
        if(data_size > MAX_IR_MESSAGE_SIZE - 6 )
        {
            printf("Warning: only %d of %d bytes will be sent\n", MAX_IR_MESSAGE_SIZE - 5, data_size);
            data_size = MAX_IR_MESSAGE_SIZE - 6;
        }
        memcpy(buf + 5, data, data_size);
        SendIRMessage(channel, IR_MSG_TYPE_PROPAGATED, buf, data_size + 5, true);
    }
}

void Robot::PropagateIRMessage(uint8_t type, uint8_t *data, uint32_t len, int excluded_channel)
{
    for(uint8_t i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && i != excluded_channel)
        {
            PropagateSingleIRMessage( type, i, data, len );

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

    SendIRMessage(channel, IR_MSG_TYPE_FAILED, buf, 1, true);

    printf("%d Sending failure message %d %#x\n",timestamp,(int)buf[0],docked[channel]);
}

/*
 * If a module is present on side 'channel' it
 * is sent the current sub-organism string.
 */
void Robot::SendSubOrgStr( int channel, uint8_t *seq )
{
    if( channel < SIDE_COUNT && docked[channel] )
    {
        uint8_t buf[MAX_IR_MESSAGE_SIZE-1];

        buf[0] = seq[0]+1;

        if( buf[0] > MAX_IR_MESSAGE_SIZE-3 )
        {
            printf("Warning: only %d of %d bytes will be sent (SendSubOGStr)\n", MAX_IR_MESSAGE_SIZE-2, (int) buf[0] );
            buf[0] = MAX_IR_MESSAGE_SIZE - 3;
        }

        // copy previous string
        memcpy(buf+1,seq+1,seq[0]);

        // if sending string back to parent
        if( channel == parent_side )
        {
            // add zeros
            buf[buf[0]] = 0;
        }
        else
        {
            buf[buf[0]] = 0;
            buf[buf[0]] |= type;	    // 0:1
            buf[buf[0]] |= channel<<2;  // 2:3
        }

        // Send the direction that the neighbour should move in
        buf[buf[0]+1] = getNeighbourHeading( docked[channel] );

        SendIRMessage(channel, IR_MSG_TYPE_SUB_OG_STRING, buf, buf[0]+2, true);

        printf("%d Sending sub-og string\n",timestamp);
        PrintSubOGString(buf);
    }
}

void Robot::SendScoreStr( int channel, const OrganismSequence& seq, uint8_t score )
{

    if( channel < SIDE_COUNT && docked[channel] )
    {
        uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
        buf[0] = seq.Size();


        if( buf[0] > MAX_IR_MESSAGE_SIZE-2 )
        {
            printf("Warning: only %d of %d bytes will be sent (SendSubOGStr)\n", MAX_IR_MESSAGE_SIZE-2, (int) buf[0] );
            buf[0] = MAX_IR_MESSAGE_SIZE - 2;
        }

        for(unsigned int i=0; i < buf[0];i++)
            buf[i+1] =seq.Encoded_Seq()[i].data;

        buf[(int)(buf[0])+1] = score;


        std::cout << "sending score: " << (int) score << " and seq: " << seq << std::endl;
        SendIRMessage(channel, IR_MSG_TYPE_SCORE_STRING, buf, buf[0] + 2, true);
    }
}


void Robot::PropagateReshapeScore( uint8_t score, int ignore_side )
{
    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = score;

    for( int i=0; i<SIDE_COUNT; i++ )
    {
        if( docked[i] && i != ignore_side )
        {
            SendIRMessage(i, IR_MSG_TYPE_RESHAPING, buf, 1, true);
            printf("%d Propagating reshaping score: %d on side %d\n",timestamp, score,i);
        }
    }


}


void Robot::PropagateSubOrgScore( uint8_t id, uint8_t score, int ignore_side )
{

    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = id;
    buf[1] = score;

    for( int i=0; i<SIDE_COUNT; i++ )
    {
        if( docked[i] && i != ignore_side )
        {
            SendIRMessage(i, IR_MSG_TYPE_SCORE, buf, 2, true);
            printf("%d Propagating id score: %d %d on side %d\n",timestamp,id,score,i);
        }
    }

}
void Robot::BroadcastScore( int i, uint8_t score, uint8_t id )
{

    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
    buf[0] = id;
    buf[1] = score;

    printf("%d Broadcasting id score %d %d\n",timestamp,id,score);
    BroadcastIRMessage( i, IR_MSG_TYPE_SCORE, buf, 2);

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

    SendIRMessage(channel, IR_MSG_TYPE_ORGANISM_SEQ, buf, buf[0] + 1, true);
    std::cout<<timestamp<<": "<<name<<" send branch:"<<seq<<std::endl;

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
