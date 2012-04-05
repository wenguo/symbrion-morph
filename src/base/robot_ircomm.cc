#include <comm/IRComm.h>
#include "robot.hh"
#include "utils/support.hh"

void * Robot::IRCommRxThread(void * para)
{
    printf("IRCommRxThread is running.\n");
    Robot * robot = (Robot*) para;
    while(1)
    {
        pthread_mutex_lock(&robot->mutex);
        while(IRComm::HasMessage())
        {
            robot->ProcessIRMessage(IRComm::ReadMessage());
        }
        pthread_mutex_unlock(&robot->mutex);

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
            pthread_mutex_lock(&robot->txqueue_mutex);
            if(robot->TXMsgQueue[i].size()>0)
            {
                IRMessage &msg = robot->TXMsgQueue[i].front();

                if(!msg.ack_required)
                {
                    //delay a second for acknowledgement
                    if(msg.type != IR_MSG_TYPE_ACK || robot->timestamp - msg.timestamp > robot->para.ir_msg_ack_delay)
                    {
                        robot->IRSendMessage(msg);
                        robot->TXMsgQueue[i].erase(robot->TXMsgQueue[i].begin());
                    }
                }
                else if(msg.repeated < robot->para.ir_msg_repeated_num)
                {
                    if(robot->timestamp - msg.timestamp > robot->para.ir_msg_repeated_delay)
                    {
                        msg.repeated++;
                        robot->IRSendMessage(msg);
                        msg.timestamp = robot->timestamp;
                    }
                }
                else 
                {
                    printf("%d: Message %s has been repeated %d times, but no Ack received, remove it now!\n", robot->timestamp,
                            irmessage_names[msg.type], msg.repeated);
                    robot->TXMsgQueue[i].erase(robot->TXMsgQueue[i].begin());
                }
            }
            pthread_mutex_unlock(&robot->txqueue_mutex);

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

    //first byte indicates the msg type
    switch(data[0])
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
                    assembly_info = OrganismSequence::Symbol(data[1]);
                }
            }
            break;
        case IR_MSG_TYPE_EXPELLING:
            expelling_signals_detected |=1<<channel;
            break;
        case IR_MSG_TYPE_GUIDEME:
            msg_guideme_received |=1<<channel;
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
                        rt_status ret = mytree.reBuild((uint8_t*)&(data[3]), data[1]-2);

                        //fill branches sequence
                        if(ret.status < RT_ERROR)
                            ret=OrganismSequence::fillBranches(mytree, mybranches);

                        if(ret.status >= RT_ERROR)
                            printf("%d : Error in filling branches!\n", timestamp);

                        std::cout<<ClockString()<<": "<<name<<" receive new subtree: "<<mytree<<std::endl;
                    }
                }
            }
            break;
        case IR_MSG_TYPE_ASSEMBLY_INFO:
            {
                assembly_info_checked = true;
                assembly_info = OrganismSequence::Symbol(data[2]);
            }
            break;
        case IR_MSG_TYPE_ASSEMBLY_INFO_REQ:
            {
                OrganismSequence seq;
                rt_status ret = mytree.getBranch(seq, (robot_side)channel);
                if(ret.status == RT_OK)
                    BroadcastIRMessage(channel, IR_MSG_TYPE_ASSEMBLY_INFO, &(seq.getSymbol(0).data), 1, true);
                else
                    printf("Error in getting branch, no Assembly info be sent\n");
            }
            break;
        case IR_MSG_TYPE_IP_ADDR:
            {
                OrganismSequence::Symbol sym = OrganismSequence::Symbol(data[1]);
                if(channel == sym.side1 && type == sym.type1)
                {
                    neighbours_IP[channel] = data[2] << 24 | data[3] << 16 | data[4] << 8 |data[5];
                    //memcpy((uint8_t*)&neighbours_IP[channel], (uint8_t*)&data[2], 4);
                    printf("get neighbours_IP[%d]:%#x\n", channel, neighbours_IP[channel]);
                    ack_required = true;
                }
            }
            break;
        case IR_MSG_TYPE_IP_ADDR_REQ:
            {
                //REQ: data[1] -- child_side:child_type:parent_side:parent_type 
                //only respond to the valid request
                OrganismSequence::Symbol sym = OrganismSequence::Symbol(data[1]);
                if( channel == sym.side2 && type == sym.type2)
                {
                    uint8_t replied_data[5];
                    replied_data[0] = data[1];
                    replied_data[1] = (my_IP >> 24) & 0xFF;
                    replied_data[2] = (my_IP >> 16) & 0xFF;
                    replied_data[3] = (my_IP >> 8) & 0xFF;
                    replied_data[4] = my_IP & 0xFF;
                    //memcpy((uint8_t*)&replied_data[1], (uint8_t*)&my_IP, 4);
                    BroadcastIRMessage(channel, IR_MSG_TYPE_IP_ADDR, replied_data, 5, true);
                    
                    neighbours_IP[channel] = data[2] << 24 | data[3] << 16 | data[4] << 8 |data[5];
                    printf("request from neighbours_IP[%d]:%#x\n", channel, neighbours_IP[channel]);
                }

               // uint8_t data[5];
               // data[0] = channel;
               // memcpy(data+1, &my_IP, 4);
               // BroadcastIRMessage(channel, IR_MSG_TYPE_IP_ADDR, data, 5, true);
                
            }
            break;
        case IR_MSG_TYPE_ACK:
            {
                std::vector<IRMessage>::iterator it=TXMsgQueue[channel].begin();
                while(it!=TXMsgQueue[channel].end())
                {
                    IRMessage& msg = *it;
                    if(msg.ack_required && msg.type == data[1])
                        it = TXMsgQueue[channel].erase(it);
                    else
                        ++it;
                }
            }
            break;
        case IR_MSG_TYPE_FAILED:
			{
				msg_failed_received |= 1<<channel;
				subog_id = data[1];
				parent_side = channel;
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
					memcpy(subog_str,data+1,data[1]+1);

					// if module has not yet entered a repair state
					if( parent_side >= SIDE_COUNT )
					{
						parent_side = channel;
						subog_str[subog_str[0]] |= type<<4;     // 4:5
						subog_str[subog_str[0]] |= channel<<6;  // 5:6
					}

					printf("%d Sub-organism string received\n",timestamp);
					PrintSubOGString(subog_str);
				}

				ack_required = true;
            }
	    break;
        case IR_MSG_TYPE_SCORE_STRING:
			{
				if( msg_score_seq_expected & 1<<channel )
				{
					msg_score_seq_expected &= ~(1<<channel);
					msg_score_seq_received |= 1<<channel;

					// extract subog_str and best_score
					memcpy(subog_str,data+1,data[1]+1);
					best_score = data[(int)(data[1])+2];
			                	
                                        std::cout << "Score received: " << (int)best_score << std::endl;
                                        PrintSubOGString(subog_str);
                                        

                                          
                                }
				ack_required = true;
			}
        	break;
        case IR_MSG_TYPE_PROPAGATED:
            {
                //data[0] IR_MSG_TYPE_PROPAGATED
                //data[1] Propagated message type
                //data[2 - 5] timestamp
                //data[6] data size
                //data[7-x] data
                if(docked[channel])
                {
                    ack_required = true;
                    uint32_t ts = data[2] | data[3] << 8 | data[4] << 16 | data[5] << 24;
                    //check if the same message received, using timestamp
                    if(ts != timestamp_propagated_msg_received)
                    {
                        timestamp_propagated_msg_received = ts;
                        bool valid=true;
                        switch(data[1])
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
                            case IR_MSG_TYPE_TRANSFORMING:
                                {
                                    msg_transforming_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to transform !", timestamp);
                                }
                                break;
                            case IR_MSG_TYPE_RESHAPING:
                                {
                                    msg_reshaping_received |= 1<<channel;
                                    CPrintf1(SCR_GREEN,"%d -- start to reshaping !", timestamp);
                                }
                                break;
                            default:
                                valid = false;
                                CPrintf1(SCR_GREEN, "%d -- received unknow message", timestamp);
                                break;
                        }

                        if(valid)
                            PropagateIRMessage(data[1], NULL, 0, channel);
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
        printf("%d: channel %d recevied message %s %#x\n", timestamp, channel, irmessage_names[data[0]], data[1]);

    //send acknowledgement
    if(ack_required)
    {
        printf("Send ack %d %s\n", channel, irmessage_names[data[0]]);
        BroadcastIRAckMessage(channel, data[0]);
    }

    //flash led briefly
    if(RGBLED_status[channel] ==0)
    {
        SetRGBLED(channel, 0,0, GREEN, GREEN);
        RGBLED_flashing |=1<<channel;
    }


}

void Robot::IRSendMessage(const IRMessage& msg)
{
    uint8_t buf[MAX_IR_MESSAGE_SIZE];
    buf[0]=msg.data_len + 2;          //has to set the length explicityly for IR comm
    buf[1]=msg.type;
    for(int i=0;i<msg.data_len;i++)
        buf[i+2]=msg.data[i];
    IRComm::SendMessage(board_dev_num[msg.channel], buf, std::min(msg.data_len+2, MAX_IR_MESSAGE_SIZE));

    //flash led briefly
    if(RGBLED_status[msg.channel] ==0)
    {
        SetRGBLED(msg.channel, GREEN, GREEN, 0, 0);
        RGBLED_flashing |=1<<msg.channel;
    }

    printf("%d: %s send message %s via channel %d (%#x)\n",msg.timestamp, name, irmessage_names[msg.type],msg.channel, board_dev_num[msg.channel]);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, bool ack_required)
{
    BroadcastIRMessage(channel, type, NULL, 0, ack_required);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t data, bool ack_required)
{
    BroadcastIRMessage(channel, type, &data, 1, ack_required);
}

void Robot::BroadcastIRAckMessage(int channel, uint8_t type)
{
    BroadcastIRMessage(channel, IR_MSG_TYPE_ACK, (const uint8_t*)&type, 1, false);
}

void Robot::BroadcastIRMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{
    pthread_mutex_lock(&txqueue_mutex);
    TXMsgQueue[channel].push_back(IRMessage(channel, timestamp, type, data, size, ack_required));
    pthread_mutex_unlock(&txqueue_mutex);
}

void Robot::PropagateIRMessage(uint8_t type, uint8_t *data, uint32_t len, int excluded_channel)
{
    for(uint8_t i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && i != excluded_channel)
        {
            uint8_t buf[MAX_IR_MESSAGE_SIZE-1];
            buf[0] = type;
            buf[1] = timestamp;
            buf[2] = timestamp;
            buf[3] = timestamp;
            buf[4] = timestamp;
            int data_size = len;
            if(data_size > MAX_IR_MESSAGE_SIZE - 6 )
            {
                printf("Warning: only %d of %d bytes will be sent\n", MAX_IR_MESSAGE_SIZE - 5, data_size);
                data_size = MAX_IR_MESSAGE_SIZE - 6;
            }
            memcpy(buf + 5, data, data_size);
            BroadcastIRMessage(i, IR_MSG_TYPE_PROPAGATED, buf, data_size + 5, true);
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

    BroadcastIRMessage(channel, IR_MSG_TYPE_FAILED, buf, 1, true);

    printf("%d Sending failure message %d\n",timestamp,(int)buf[0]);
}

/*
 * If a module is present on side 'channel' it
 * is sent the current sub-organism string.
 */
void Robot::SendSubOGStr( int channel, uint8_t *seq )
{
	if( channel < SIDE_COUNT && docked[channel] )
	{
	    uint8_t buf[MAX_IR_MESSAGE_SIZE-1];

		buf[0] = seq[0]+1;

	    if( buf[0] > MAX_IR_MESSAGE_SIZE-2 )
	    {
	        printf("Warning: only %d of %d bytes will be sent (SendSubOGStr)\n", MAX_IR_MESSAGE_SIZE-2, (int) buf[0] );
	        buf[0] = MAX_IR_MESSAGE_SIZE - 2;
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

	    BroadcastIRMessage(channel, IR_MSG_TYPE_SUB_OG_STRING, buf, buf[0]+1, true);

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

            BroadcastIRMessage(channel, IR_MSG_TYPE_SCORE_STRING, buf, buf[0] + 2, true);

	}

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

    BroadcastIRMessage(channel, IR_MSG_TYPE_ORGANISM_SEQ, buf, buf[0] + 1, true);
    std::cout<<timestamp<<": "<<name<<" send branch:"<<seq<<std::endl;

}

bool Robot::MessageWaitingAck(int channel, uint8_t type)
{
    std::vector<IRMessage>::iterator it = TXMsgQueue[channel].begin();
    while(it!=TXMsgQueue[channel].end())
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
