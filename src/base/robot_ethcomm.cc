#include "robot.hh"


void * Robot::EthCommRxThread(void * para)
{
    printf("EthCommRxThread is running.\n");
    Robot * robot = (Robot*) para;
    
    while(1)
    {
        //TODO: mutex seems not required?
        pthread_mutex_lock(&robot->eth_rx_mutex);
        while(Ethernet::HasMessage())
        {
            robot->ProcessEthMessage(Ethernet::ReadMessage());
        }
        pthread_mutex_unlock(&robot->eth_rx_mutex);

        usleep(20000);
    }
    
    printf("EthCommThread is quiting.\n");
    return NULL;
}

void * Robot::EthCommTxThread(void * para)
{
    printf("IRCommTxThread is running.\n");
    Robot * robot = (Robot*) para;
    while(1)
    {
        for(int i=0;i<NUM_DOCKS;i++)
        {
            pthread_mutex_lock(&robot->eth_txqueue_mutex);
            if(robot->Eth_TXMsgQueue[i].size()>0)
            {
                EthMessage &msg = robot->Eth_TXMsgQueue[i].front();

                if(!msg.ack_required)
                {
                        robot->SendEthMessage(msg);
                        robot->Eth_TXMsgQueue[i].erase(robot->Eth_TXMsgQueue[i].begin());
                }
            }
            pthread_mutex_unlock(&robot->eth_txqueue_mutex);

        }
        usleep(20000);
    }
    printf("EthCommThread is quiting.\n");
    return NULL;
}

// Given an IP address, get the channel
// through which it was delivered.
uint8_t Robot::getEthChannel(Ethernet::IP ip)
{
	for( int i=0; i<SIDE_COUNT; i++ )
	{
		if( neighbours_IP[i] == ip )
			return i;
	}

	// If message originated from an unknown (non-neighbouring) IP
	return SIDE_COUNT+1;
}


void Robot::ProcessEthMessage(std::auto_ptr<Message> msg)
{
    int size = msg.get()->GetDataLength();
    Ethernet::IP sender = msg.get()->GetSender();
    char * data = (char *)msg.get()->GetData();

    if( sender == 0 )
    	return;

    uint8_t channel = getEthChannel(sender);

    switch(data[0])
    {
    	// Note that the indexes are slightly different for ETH than for IR
    	case ETH_MSG_TYPE_SUB_OG_STRING:
			{
				// only copy string when it is expected
				if( msg_subog_seq_expected & 1<<channel )
				{
					msg_subog_seq_expected &= ~(1<<channel);
					msg_subog_seq_received |= 1<<channel;
					memcpy(subog_str,data+1,data[1]+1);

					printf("%d parent_side: %d type: %d channel: %d\n", timestamp,parent_side,type,channel);
					// if module has not yet entered a repair state
					//if( parent_side >= SIDE_COUNT )
					if(  current_state != REPAIR && current_state != LEADREPAIR )
					{
						parent_side = channel;
						subog_str[subog_str[0]] |= type<<4;     // 4:5
						subog_str[subog_str[0]] |= channel<<6;  // 5:6

						int ind = (int)((uint8_t*)data)[1]+1;	 // get heading index
						heading = (int)((uint8_t*)data)[ind];	 // get heading
						printf("%d My heading is: %d\n",timestamp,heading);
					}

					printf("%d Sub-organism string received\n",timestamp);
					PrintSubOGString(subog_str);

					RemoveFromQueue(channel,IR_MSG_TYPE_SUB_OG_STRING);
				}
			}
			break;
    	case ETH_MSG_TYPE_SCORE_STRING:
    		{
    			if( msg_score_seq_expected & 1<<channel )
				{
					msg_score_seq_expected &= ~(1<<channel);
					msg_score_seq_received |= 1<<channel;

					memcpy(subog_str,data+1,data[1]+1);
					best_score = data[(int)(data[1])+2];

					std::cout << "Score received: " << (int) best_score << std::endl;
					PrintSubOGString(subog_str);
				}
    		}
    		break;
        case ETH_MSG_TYPE_PROPAGATED:
            {
            	bool valid = true;
            	switch( data[1] )
            	{
            	case ETH_MSG_TYPE_RETREAT:
					{
						msg_retreat_received = true;
						CPrintf1(SCR_BLUE,"%d -- retreating !", timestamp);

						// Robot no longer needs to send sub_og_string
						RemoveFromQueue(channel,IR_MSG_TYPE_SUB_OG_STRING);
					}
            		break;
            	case ETH_MSG_TYPE_STOP:
					{
						msg_stop_received = true;
						CPrintf1(SCR_RED,"%d -- stopping !", timestamp);
					}
					break;
            	default:
            		valid = false;
            		CPrintf1(SCR_GREEN, "%d -- received unknown ETH message", timestamp);
            		break;
            	}

            	if( valid )
            		PropagateEthMessage(data[1], (uint8_t*)&data[2], size-2, sender);
            }
            break;
        default:
            break;
    }

    printf("%d Ethernet message received: %s",timestamp,ethmessage_names[(int)data[0]]);

    if( data[0] == ETH_MSG_TYPE_PROPAGATED )
    	printf("(%s)",ethmessage_names[(int)data[1]]);

    printf(" from %s\n",Ethernet::IPToString(sender));

}

void Robot::SendEthMessage(const EthMessage& msg)
{
    //IP not set, return directly
    if(neighbours_IP[msg.channel] == 0)
        return;

    uint8_t buf[msg.data_len+1];
    buf[0]=msg.type;
    for(int i=0;i<msg.data_len;i++)
        buf[i+1]=msg.data[i];
    Ethernet::SendMessage(neighbours_IP[msg.channel], buf, msg.data_len+1);

    //flash led briefly
    if(RGBLED_status[msg.channel] ==0)
    {
        SetRGBLED(msg.channel, RED, RED, 0, 0);
        RGBLED_flashing |=1<<msg.channel;
    }

    if(msg.type == ETH_MSG_TYPE_PROPAGATED)
    	printf("%d: %s send ETH message %s (%s) via channel %d (%s)\n",timestamp, name, ethmessage_names[msg.type],ethmessage_names[msg.data[0]],msg.channel,Ethernet::IPToString(neighbours_IP[msg.channel]));
    else
    	printf("%d: %s send ETH message %s via channel %d (%s) len: %d\n",timestamp, name, ethmessage_names[msg.type],msg.channel, Ethernet::IPToString(neighbours_IP[msg.channel]),msg.data_len);
 }

void Robot::SendEthMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{
    pthread_mutex_lock(&eth_txqueue_mutex);
    Eth_TXMsgQueue[channel].push_back(EthMessage(channel, type, data, size, ack_required));
    pthread_mutex_unlock(&eth_txqueue_mutex);
}

void Robot::PropagateEthMessage(uint8_t type, uint8_t *data, uint32_t size, Ethernet::IP sender)
{
    uint8_t buf[size+1];
    buf[0]=type;
    memcpy(buf+1, data, size);
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && neighbours_IP[i] != sender)
            SendEthMessage(i, ETH_MSG_TYPE_PROPAGATED, buf, size+1, false);
    }
}
