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

// Given an IP address, return the channel
// at which the sending robot is docked
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
        case MSG_TYPE_SUB_OG_STRING:
            {
                // only copy string when it is expected
                if( msg_subog_seq_expected & 1<<channel )
                {
                    msg_subog_seq_expected &= ~(1<<channel);
                    msg_subog_seq_received |= 1<<channel;
                    memcpy(subog_str,data+1,data[1]+1);

                    //printf("%d parent_side: %d type: %d channel: %d\n", timestamp,parent_side,type,channel);

                    // if module has not yet entered a repair state
                    if(  current_state != REPAIR && current_state != LEADREPAIR )
                    {
                        parent_side = channel;
                        subog_str[subog_str[0]] |= type<<4;     // 4:5
                        subog_str[subog_str[0]] |= channel<<6;  // 5:6
                    }

                    int ind = (int)(((uint8_t*)data)[1])+2;	 // get heading index
                    heading = ((uint8_t*)data)[ind];	 // get heading
                    printf("%d: My heading is: %d @ %d\n",timestamp,(int)heading,ind);

                    printf("%d Sub-organism string received\n",timestamp);
                    PrintSubOGString(subog_str);

                    RemoveFromQueue(channel,MSG_TYPE_SUB_OG_STRING);
                }
            }
            break;
        case MSG_TYPE_SCORE_STRING:
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
        case MSG_TYPE_IP_ADDR_COLLECTION:
            {
                if(channel == parent_side)
                {
                    printf("%d: I shouldn't receive this message from my parent %d\n", timestamp, channel);
                }
                else
                {
                    //first fill in the ip_list in the right position of 'mytree' using received data
                    std::vector<uint8_t> branch_IPs;
                    printf("Eth received IPs: ");
                    for(int i=0;i<data[1];i++)
                    {
                        printf("%d ", data[i+2]);
                        branch_IPs.push_back(data[i+2]);
                    }
                    printf("\n");
                    mytree.setBranchIPs(robot_side(channel), branch_IPs);
                    
                    SendEthAckMessage(channel, data[0]);
                }
            }
            break;
        case MSG_TYPE_PROPAGATED:
            {
                uint32_t ts = data[2] | data[3] << 8 | data[4] << 16 | data[5] << 24; 

                if(ts != timestamp_propagated_msg_received)
                {
                    timestamp_propagated_msg_received = ts; //set the timestamp to prevent receive IR Propagated message at the same time;
                    bool valid = true;
                    switch( data[1] )
                    {
                        case MSG_TYPE_RETREAT:
                            {
                                msg_retreat_received = true;
                                CPrintf1(SCR_BLUE,"%d -- retreating !", timestamp);
                                // Robot no longer needs to send sub_og_string
                                RemoveFromQueue(channel,MSG_TYPE_SUB_OG_STRING);
                            }
                            break;
                        case MSG_TYPE_STOP:
                            {
                                msg_stop_received = true;
                                CPrintf1(SCR_RED,"%d -- stopping !", timestamp);
                            }
                            break;
                        case MSG_TYPE_RAISING:
                            {
                                msg_raising_received |= 1<<channel;
                                CPrintf1(SCR_GREEN,"%d -- start to raise !", timestamp);
                            }
                            break;
                        case MSG_TYPE_LOWERING:
                            {
                                msg_lowering_received |= 1<<channel;
                                CPrintf1(SCR_GREEN,"%d -- start to lower !", timestamp);
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
                        case MSG_TYPE_DISASSEMBLY:
                            {
                                msg_disassembly_received |= 1<<channel;
                                CPrintf1(SCR_GREEN,"%d -- organism starts to disassemble !", timestamp);
                            }
                            break;
                        case MSG_TYPE_RESHAPING:
                            {
                                msg_reshaping_received |= 1<<channel;
                                CPrintf1(SCR_GREEN,"%d -- start to reshaping !", timestamp);
                            }
                            break;
                        case MSG_TYPE_NEWROBOT_JOINED:
                            {
                                num_robots_inorganism++;
                                CPrintf1(SCR_BLUE,"%d -- new robot joined !", timestamp);
                            }
                            break;
                        case MSG_TYPE_ORGANISM_FORMED:
                            {
                                organism_formed = true;
                                rt_status ret = target.reBuild((uint8_t*)&(data[6]), size-8);
                                commander_IP = getFullIP(data[size - 2]);
                                commander_port = COMMANDER_PORT_BASE + (uint8_t)data[size - 1];
                                std::cout<<timestamp<<": "<<name<<" receive the whole tree:("<<size-6<<") "<<target<<std::endl;
                                std::cout<<"commander_IP: "<<IPToString(commander_IP)<<" port: "<<commander_port<<std::endl;
                                CPrintf1(SCR_BLUE,"%d -- organism formed !", timestamp);
                            }
                            break;
                        default:
                            valid = false;
                            CPrintf1(SCR_BLUE, "%d -- received unknown ETH message", timestamp);
                            break;
                    }

                    if( valid )
                    {
                        PropagateEthMessage(data[1], (uint8_t*)&data[6], size-6, sender);
                        PropagateIRMessage(data[1], (uint8_t*)&data[6], size-6, channel);
                        SendEthAckMessage(channel, data[1]);
                    }

                }
                else
                {
                    printf("ts is the same: %d %d\n", ts,timestamp_propagated_msg_received);
                }
            }
            break;
        case MSG_TYPE_ACK:
            {
                switch( data[1] )
                {
                    case MSG_TYPE_RETREAT:
                        break;
                    case MSG_TYPE_STOP:
                        break;
                    case MSG_TYPE_RAISING:
                        break;
                    case MSG_TYPE_RAISING_START:
                        break;
                    case MSG_TYPE_RAISING_STOP:
                        break;
                    case MSG_TYPE_DISASSEMBLY:
                        break;
                    case MSG_TYPE_NEWROBOT_JOINED:
                        CPrintf2(SCR_GREEN, "%d -- Removed MSG_TYPE_NEWROBOT_JOINED from channel %d", timestamp, channel);
                        break;
                    case MSG_TYPE_ORGANISM_FORMED:
                        CPrintf2(SCR_GREEN, "%d -- Removed MSG_TYPE_ORGANISM_FORMED from channel %d", timestamp, channel);
                        break;
                    case MSG_TYPE_IP_ADDR_COLLECTION:
                        CPrintf2(SCR_GREEN, "%d -- Removed MSG_TYPE_IP_ADDR_COLLECTION from channel %d", timestamp, channel);
                        RemoveFromQueue(channel,MSG_TYPE_IP_ADDR_COLLECTION);
                        break;
                    default:
                        break;
                }

                RemoveFromQueue(channel,MSG_TYPE_PROPAGATED, data[1]);

            }
            break;
        default:
            break;
    }

    printf("%d Ethernet message received: %s",timestamp,message_names[(int)data[0]]);

    if( data[0] == MSG_TYPE_PROPAGATED )
        printf("(%s)",message_names[(int)data[1]]);

    printf(" from %s\n",IPToString(sender));

}

void Robot::SendEthMessage(const EthMessage& msg)
{
    //IP not set, return directly
    if(neighbours_IP[msg.channel].i32 == 0)
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
    if(msg.type == MSG_TYPE_PROPAGATED)
    {
        printf("%d: %s send ETH message %s (%s) via channel %d (%s)\n",timestamp, name, message_names[msg.type],message_names[msg.data[0]],msg.channel,IPToString(neighbours_IP[msg.channel]));
    }
    else
        printf("%d: %s send ETH message %s via channel %d (%s) len: %d\n",timestamp, name, message_names[msg.type],msg.channel, IPToString(neighbours_IP[msg.channel]),msg.data_len);
}

void Robot::SendEthMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{
    if(channel >= NUM_DOCKS || channel <0)
        return;

    pthread_mutex_lock(&eth_txqueue_mutex);
    Eth_TXMsgQueue[channel].push_back(EthMessage(channel, type, data, size, ack_required));
    pthread_mutex_unlock(&eth_txqueue_mutex);
}

void Robot::PropagateEthMessage(uint8_t type, uint8_t *data, uint32_t size, Ethernet::IP sender)
{
    uint8_t buf[size+5];
    buf[0] = type;
    buf[1] = timestamp & 0xFF;
    buf[2] = (timestamp >>8) & 0xFF;
    buf[3] = (timestamp >>16) & 0xFF;
    buf[4] = (timestamp >>24) & 0xFF; //not used at moment
    memcpy(buf+5, data, size);
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(docked[i] && neighbours_IP[i] != sender)
            SendEthMessage(i, MSG_TYPE_PROPAGATED, buf, size + 5, false);
    }
}

void Robot::SendEthAckMessage(int channel, uint8_t type)
{
    SendEthMessage(channel, MSG_TYPE_ACK, (const uint8_t*)&type, 1, false);
}

void Robot::SendEthAckMessage(int channel, uint8_t type, uint8_t *data, int size)
{
    uint8_t dst_data[size+1];
    dst_data[0] = type;
    memcpy(dst_data+1, data, size);
    SendEthMessage(channel, MSG_TYPE_ACK, dst_data, size+1, false);
}
