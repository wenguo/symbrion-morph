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

void Robot::ProcessEthMessage(std::auto_ptr<Message> msg)
{
    int size = msg.get()->GetDataLength();
    Ethernet::IP sender = msg.get()->GetSender();
    char * data = (char *)msg.get()->GetData();

    switch(data[0])
    {
        case ETH_MSG_TYPE_PROPAGATED:
            {
                PropagateEthMessage(data[1], (uint8_t*)&data[2], size-2, sender);
            }
            break;
        default:
            break;
    }

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
