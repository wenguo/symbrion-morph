#include "robot.hh"

void Robot::CheckEthCommunication()
{

}

void Robot::EthSendMessage(int channel, uint8_t type, const uint8_t *data, int size, bool ack_required)
{

}

void *Robot::ProcessEthMessage()
{
}

void * Robot::EthCommRxThread(void * para)
{
    printf("EthCommRxThread is running.\n");
    Robot * robot = (Robot*) para;
    /*
    while(1)
    {
        pthread_mutex_lock(&robot->mutex);
        while(IRComm::HasMessage())
        {
        }
        pthread_mutex_unlock(&robot->mutex);

        usleep(20000);
    }
    */
    printf("EthCommThread is quiting.\n");
    return NULL;
}

void * Robot::EthCommTxThread(void * para)
{
    printf("IRCommTxThread is running.\n");
    Robot * robot = (Robot*) para;
    /*
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
                        robot->SendIRMessage(msg);
                        robot->TXMsgQueue[i].erase(robot->TXMsgQueue[i].begin());
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
                    robot->TXMsgQueue[i].erase(robot->TXMsgQueue[i].begin());
                }
            }
            pthread_mutex_unlock(&robot->txqueue_mutex);

        }
        usleep(20000);
    }*/
    printf("EthCommThread is quiting.\n");
    return NULL;
}


