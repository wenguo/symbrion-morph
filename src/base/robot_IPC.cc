#include "robot.hh"
void Robot::Process_Organism_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || connection || !user_ptr)
        return;

    static int client_port = 0;

    Robot *robot = (Robot*)user_ptr;
    switch(msg->command)
    {       
        case  REQ_SUBSCRIPTION:
            {
                printf("received subscription request from: %s\n", msg->data);
                LolMessage msg;
                lolmsgInit(&msg,REQ_SUBSCRIPTION_ACK, (uint8_t*)&client_port, 4);
                int size = lolmsgSerializedSize(&msg);
                uint8_t buf[size];
                lolmsgSerialize(&msg, buf);

                //robot->subscription_IPC.SendData();

                client_port++;
            }
            break;
        case REQ_SUBSCRIPTION_ACK:
            {
                printf("I am a client, create a new monitoring ipc");
            }  
            break;   
        default:
            break;
    }
}
