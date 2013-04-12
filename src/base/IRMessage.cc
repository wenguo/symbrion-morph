#include "IRMessage.hh"
#include <string.h>
#include <stdio.h>

IRMessage::IRMessage(uint8_t ch, uint32_t ts, uint8_t r, uint8_t t, const uint8_t * d, uint16_t len, uint8_t ack)
{
    channel = ch;
    repeated = 0;
    timestamp = ts;
    receiver = r;
    type = t;
    ack_required = ack;
    data_len = len;
    if(data_len > MAX_IR_MESSAGE_SIZE-1)
        data_len = MAX_IR_MESSAGE_SIZE-1;
    if(data_len > 0)
        memcpy(data, d, data_len);
}

IRMessage::~IRMessage()
{
    //    printf("Deconstruct IRMessage\n");
}


std::ostream& operator<<(std::ostream& os, const EthMessage& msg)
{
    os <<"Type: "<< (int)msg.type <<std::endl;;
    os <<"channel: "<< (int)msg.channel <<std::endl;;
    os <<"data: ";
    for(int i=0;i<msg.data_len;i++)
        os <<(int)msg.data[i]<<" ";
    os<<std::endl;

    return os;
}
