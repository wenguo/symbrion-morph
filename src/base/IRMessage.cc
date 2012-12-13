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


