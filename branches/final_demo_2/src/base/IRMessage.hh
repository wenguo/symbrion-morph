#ifndef IR_MESSAGE_HH
#define IR_MESSAGE_HH
#include <stdint.h>
#include <vector>
#include <iostream>

#define MAX_IR_MESSAGE_SIZE 20

class IRMessage
{
    public:
        IRMessage(uint8_t ch, uint32_t ts, uint8_t receiver, uint8_t type, const uint8_t * data, uint16_t data_len, uint8_t ack=0);
        ~IRMessage();

  //  private:
        uint8_t channel;
        uint8_t repeated;
        uint32_t timestamp;
        uint8_t ack_required;
        uint8_t receiver;
        uint8_t type;
        uint8_t data[MAX_IR_MESSAGE_SIZE-1];
        uint8_t data_len;
        friend std::ostream& operator<<(std::ostream&, const IRMessage&);
};

class EthMessage
{
    public:
        EthMessage(uint8_t ch, uint8_t type, const uint8_t * data, uint8_t data_len, bool ack=false):
            channel(ch),type(type), data(data, data+data_len),data_len(data_len), ack_required(ack){};
        ~EthMessage(){};

        uint8_t channel;
        uint8_t type;
        bool ack_required;

        std::vector<uint8_t> data;
        uint8_t data_len;
        friend std::ostream& operator<<(std::ostream&, const EthMessage&);
};



#endif
