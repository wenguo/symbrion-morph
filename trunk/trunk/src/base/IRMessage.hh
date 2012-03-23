#ifndef IR_MESSAGE_HH
#define IR_MESSAGE_HH
#include <stdint.h>

#define MAX_IR_MESSAGE_SIZE 20

class IRMessage
{
    public:
        IRMessage(uint8_t ch, uint32_t ts, uint8_t type, const uint8_t * data, uint16_t data_len, bool ack=false);
        ~IRMessage();

  //  private:
        uint8_t channel;
        uint8_t repeated;
        uint32_t timestamp;
        bool ack_required;
        uint8_t type;
        uint8_t data[MAX_IR_MESSAGE_SIZE-1];
        uint8_t data_len;
};

#endif
