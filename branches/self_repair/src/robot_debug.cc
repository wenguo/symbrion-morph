#include "robot.hh"

void Robot::LogRecruiting()
{

}

void Robot::LogDocking(uint8_t side, const char connection_sym[4])
{
    if(timestamp ==32)
    {
        OrganismSequence::Symbol sym;
        sym.reBuild(connection_sym);
        docked[side]=sym.data;
        //using reflective signals if not set
        for(int i=0; i< NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0 | IR_PULSE1);
    }

    //send synchronisation signals, using ip_req
    if(msg_ip_addr_received==0)
    {
        if(timestamp % 20 ==5)
        {
            uint8_t data[5];
            data[side] = docked[side]; //TODO: remove this as it is already included when using SendIRMessage
            memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP, 4);
            Robot::SendIRMessage(side, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, false);
        }
    }
    else
        clock++;

    if(clock >= para.debug.para[5])
    {
        Log();

        if(clock == para.debug.para[5])
        {
            leftspeed = -20;
            rightspeed = -20;
            sidespeed = 0;
        }
        else if(clock == para.debug.para[6])
        {
            leftspeed = 0;
            rightspeed = 0;
            sidespeed = 0;
        }
    }
    break;

}
