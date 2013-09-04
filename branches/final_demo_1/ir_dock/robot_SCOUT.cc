#include "robot.hh"

void RobotSCOUT::Recruiting()
{
    speed[LEFTWHEEL] = 0;
    speed[RIGHTWHEEL] = 0;
    printf("\tScoutBot Recruiting\n");
    for(int i=0;i<NUM_DOCKS;i++)
    {
        switch(recruiting_status[i])
        {
            case STAGE1://emits docking signals
                if(msg_docking_signal_req_received & (1<<i))
                {
                    msg_docking_signal_req_received &= ~(1<<i);
                    SetIRLED(i, IRLEDDOCKING, ScoutBot::IRTOP, IRPULSE0 | IRPULSE1); //TODO: better to switch off ir pulse
                }
                else if(msg_guideme_received & (1<<i))
                {
                //    if(it1->getSymbol(0).type2 == ROBOT_SCOUT)
                //        msg_locked_expected |= 1<<i;
                    msg_guideme_received &= ~(1<<i);
                    recruiting_status[i]=STAGE2;
                    SetIRLED(i, IRLEDPROXIMITY, ScoutBot::IRLEFT|ScoutBot::IRRIGHT, 0); //switch docking signals 2 on left and right leds
                }
                break;
            case STAGE2:
                msg_guideme_received &= ~(1<<i);
                msg_lockme_expected |=1<<i;
                if((msg_lockme_received & (1<<i) || msg_locked_received &(1<<i)))
                {
                    recruiting_status[i]=STAGE3;
                    SetIRLED(i, IRLEDOFF, ScoutBot::IRLEFT|ScoutBot::IRRIGHT, 0);
                    irobot->SetIRRX(i, false);
                    printf("Recruitment: channel %d  switch to Stage%d\n\n", i, recruiting_status[i]);
                }

                //give up, back to stage 1
                if((msg_docking_signal_req_received & (1<<i)) || guiding_signals_count[i]++ >= 1000)
                {
                    msg_docking_signal_req_received &=~(1<<i);
                    guiding_signals_count[i] = 0;
                    recruiting_status[i]=STAGE1;
                    SetIRLED(i, IRLEDOFF, 0, 0);
                    irobot->SetIRRX(i, true);
                    printf("Recruitment: channel %d  waits too long, switch back to Stage%d\n\n", i, recruiting_status[i]);
                }
                break;
            case STAGE3:
                if(msg_lockme_received & (1<<i))
                {
                    msg_lockme_received &=~(1<<i);

                   // ((ScoutBot*)irobot)->CloseDocking(ScoutBot::Side(i));

                    recruiting_status[i]=STAGE4;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruiting_status[i]);
                }
                else if(msg_locked_received & (1<<i))
                {
                    msg_locked_received &=~(1<<i);
                    msg_locked_expected &=~(1<<i);
                    recruiting_status[i]=RECRUITING_DONE;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruiting_status[i]);
                }
                else if(msg_docking_signal_req_received & 1<<i)
                {
                    msg_docking_signal_req_received &=~(1<<i);
                    guiding_signals_count[i] = 0;
                    recruiting_status[i]=STAGE1;
                    SetIRLED(i, IRLEDOFF, 0, 0);
                    irobot->SetIRRX(i, true);
                    printf("%d -- Recruitment: channel %d  received docking signals req, switch back to Stage%d\n\n", timestamp,i, recruiting_status[i]);
                }
                else if(msg_guideme_received & (1<<i))
                {
                    msg_guideme_received &= ~(1<<i);
                    recruiting_status[i] = STAGE2;
                    SetIRLED(i, IRLEDPROXIMITY, ScoutBot::IRLEFT|ScoutBot::IRRIGHT, 0); //switch docking signals 2 on left and right leds
                    printf("%d -- Recruitment: channel %d  switch back to Stage%d\n\n", timestamp,i, recruiting_status[i]);
                }

                break;
            case STAGE4:
                //wait for lock motor closed
                    recruiting_status[i] = RECRUITING_DONE;

                break;
            default:
                break;
        }
    }
}

void RobotSCOUT::LocateBeacon()
{
    printf("\tScoutBot LocateBeaon\n");
}

void RobotSCOUT::Aligning()
{
    printf("\tScoutBot Aligning\n");
}

void RobotSCOUT::Locking()
{
    printf("\tScoutBot Locking\n");
}

void RobotSCOUT::UpdateSensors()
{
    printf("Scout Update Sensor\n");
    IRValues ret_A;// = irobot->GetIRValues(ScoutBot::FRONT);
    IRValues ret_B;// = irobot->GetIRValues(ScoutBot::LEFT);
    IRValues ret_C = irobot->GetIRValues(ScoutBot::REAR);
    IRValues ret_D;// = irobot->GetIRValues(ScoutBot::RIGHT);
    ambient[0] = ret_A.sensor[0].ambient;
    ambient[1] = ret_A.sensor[1].ambient;
    reflective[0] = ret_A.sensor[0].reflective;
    reflective[1] = ret_A.sensor[1].reflective;
    proximity[0] = ret_A.sensor[0].proximity;
    proximity[1] = ret_A.sensor[1].proximity;
    beacon[0] = ret_A.sensor[0].docking;
    beacon[1] = ret_A.sensor[1].docking;
    ambient[3] = ret_B.sensor[0].ambient;
    ambient[2] = ret_B.sensor[1].ambient;
    reflective[3] = ret_B.sensor[0].reflective;
    reflective[2] = ret_B.sensor[1].reflective;
    proximity[3] = ret_B.sensor[0].proximity;
    proximity[2] = ret_B.sensor[1].proximity;
    beacon[3] = ret_B.sensor[0].docking;
    beacon[2] = ret_B.sensor[1].docking;
    ambient[4] = ret_C.sensor[0].ambient;
    ambient[5] = ret_C.sensor[1].ambient;
    reflective[4] = ret_C.sensor[0].reflective;
    reflective[5] = ret_C.sensor[1].reflective;
    proximity[4] = ret_C.sensor[0].proximity;
    proximity[5] = ret_C.sensor[1].proximity;
    beacon[4] = ret_C.sensor[0].docking;
    beacon[5] = ret_C.sensor[1].docking;
    ambient[6] = ret_D.sensor[0].ambient;
    ambient[7] = ret_D.sensor[1].ambient;
    reflective[6] = ret_D.sensor[0].reflective;
    reflective[7] = ret_D.sensor[1].reflective;
    proximity[6] = ret_D.sensor[0].proximity;
    proximity[7] = ret_D.sensor[1].proximity;
    beacon[6] = ret_D.sensor[0].docking;
    beacon[7] = ret_D.sensor[1].docking;

    uint8_t ethernet_status=0;
    for(int i=0;i<NUM_DOCKS;i++)
    {
    //    color[i] = GetRGB(ScoutBot::Side(i));
        if(((ScoutBot*)irobot)->isEthernetPortConnected(ScoutBot::Side(i)))
            ethernet_status |= 1<<i;
    }
    ethernet_status_hist.Push2(ethernet_status);

    for(int i=0;i<NUM_IRS;i++)
    {
    //    reflective_hist[i].Push(reflective[i]-para.reflective_calibrated[i]);
    //    ambient_hist[i].Push(para.ambient_calibrated[i] - ambient[i]);
    }
}

void RobotSCOUT::UpdateActuators()
{
    printf("Scout Update Actuators\n");

    if(speed[LEFTWHEEL] > 100)
        speed[LEFTWHEEL]  = 100;
    else if(speed[LEFTWHEEL]  < -100)
        speed[LEFTWHEEL]  = -100;
    if(speed[RIGHTWHEEL]  > 100)
        speed[RIGHTWHEEL]  = 100;
    else if(speed[RIGHTWHEEL]  < -100)
        speed[RIGHTWHEEL]  = -100;

    ((ScoutBot*)irobot)->Move(speed[LEFTWHEEL], speed[RIGHTWHEEL]);
}
