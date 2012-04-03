#include "robot_KIT.hh"
#include "og/organism_sample.hh"

RobotKIT::RobotKIT():Robot(),KaBot()
{
    if(name)
        free(name);
    name = strdup("RobotKIT");
    type = ROBOT_KIT;

    //point to right SPI device number
    board_dev_num[FRONT] = KaBot::FRONT;
    board_dev_num[RIGHT] = KaBot::RIGHT;
    board_dev_num[BACK] = KaBot::REAR;
    board_dev_num[LEFT] = KaBot::LEFT;

    LED0 = 0x1;
    LED1 = 0x2;
    LED2 = 0x4;
    IR_PULSE0 = 0x1;
    IR_PULSE1 = 0x2;
    IR_PULSE2 = 0x4;

    printf("Consctruction RobotKIT\n");
}

RobotKIT::~RobotKIT()
{
    printf("Desctruction RobotKIT\n");
}

void RobotKIT::InitHardware()
{
    for(int i=0;i<NUM_DOCKS;i++)
    {
        SetPrintEnabled(i, false); 
    }

    IRComm::Initialize();
}

void RobotKIT::Reset()
{
    RobotBase::MSPReset();
}

void RobotKIT::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    int board = board_dev_num[channel];
    RobotBase::SetIRLED(board, led);
    RobotBase::SetIRPulse(board, pulse_led);
    RobotBase::SetIRMode(board, mode);

    uint8_t status = (uint8_t)mode | 0x4;
    for(int i=0;i<3;i++)
    {
        IRLED_status[channel] &= ~ (0x7 << ( 3 * i )); //clear corresponding bits first
        if( led & (1<<i) )
        {
            IRLED_status[channel] |= status << ( 3 * i );; //set the bits;
        }

    }

    IRLED_status[channel] &= 0x1FF; // bits 7 and 8 for IR_pulse;
    IRLED_status[channel] |= pulse_led << 9;

}

void RobotKIT::SetRGBLED(int channel, uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br)
{
    int board = board_dev_num[channel];
    RobotBase::SetLED(board, tl, tr, bl, br);
    RGBLED_status[channel] = tl|tr|bl|br;

}

void RobotKIT::SetSpeed(int8_t leftspeed, int8_t rightspeed, int8_t sidespeed)
{
    if(fabs(sidespeed) > 10)
    {
        MoveScrewFront(para.speed_sideward * sidespeed);
        MoveScrewRear(para.speed_sideward* sidespeed);
    }
    else
    {
        MoveScrewFront(leftspeed * direction);
        MoveScrewRear(-rightspeed * direction);
    }
}



bool RobotKIT::SetDockingMotor(int channel, int status)
{
    CPrintf4(SCR_BLUE,"%d -- side %d set motor %#x, (status: %#x)", timestamp, channel, status, docking_motors_status[channel]);

    if(status != STOP)
    {
            //closed -> open?
            if(docking_motors_status[channel] == CLOSED  && status == OPEN)
            {
                //change status to be opening
                docking_motors_status[channel] = OPENING; //clear first
//                OpenDocking(KaBot::Side(board_dev_num[channel]));
                printf("open docking\n");
            }
            //or open -> close
            else if(docking_motors_status[channel]== OPENED &&  status == CLOSE )
            {
                //change status to be closing
                docking_motors_status[channel] = CLOSING; //clear first
//                CloseDocking(KaBot::Side(board_dev_num[channel]));
                printf("close docking\n");
            }
            else
                return false;

    }
    else
    {
        // opeing/open -->stop
        if(docking_motors_status[channel] == OPENED || 
           docking_motors_status[channel] == OPENING )
        {
            docking_motors_status[channel]=OPENED;
        }
        // closing/closed -> stop ?
        else
        {
            docking_motors_status[channel] = CLOSED;
        }
    }

    //TODO: checking this
    //MoveDocking(KaBot::Side(board_dev_num[channel]), status);
    SetRGBLED(1, GREEN, GREEN, RED, RED);
    printf("\n------ moving docking motor ----\n");
    return true;
}

bool RobotKIT::SetHingeMotor(int status)
{
    if(status !=STOP && status !=UP && status !=DOWN)
        return false;

    printf("\n\n%d -- set hinge motor %#x, (status: %#x)\n\n", timestamp, status, hinge_motor_status);

    if(status != STOP)
    {
        //not in status closing or opening?
        //closed -> open?
        if(hinge_motor_status == LIFTED && status == DOWN)
        {
            //change status to be opening
            hinge_motor_status = LOWING; //clear first
        }
        //or open -> close
        else if(hinge_motor_status == LOWED &&  status == UP )
        {
            //change status to be closing
            hinge_motor_status = LIFTING; //then set
        }
        else
            return false;

    }
    else
    {
        // opeing/open -->stop
        if((hinge_motor_status & 0x2) == 0)
            hinge_motor_status = 0;
        // closing/closed -> stop ?
        else
            hinge_motor_status = 0x2;
    }

    // MoveHinge(45 * status);
    printf("\n\n------ moving hinge motor ----\n\n");
    return true;
}



void RobotKIT::UpdateSensors()
{
    //sensor no
    //0 -- front right
    //1 -- front left
    //2 -- left front
    //3 -- left rear
    //4 -- rear left
    //5 -- rear right
    //6 -- right rear
    //7 -- right front
    IRValues ret_A = GetIRValues(FRONT);
    IRValues ret_B = GetIRValues(LEFT);
    IRValues ret_C = GetIRValues(REAR);
    IRValues ret_D = GetIRValues(RIGHT);
    ambient[1] = ret_A.sensor[0].ambient;
    ambient[0] = ret_A.sensor[1].ambient;
    reflective[1] = ret_A.sensor[0].reflective;
    reflective[0] = ret_A.sensor[1].reflective;
    proximity[1] = ret_A.sensor[0].proximity;
    proximity[0] = ret_A.sensor[1].proximity;
    beacon[1] = ret_A.sensor[0].docking;
    beacon[0] = ret_A.sensor[1].docking;
    ambient[3] = ret_B.sensor[0].ambient;
    ambient[2] = ret_B.sensor[1].ambient;
    reflective[3] = ret_B.sensor[0].reflective;
    reflective[2] = ret_B.sensor[1].reflective;
    proximity[3] = ret_B.sensor[0].proximity;
    proximity[2] = ret_B.sensor[1].proximity;
    beacon[3] = ret_B.sensor[0].docking;
    beacon[2] = ret_B.sensor[1].docking;
    ambient[5] = ret_C.sensor[0].ambient;
    ambient[4] = ret_C.sensor[1].ambient;
    reflective[5] = ret_C.sensor[0].reflective;
    reflective[4] = ret_C.sensor[1].reflective;
    proximity[5] = ret_C.sensor[0].proximity;
    proximity[4] = ret_C.sensor[1].proximity;
    beacon[5] = ret_C.sensor[0].docking;
    beacon[4] = ret_C.sensor[1].docking;
    ambient[7] = ret_D.sensor[0].ambient;
    ambient[6] = ret_D.sensor[1].ambient;
    reflective[7] = ret_D.sensor[0].reflective;
    reflective[6] = ret_D.sensor[1].reflective;
    proximity[7] = ret_D.sensor[0].proximity;
    proximity[6] = ret_D.sensor[1].proximity;
    beacon[7] = ret_D.sensor[0].docking;
    beacon[6] = ret_D.sensor[1].docking;

    //for(int i=0;i<NUM_DOCKS;i++)
    //    color[i] = GetRGB(KaBot::Side(i));

    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-reflective_calibrated[i]);
        ambient_hist[i].Push(ambient_calibrated[i] - ambient[i]);
    }
}

void RobotKIT::UpdateActuators()
{
    CheckDockingMotor();
    CheckHingeMotor();
    SetSpeed(leftspeed, rightspeed,sidespeed); 
}

// for self-repair
void RobotKIT::UpdateFailures()
{
	if( para.debug.para[2] > 0 && timestamp > para.debug.para[2] )
		module_failed = true;
}

void RobotKIT::Avoidance()
{
    leftspeed = 40;
    rightspeed = 40;
    sidespeed = 0;


    for(int i=0;i<NUM_IRS;i++)
    {
        //   leftspeed +=(direction * avoid_weightleft[i] * (reflective_avg[i]))>>3;
        //   rightspeed += (direction * avoid_weightleft[i] * (reflective_avg[i]))>>3;
        sidespeed += (para.avoid_weightside[i] * (reflective_hist[i].Avg()))>>3;
    }

    if(reflective_hist[1].Avg() > para.avoidance_threshold || reflective_hist[0].Avg()>para.avoidance_threshold)
        direction = BACKWARD;
    else if(reflective_hist[4].Avg() > para.avoidance_threshold || reflective_hist[5].Avg()>para.avoidance_threshold)
        direction = FORWARD;

    sidespeed = 0;
    leftspeed = 0;
    rightspeed = 0;


}

void RobotKIT::Exploring()
{
    Avoidance();
}


void RobotKIT::Resting()
{
    /*
    if(timestamp == 30)
    {
        mytree.Clear();
        if(og)
            delete og;
        //select predefined organism
        og = new Organism;
        RealDemoOrganism_KAK(og);
        og->GraphToSequence(mytree);
        std::cout<<*og<<std::endl;
        std::cout<<mytree<<std::endl;
        //prepare branches sequence
        rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
        if(ret.status >= RT_ERROR)
        {
            std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        SendBranchTree(2, mytree);
    }
*/

}
void RobotKIT::Seeding()
{
    mytree.Clear();
    //select predefined organism
    //og = new Organism;
    //RealDemoOrganism_KAK(og);
    //og->GraphToSequence(mytree);
    //std::cout<<*og<<std::endl;
    if(!para.og_seq_list.empty())
    {
        mytree = para.og_seq_list[0];
        std::cout<<mytree<<std::endl;
    }
    else
        printf("Warning: empty organism sequence info\n");


    for(int i=0;i<SIDE_COUNT;i++)
    {
        recruitment_count[i] = 0;
        recruitment_signal_interval_count[i] = DEFAULT_RECRUITMENT_COUNT;
    }

    current_state = RECRUITMENT;
    last_state = SEEDING;

    seed = true;


    //prepare branches sequence
    rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
    if(ret.status >= RT_ERROR)
    {
        std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }

    std::vector<OrganismSequence>::iterator it;

    //TODO: not to disable all
    //disable all ir leds first
    for(int i=0;i<NUM_DOCKS;i++)
        SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0x0); 
    //enable the recruiting side
    for(it = mybranches.begin() ; it != mybranches.end(); it++)
    {
        //check the first symbol that indicates the parent and child side of the connection
        uint8_t branch_side = it->getSymbol(0).side1;
        //enalbe docking signals
        SetIRLED(branch_side, IRLEDDOCKING, LED1, IR_PULSE0|IR_PULSE1); 
        std::cout<<name<<" branch "<<*it<<std::endl;
    }
}
void RobotKIT::Foraging()
{
    //time up?
    if(foraging_count--<=0)
    {
        foraging_count = DEFAULT_FORAGING_COUNT;
        waiting_count=DEFAULT_WAITING_COUNT;

        //switch off all ir leds
        for(uint8_t i=0; i< NUM_DOCKS; i++)
        {
            SetIRLED(i, IRLEDOFF, LED1, 0x0);
            SetRGBLED(i, 0,0,0,0);
        }
        current_state = WAITING;
        last_state = FORAGING;

        leftspeed = 0;
        rightspeed = 0;
        sidespeed = 0;
    }
    else
    {
        Avoidance();

        if(powersource_found)
        {
            current_state = LOCATEENERGY;
            last_state = FORAGING;
        }
    }


}
void RobotKIT::Waiting()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    msg_unlockme_received = 0;
    msg_locked_received = 0;
    msg_lockme_received = 0;

    if(waiting_count--<=0)
    {
        waiting_count=DEFAULT_WAITING_COUNT;
        foraging_count=DEFAULT_FORAGING_COUNT;

        for(int i=0;i< NUM_IRS;i++)
            reflective_hist[i].Reset();

        //switch on proximity ir leds and ir pulsing
        for(uint8_t i=0; i< NUM_DOCKS; i++)
            SetIRLED(i, IRLEDOFF, LED1, IR_PULSE0|IR_PULSE1);

        current_state = FORAGING;
        last_state = WAITING;
    }
    else if(organism_found)
    {
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);

        current_state = ASSEMBLY;
        last_state = WAITING;
    }
}

void RobotKIT::Assembly()
{
    if(assembly_count--<=0)
    {
        organism_found = false;

        current_state = FORAGING;
        last_state = ASSEMBLY;
    }
    //right type of recruitment message recived, then locatebeacon
    else if (assembly_info.type2 == type)
    {
        current_state = LOCATEBEACON;
        last_state = FORAGING;
    }
    else
        Avoidance();
}

void RobotKIT::LocateEnergy()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    if(1)
    {
        current_state = SEEDING;
        last_state = LOCATEENERGY;
        return;
    }
}


void RobotKIT::LocateBeacon()
{
    direction = FORWARD;

    //TODO: any side docking?
    if(beacon_signals_detected & 0x3)
    {
        int temp = beacon[1]-beacon[0];
        if(abs(temp) > 20)
        {
            leftspeed = 0;
            rightspeed = 0;
            sidespeed = 20 * sign(temp);
        }
        else if(abs(temp) > 10)
        {
            leftspeed = 0;
            rightspeed = 0;
            sidespeed = 15 * sign(temp);
        }
        else
        {
            leftspeed = 20;
            rightspeed = 20;

            if((timestamp/5)%2 ==0)
                sidespeed = 20;
            else
                sidespeed = -20;
        }
    }
    else
    {
        leftspeed = 20;
        rightspeed = 20;

        if((timestamp/5)%2 ==0)
            sidespeed = 20;
        else
            sidespeed = -20;

    }

    //      printf("beacon: (%d %d) -- speed: (%d %d %d)\n", beacon[1], beacon[0], leftspeed, rightspeed, sidespeed);
    //switch on ir led at 64Hz so the recruitment robot can sensing it
    //and turn on its docking signals, the robot need to switch off ir 
    //led for a while to check if it receives docking signals
    static int flag=0;
    if(timestamp % 5 ==0)
    {
        flag++;
        switch (flag % 6)
        {
            case 0:
            case 1:
            case 2:
                break;
            case 3:
            case 4:
                {
                    //switch off all ir led, 
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
                        SetRGBLED(i, 0,0,0,0);
                    }
                }
                break;
            case 5:
                {
                    //check if received docking signals
                    if(beacon_signals_detected_hist.Sum(0) >= 5 && beacon_signals_detected_hist.Sum(1) >= 5)  //TODO:only FRONT side at the moment, should changes according to message received
                    {
                        current_state = ALIGNMENT;
                        last_state = LOCATEBEACON;

                        //using reflective signals if not set
                        for(int i=0; i< NUM_DOCKS;i++)
                            SetIRLED(i, IRLEDOFF, LED1, IR_PULSE0 | IR_PULSE1);

                        return;
                    }
                    else
                    {
                        //then swith on all ir led at 64Hz frequency
                        for(int i=0;i<NUM_DOCKS;i++)
                            SetIRLED(i, IRLEDPROXIMITY, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
                    }
                }
                break;
            default:
                break;
        }
    }

}

void RobotKIT::Alignment()
{
    leftspeed = para.speed_forward;
    rightspeed = para.speed_forward;
    sidespeed = 0;

    int temp = beacon[1]-beacon[0];
    int temp2 = (reflective_hist[1].Avg())-(reflective_hist[0].Avg());

    if(abs(temp) > 40)
    {
        leftspeed = 0;
        rightspeed=0;
        sidespeed = 40 * sign(temp);
    }
    else if(abs(temp) > 20)
    {
        leftspeed = 0;
        rightspeed = 0;
        sidespeed = 30 * sign(temp);
    }
    else if(abs(temp) > 10)
    {
        leftspeed = 0;
        rightspeed = 0;
        sidespeed = 20 * sign(temp);
    }
    if(temp2> 200)
    {
        //turn left
        leftspeed = 40;
        rightspeed = -20;
        sidespeed = 0; //overwrite sidespeed
    }
    else if(temp2 > -200)
    {
        //do nothing
    }
    else
    {
        leftspeed = 20;
        rightspeed = -40;
        sidespeed = 0; //overwrite sidespeed
    }

    //lost signals
    //if(beacon_signals_detected==0)
    //beacon signals drop to certain threshold?
    if(beacon_signals_detected_hist.Sum(0) < 0 || beacon_signals_detected_hist.Sum(1) < 1)
    {
        current_state = RECOVER;
        last_state = ALIGNMENT;

        return;
    }

    //check if it is aligned well and also closed enough for docking
    int input[4] = {reflective_hist[0].Avg(), reflective_hist[1].Avg(), beacon[0], beacon[1]};
    in_docking_region_hist.Push(in_docking_region(input));
    if(in_docking_region_hist.Sum() >= 2) // at least 7 successful prediction out of 8  in docking region
    {
        in_docking_region_hist.Reset();

        current_state = DOCKING;
        last_state = ALIGNMENT;

        docking_count = 0;

        for(int i=0;i<NUM_IRS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
        SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);
        return;
    }
}
void RobotKIT::Recover()
{
    SetRGBLED(1, RED, RED, RED, RED);
    SetRGBLED(3, RED, RED, RED, RED);

    //  if(reflective_hist[1].Avg() > AVOIDANCE_THRESHOLD || reflective_hist[0].Avg()>AVOIDANCE_THRESHOLD)
    //      direction = BACKWARD;
    //  else if(reflective_hist[4].Avg() > AVOIDANCE_THRESHOLD || reflective_hist[5].Avg()>AVOIDANCE_THRESHOLD)
    //      direction = FORWARD;


    leftspeed = -30;
    rightspeed = -30;

    if(beacon_signals_detected & 0x3)
    {
        SetRGBLED(1, 0,0,0,0);
        SetRGBLED(3, 0,0,0,0);
        direction = FORWARD;
        current_state = ALIGNMENT;
        last_state = RECOVER;
    }
}
void RobotKIT::Docking()
{
    if(proximity[0] > 200 && proximity[1] > 200)
        SetRGBLED(0,0,0,0,0);
    else if(proximity[0]<20 && proximity[1]< 20)
        SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);

    int input[4]={proximity[0], proximity[1], reflective_hist[0].Avg(), reflective_hist[1].Avg()};
    in_locking_region_hist.Push(in_locking_region(input));

    int temp_reflective = reflective_hist[1].Avg() - reflective_hist[0].Avg();
    int temp_proximity = proximity[1] - proximity[0];
    static  int status = FORWARD;

    if(timestamp % (DOCKING_CHECKING_INTERVAL/2) == 0)
    {
        status = CHECKING;
    }
    else if(timestamp %DOCKING_CHECKING_INTERVAL == 1)
    {
        status = CHECKING;//MOVE_FORWARD;
    }
    else if(timestamp % DOCKING_CHECKING_INTERVAL == DOCKING_CHECKING_INTERVAL/2 + 1)
    {
        printf("%d %d \n", temp_proximity, temp_reflective);
        if(abs(temp_proximity)> 400 || abs(temp_reflective) > 1200)
            status = MOVE_BACKWARD;
        else if(temp_proximity > 200 || temp_reflective > 400)
            status = TURN_LEFT;
        else if(temp_proximity< -200 || temp_reflective < -300)
            status = TURN_RIGHT;
        else
            status = MOVE_FORWARD;
    }

    if(in_locking_region_hist.Sum() > 2) // 4 successful predition out of 8 
    {
        in_locking_region_hist.Reset();
        leftspeed = 0;
        rightspeed= 0;  
        SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);
        SetIRLED(0, IRLEDOFF, LED0|LED2, 0);
        RobotBase::SetIRRX(board_dev_num[0], false);
        SetDockingMotor(0, CLOSE);

        //TODO depend which types of robot it docks to, it may be required to send lockme messages

        current_state = LOCKING;
        last_state = DOCKING;
    }

    switch (status)
    {
        case TURN_RIGHT:
            leftspeed = para.docking_turn_right_speed[0];
            rightspeed = para.docking_turn_right_speed[1];
            sidespeed = para.docking_turn_right_speed[2];
            break;
        case TURN_LEFT:
            leftspeed = para.docking_turn_left_speed[0];
            rightspeed = para.docking_turn_left_speed[1];
            sidespeed = para.docking_turn_left_speed[2];
            break;
        case MOVE_FORWARD:
            leftspeed = para.docking_forward_speed[0];
            rightspeed = para.docking_forward_speed[1];
            sidespeed = para.docking_forward_speed[2];
            break;
        case MOVE_BACKWARD:
            leftspeed = para.docking_backward_speed[0];
            rightspeed = para.docking_backward_speed[1];
            sidespeed = para.docking_backward_speed[2];
            break;
        case CHECKING:
            leftspeed = 0;
            rightspeed = 0;
            sidespeed = 0;
            //no beacon2 signals?
            if(proximity[0] < 50 && proximity[1]<50)
            {
                //          SetRGBLED(0, 0, 0, 0, 0);
                BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_GUIDEME, false);
                //              SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);
            } 
            break;
        default:
            break;
    }
}

void RobotKIT::Locking()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    int docking_side = assembly_info.side2;


    SetRGBLED(docking_side, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason

    //docking motor is done?
    if(docking_motors_status[docking_side] == CLOSED) //replace 0 with corresponding docking face
    {
        if(docked[docking_side]==false)
        {
            docked[docking_side] = true;
            unlocking_required[docking_side] = true;
            //for(int i=0;i<NUM_DOCKS;i++)
            {
                // SetIRLED(docking_side, IRLEDOFF, LED0|LED2, 0);
                // RobotBase::SetIRRX(board_dev_num[docking_side], false);

            }
            BroadcastIRMessage(docking_side, IR_MSG_TYPE_LOCKED, true);
        }
        else if(docked[docking_side]==true && !MessageWaitingAck(docking_side, IR_MSG_TYPE_LOCKED))
        {
            msg_organism_seq_expected = true;
            current_state = INORGANISM;
            last_state = LOCKING;
        }
    }

}

void RobotKIT::Recruitment()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;
    std::vector<OrganismSequence>::iterator it1 = mybranches.begin();
    while(it1 !=mybranches.end())
    {
        int8_t i = it1->getSymbol(0).side1;

        bool erase_required = false;
        if(recruitment_stage[i]==STAGE0)
        {
            if(robots_in_range_detected_hist.Sum(2*i) > 14 || robots_in_range_detected_hist.Sum(2*i+1) >14)
            {
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDDOCKING, LED1, IR_PULSE0 | IR_PULSE1); //TODO: better to switch off ir pulse
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
            else
            {
                //or send recruitment message
                SetIRLED(i, IRLEDOFF, LED1, IR_PULSE0 | IR_PULSE1); //TODO: better to switch off ir pulse
                SetRGBLED(i, 0,0,0,0);
                if(timestamp % RECRUITMENT_SIGNAL_INTERVAL == i)
                {
                    BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING, it1->getSymbol(0).data);
                }
            }
        }
        else if(recruitment_stage[i]==STAGE1)
        {
            if(msg_guideme_received & (1<<i) || ( (robot_in_range_detected & (0x3<<(2*i))) ==0
                        && ambient_hist[2*i].Avg()> para.recruiting_ambient_offset1
                        && ambient_hist[2*i+1].Avg()>para.recruiting_ambient_offset1
                        && reflective_hist[2*i].Avg()>20 && reflective_hist[2*i+1].Avg()>20))
            {
                recruitment_stage[i]=STAGE2;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                proximity_hist[2*i].Reset();
                proximity_hist[2*i+1].Reset();
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE2)
        {
            proximity_hist[2*i].Push(proximity[2*i]);
            proximity_hist[2*i+1].Push(proximity[2*i+1]);

            proximity_hist[2*i].Print();
            ambient_hist[2*i].Print();
            msg_guideme_received &= ~(1<<i);
            msg_lockme_expected |=1<<i;

            if(ambient_hist[2*i].Avg() > para.recruiting_ambient_offset2 
                    && ambient_hist[2*i+1].Avg() > para.recruiting_ambient_offset2 
                    && proximity_hist[2*i].Avg() > para.recruiting_proximity_offset1 
                    && proximity_hist[2*i+1].Avg() > para.recruiting_proximity_offset2)
            {
                recruitment_stage[i]=STAGE3;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                RobotBase::SetIRRX(board_dev_num[i], false);
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE3)
        {
            if(msg_lockme_received & (1<<i))
            {
                msg_lockme_received &=~(1<<i);
                SetDockingMotor(i, CLOSE); //here it is safe to call clos motor repeately

                recruitment_stage[i]=STAGE4;
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
            else if(msg_guideme_received & (1<<i))
            {
                proximity_hist[2*i].Reset();
                proximity_hist[2*i+1].Reset();
                recruitment_stage[i] = STAGE2;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                printf("%d -- Recruitment: switch back to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE4)
        {
            // printf("%d STAGE4\n", timestamp);
            if(docking_motors_status[i] == CLOSED && docked[i]==false)
            {
                BroadcastIRMessage(i, IR_MSG_TYPE_LOCKED, true);
                unlocking_required[i] = true;
                docked[i]=true;
            }
            else if(docked[i]==true && !docking_done[i])
            {
                if(!MessageWaitingAck(i, IR_MSG_TYPE_LOCKED))
                {
                    docking_done[i] = true;
                    SendBranchTree(i, mytree);
                }
            }
            else if(docking_done[i])
            {
                //recevied acks after sending sequence information?
                if(!MessageWaitingAck(i, IR_MSG_TYPE_ORGANISM_SEQ))
                {
                    docked[i] = true;
                    recruitment_stage[i] = STAGE0;
                    docking_done[i] = false;

                    //SetRGBLED(i, RED, 0, 0, 0);
                    //SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    //RobotBase::SetIRRX(board_dev_num[i], false);

                    num_robots_inorganism++;
                    //prepare the newrobot_joined messages
                    if(!seed)
                        PropagateIRMessage(IR_MSG_TYPE_NEWROBOT_JOINED, NULL, 0, i);
                    //remove branches since it has been sent to newly joined robot
                    erase_required = true;
                }
            }
        }

        if(erase_required)
            it1 = mybranches.erase(it1);
        else
            ++it1;

    }

    //recruitment done?
    if(mybranches.empty())
    {
        current_state = INORGANISM;
        last_state = RECRUITMENT;
        memset(docking_done, 0, NUM_DOCKS);
        robot_in_range_replied = 0;

        for(int i=0;i<NUM_DOCKS;i++)
            SetRGBLED(i, 0, 0, 0, 0);
    }
}


void RobotKIT::InOrganism()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    if( timestamp < 40 )
        	return;

	if(timestamp == 40)
	 {
		 for(int i=0;i<NUM_IRS;i++)
			 SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);

		 docked[BACK] = true;
	 }

	// for self-repair - needs to be replicated for all
	// other states from which self-repair is possible.
	if( module_failed )
	{
		// Send 'failed' message to all neighbours
		for( int i=0; i<NUM_DOCKS; i++ )
		{
			if( docked[i] )
				SendFailureMsg(i);
		}

		last_state = INORGANISM;
		current_state = FAILED;

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, RED, RED, RED, RED);


		printf("%d Module failed!\n",timestamp);

	}
	else if( msg_failed_received )
	{
		msg_failed_received = 0;

		wait_side = FRONT;
		repair_stage = STAGE0;
		subog.Clear();
		best_score = 0;
		subog_str[0] = 0;

		// Find the first side at which another neighbour is docked
		while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
			wait_side++;

		if( wait_side < SIDE_COUNT )
			SendSubOGStr( wait_side, subog_str );

		last_state = INORGANISM;
		current_state = LEADREPAIR;

		printf("%d Detected failed module, entering LEADREPAIR, sub-organism ID:%d\n",timestamp,subog_id);

		for( int i=0; i<SIDE_COUNT; i++ )
			SetRGBLED(i, GREEN, GREEN, GREEN, GREEN);
	}
	else if( msg_sub_og_seq_received )
	{
		msg_sub_og_seq_received = 0;
		repair_stage = STAGE0;

		// Find the first side at which another neighbour is docked
		while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
			wait_side++;

		if( wait_side < SIDE_COUNT )
			SendSubOGStr( wait_side, subog_str );

		last_state = INORGANISM;
		current_state = REPAIR;
	}
	//////// END self-repair ////////

	else
	{
		return;
	}

    //seed robot monitoring total number of robots in the organism
    if(seed)
    {
        if( mytree.Edges() + 1 == (unsigned int)num_robots_inorganism)
        {
            organism_formed = true;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 

            //prepare organism_formed_messages
            PropagateIRMessage(IR_MSG_TYPE_ORGANISM_FORMED);

            current_state = MACROLOCOMOTION;
            last_state = INORGANISM;
        }
    }
    //otherwise check if new info received
    else
    {
        if(msg_organism_seq_expected && msg_organism_seq_received)
        {
            msg_organism_seq_received = 0;
            msg_organism_seq_expected = false;

            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, 0,0,0,0);

            //check if  need to recruit new robots
            std::vector<OrganismSequence>::iterator it = mybranches.begin();
            bool recruiting_required = false;
            while(it!=mybranches.end())
            {
                //check the first symbol that indicates the parent and child side of the connection
                uint8_t branch_side = it->getSymbol(0).side1;
                std::cout<<name<<" branch "<<*it<<std::endl;

                recruitment_count[branch_side] = 0;
                recruitment_signal_interval_count[branch_side] = DEFAULT_RECRUITMENT_COUNT;

                ++it;
                recruiting_required = true;
            }

            if(recruiting_required)
            {
                current_state = RECRUITMENT;
                last_state = INORGANISM;
            }
        }
        else if(organism_formed)
        {
            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 

            macrolocomotion_count=0;
            current_state = MACROLOCOMOTION;
            last_state = INORGANISM;
        }
    }

}

void RobotKIT::Disassembly()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    if(!MessageWaitingAck(IR_MSG_TYPE_PROPAGATED))
    {
        //check if need to unlocking docking faces which is connected to Activewheel
        int num_docked = 0;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i])
            {
                num_docked++;
                if(unlocking_required[i])
                {
                    SetDockingMotor(i, OPEN);
                    unlocking_required[i]=false;
                }
                //TODO: how about two KIT robots docked to each other
                else if(docking_motors_status[i]==OPENED)
                {
                    BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, true);
                    docked[i]=false;
                    num_docked--;
                }
            }

        }

        //only one  or less 
        if(num_docked ==0)
        {
            current_state = UNDOCKING;
            last_state = DISASSEMBLY;
        }

    }


}

void RobotKIT::Undocking()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(timestamp % 4 ==0)
            SetRGBLED(i, 0,0,0,0);
        else
            SetRGBLED(i, RED,RED,RED,RED);
    }

    static int undocking_count=0;
    undocking_count++;

    if(undocking_count >= 120)
    {
        leftspeed = -30;
        rightspeed = -30;
        sidespeed = 0;
    }
}

void RobotKIT::Transforming()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    if(transforming_count ==2)
    {
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
        docked[0] = true;
        docked[2] = true;
        PropagateIRMessage(IR_MSG_TYPE_TRANSFORMING);
    }

    //flashing RGB leds
    static int index = 0;
    index = (timestamp / 2) % 4;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        switch (index)
        {
            case 0:
                SetRGBLED(i, RED, GREEN, 0, 0);
                break;
            case 1:
                SetRGBLED(i, 0, RED, 0, GREEN);
                break;
            case 2:
                SetRGBLED(i, 0, 0, GREEN, RED);
                break;
            case 3:
                SetRGBLED(i, GREEN, 0, RED, 0);
                break;
            default:
                break;
        }
    }

    if(transforming_count++ > 30 && MessageWaitingAck(IR_MSG_TYPE_PROPAGATED))
    {
        current_state = MACROLOCOMOTION;
        last_state = TRANSFORMING;
    }

}

///////// Self-repair //////////

void RobotKIT::Failed()
{

    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    /*
	if(!MessageWaitingAck(IR_MSG_TYPE_FAILED))
	{
		//check if need to unlocking docking faces which is connected to Activewheel
		int num_docked = 0;
		for(int i=0;i<NUM_DOCKS;i++)
		{
			if(docked[i])
			{
				num_docked++;
				if(unlocking_required[i])
				{
					SetDockingMotor(i, OPEN);
					unlocking_required[i]=false;
				}
				//TODO: how about two KIT robots docked to each other
				else if(docking_motors_status[i]==OPENED)
				{
					BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, true);
					docked[i]=false;
					num_docked--;
				}
			}
		}

		//only one  or less
		if(num_docked ==0)
		{
			current_state = UNDOCKING;
			last_state = FAILED;
		}
	}
	*/
}

// TODO: Initial repair state of the robot nearest
// to the failed module - not currently implemented
void RobotKIT::Support()
{



}

// Initial repair state of the robot nearest to the
// the failed/support module
void RobotKIT::LeadRepair()
{

	// notify other modules to enter repair
	// and determine shape of sub-organism
	if( repair_stage == STAGE0 )
	{
    	while(wait_side < SIDE_COUNT && (!docked[wait_side] || wait_side == parent_side))
    		wait_side++;

		if( wait_side < SIDE_COUNT )
		{
			if( msg_sub_og_seq_received )
			{
				wait_side++;
				msg_sub_og_seq_received = 0;
				SendSubOGStr( wait_side, subog_str );
			}
		}
		else
		{
			wait_side = FRONT;
	    	repair_stage = STAGE1;
	    	printf("%d Shape determined\n",timestamp );
	    	PrintSubOGString();
		}
	}
	// TODO: move away from failed module
	else if( repair_stage == STAGE1 )
	{
		// when finished send score string to first neighbour

	}
	// TODO: determine sub-organism score
	else if( repair_stage == STAGE2 )
	{
		/*
		if( wait_side < SIDE_COUNT )
		{
			if( msg_score_seq_received || !docked[wait_side] )
			{
				if( own_score < best_score ) own_score = 0;
				wait_side++;
				SendScoreStr( wait_side, subog, best_score );
			}
		}
		else
		{
			// TODO: broadcast best score
			repair_stage = STAGE0;
			wait_side = FRONT;
			current_state = BROADCASTSCORE;
			last_state = LEADREPAIR;
		}
		*/
	}


}

// Initial repair state of remaining modules
void RobotKIT::Repair()
{
	// determine sub-organism shape
	if( repair_stage == STAGE0 )
	{

		while( wait_side < SIDE_COUNT && ( wait_side == parent_side || !docked[wait_side] ) )
			wait_side++;

		if( wait_side < SIDE_COUNT )
		{
			if( msg_sub_og_seq_received )
			{
				wait_side++;
				msg_sub_og_seq_received = 0;

				// don't send message back to parent yet
				if( wait_side == (int) parent_side )
					wait_side++;

				SendSubOGStr( wait_side, subog_str );
			}
		}
		else
		{
			SendSubOGStr( parent_side, subog_str );

			own_score = -1;
			wait_side = FRONT;
			repair_stage = STAGE1;
		}
	}
	// TODO: move away from failed module
	else if( repair_stage == STAGE1 )
	{



	}
	// TODO: determine sub-organism score
	else if( repair_stage == STAGE2 )
	{
		/*
		if( own_score < 0 )
		{
			// wait for message from parent to determine own_score
			if( (msg_score_seq_received & 1<<parent_side) == 0 )
			{
				msg_score_seq_received = 0;
				own_score = calculateScore( subog, target );
				own_score > best_score ? best_score = own_score : own_score = 0;
			}
		}
		else
		{
			if( wait_side < SIDE_COUNT )
			{
				if( msg_score_seq_received || !docked[wait_side] )
				{
					if( own_score < best_score ) own_score = 0;

					wait_side++;

					// don't send message back to parent yet
					if( wait_side == (int) parent_side ) wait_side++;

					SendScoreStr( wait_side, subog, best_score );
				}
			}
			else
			{
				SendScoreStr( parent_side, subog, best_score );

				repair_stage = STAGE0;
				wait_side = FRONT;
				current_state = BROADCASTSCORE;
				last_state = REPAIR;
			}
		}
		*/
	}
}

void RobotKIT::BroadcastScore()
{



}



/////// Self-repair END ///////



void RobotKIT::Reshaping()
{

	// if received a message to disassemble - do so

	// if received a new branch
	//	send branches and disassembly messages to appropriate
	//	neighbours and enter the INORGANISM/RECRUITMENT state

}

void RobotKIT::MacroLocomotion()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;
    macrolocomotion_count++;

    //flashing RGB leds
    static int index = 0;
    index = (timestamp / 2) % 4;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        switch (index)
        {
            case 0:
                SetRGBLED(i, RED, GREEN, 0, 0);
                break;
            case 1:
                SetRGBLED(i, 0, RED, 0, GREEN);
                break;
            case 2:
                SetRGBLED(i, 0, 0, GREEN, RED);
                break;
            case 3:
                SetRGBLED(i, GREEN, 0, RED, 0);
                break;
            default:
                break;
        }
    }

    if(seed && macrolocomotion_count >= 150)
    {
        PropagateIRMessage(IR_MSG_TYPE_DISASSEMBLY);

        current_state = DISASSEMBLY;
        last_state = MACROLOCOMOTION;

        for(int i=0;i<NUM_DOCKS;i++)
            SetRGBLED(i, 0, 0, 0, 0);
    }
    else if(msg_disassembly_received)
    {
        current_state = DISASSEMBLY;
        last_state = MACROLOCOMOTION;

        msg_disassembly_received = 0;

        for(int i=0;i<NUM_DOCKS;i++)
            SetRGBLED(i, 0, 0, 0, 0);
    }

}

void RobotKIT::Debugging()
{
   // leftspeed = 0;
   // rightspeed = 0;
   // sidespeed = 0;

    if(timestamp >40)
        Log();
    printf("%d Debuging %d:\t", timestamp,para.debug.mode);

    switch (para.debug.mode)
    {
        case 0: //simulating recruitment, stage 2 
            if(timestamp ==40)
            {
                SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0);
            }

            printf("%d %d %d %d\n",  proximity[4], proximity[5], ambient_calibrated[4]-ambient[4], ambient_calibrated[4]-ambient[5]);

            break;
        case 1: // locking region threshold detection
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n",  proximity[0], proximity[1], beacon[0], beacon[1], reflective_hist[0].Avg(), reflective_hist[1].Avg());
            break;
        case 2: // simulate recruitment, stage 1, guiding signals
            if(timestamp == 40)
            {
                SetIRLED(2, IRLEDDOCKING, LED1, IR_PULSE0|IR_PULSE1);
            }
            printf("%d %d %d %d\n", reflective[4]-reflective_calibrated[4], reflective[5] - reflective_calibrated[5], ambient_calibrated[4]-ambient[4], ambient_calibrated[4]-ambient[5]);
            break;
        case 3: // docking region threshold detection
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n", reflective[0]-reflective_calibrated[0], reflective[1] - reflective_calibrated[1], beacon[0], beacon[1], proximity[0], proximity[1]);
            break;
        case 4: // measuring beacon signals
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
                leftspeed = para.debug.para[3];
                rightspeed = para.debug.para[4];
                printf("set speed %d %d\n", leftspeed, rightspeed);
            }
            else if(timestamp == para.debug.para[2])
            {
                leftspeed = 0;
                rightspeed = 0;
                current_state = RESTING;
                last_state = DEBUGGING;
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n", reflective[0]-reflective_calibrated[0], reflective[1] - reflective_calibrated[1], beacon[0], beacon[1], proximity[0], proximity[1]);
            if(beacon[0] > para.debug.para[0])
                SetRGBLED(3, GREEN, GREEN,0,0);
            else
                SetRGBLED(3, 0,0,0,0);
            if(beacon[1] > para.debug.para[1])
                SetRGBLED(1, GREEN, GREEN,0,0);
            else
                SetRGBLED(1, 0, 0, 0, 0);
            break;
            break;
        case 5: 
            printf("\n");
            if(timestamp ==40)
            {
            }

            if(isEthernetPortConnected(KaBot::FRONT))
            {
                SetRGBLED(2, RED, RED,RED,RED);
            }
            else
                SetRGBLED(2, 0,0,0,0);
            break;
        case 9:
            //      if(timestamp ==35)
            {
            }
            break;
        case 10:
            if(timestamp ==para.debug.para[8])
            {
                printf("lock motor\n");
                SetDockingMotor(0, CLOSE);
            }
            else if(timestamp == para.debug.para[9])
            {
                printf("unlock motor\n");
                SetDockingMotor(0, OPEN);
            }
            break;
        default:
            break;
    }

}

int RobotKIT::in_docking_region(int x[4])
{
    if(   x[0] > para.docking_reflective_offset1 
            && x[1] > para.docking_reflective_offset2 
            && abs(x[1]-x[0]) < para.docking_reflective_diff
            && x[2] < para.docking_beacon_offset1 
            && x[3] < para.docking_beacon_offset2 
            && abs(x[2]-x[3]) < para.docking_beacon_diff )
        return 1;
    else 
        return 0;
}

int RobotKIT::in_locking_region(int x[4])
{
    if( x[0] > para.locking_proximity_offset1 - para.locking_proximity_diff1 
            && x[0] < para.locking_proximity_offset1 + para.locking_proximity_diff1
            && x[1] > para.locking_proximity_offset2 - para.locking_proximity_diff2 
            && x[1] < para.locking_proximity_offset2 + para.locking_proximity_diff2
            && x[2] > para.locking_reflective_offset1 
            && x[3] > para.locking_reflective_offset2)
        return 1;
    else 
        return 0;
}

void RobotKIT::Log()
{
    if (logFile.is_open())
    {
      //  logFile << timestamp << "\t" << state_names[current_state] <<"\t";
      //  logFile << reflective_hist[0].Avg()<<"\t";
      //  logFile << reflective_hist[1].Avg()<<"\t";
        logFile << beacon[0]<<"\t";
        logFile << beacon[1]<<"\t";
      //  logFile << ambient_hist[0].Avg()<<"\t";
      //  logFile << ambient_hist[1].Avg()<<"\t";
      //  logFile << proximity[0]<<"\t";
      //  logFile << proximity[1]<<"\t";
        logFile << std::endl;
    }

}

