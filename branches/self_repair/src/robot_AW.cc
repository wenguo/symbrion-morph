#include "robot_AW.hh"

RobotAW::RobotAW():Robot()
{
    if(name)
        free(name);
    name = strdup("RobotAW");
    type = ROBOT_AW;

    board_dev_num[FRONT] = ActiveWheel::FRONT;
    board_dev_num[RIGHT] = ActiveWheel::RIGHT;
    board_dev_num[BACK] = ActiveWheel::REAR;
    board_dev_num[LEFT] = ActiveWheel::LEFT;

    LED0 = 0x1;
    LED1 = 0x4;
    LED2 = 0x2;
    IR_PULSE0 = 0x1;
    IR_PULSE1 = 0x2;
    IR_PULSE2 = 0x4;
    printf("Consctruction RobotAW\n");
}

RobotAW::~RobotAW()
{
    printf("Desctruction RobotAW\n");
}


void RobotAW::InitHardware()
{
    for(int i=0;i<NUM_DOCKS;i++)
    {
        SetPrintEnabled(i, false);
    }

    //RobotBase::SetIRRX(board_dev_num[2], false);

    IRComm::Initialize();
}

void RobotAW::Reset()
{
    RobotBase::MSPReset();
}

void RobotAW::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    int board = board_dev_num[channel];
    // RobotBase::SetIRLED(board, mode, led, pulse_led);
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

void RobotAW::SetRGBLED(int channel, uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br)
{
    int board = board_dev_num[channel];
    RobotBase::SetLED(board, tl, br, bl, tr);
    RGBLED_status[channel] = tl|tr|bl|br;
}

void RobotAW::SetSpeed(int8_t leftspeed, int8_t rightspeed, int8_t sidespeed)
{
    MoveWheelsFront(-leftspeed * direction, -sidespeed);
    MoveWheelsRear(rightspeed * direction,sidespeed);
}

bool RobotAW::SetDockingMotor(int channel, int status)
{
    printf("ActiveWheel has no docking motors\n");
    return true;
}

bool RobotAW::SetHingeMotor(int status)
{
    if(status !=STOP && status !=UP && status !=DOWN)
        return false;

    printf("\n\n%d -- set hinge motor %#x, (status: %#x)\n\n", timestamp, status, hinge_motor_status);

    if(status != STOP)
    {
        if(hinge_motor_status == LIFTED && status == DOWN)
        {
            hinge_motor_status = LOWING; 
        }
        else if(hinge_motor_status == LOWED  &&  status == UP )
        {
            hinge_motor_status = LIFTING;
        }
        else
        {
            printf("Hinge motor in operation, or no further up/down is allowed. motor_status: %#x, required_operation %d\n", hinge_motor_status, status);
            return false;
        }

    }
    else
    {
        if(hinge_motor_status  == LIFTING)
            hinge_motor_status = LIFTED;
        else
            hinge_motor_status = LOWED;
    }

    MoveHinge(45 * status);
    printf("\n\n------ moving hinge motor ----\n\n");
    return true;
}

void RobotAW::UpdateSensors()
{
    //for bfin testing
    IRValues ret_A = GetIRValues(ActiveWheel::RIGHT);
    IRValues ret_B = GetIRValues(ActiveWheel::FRONT);
    IRValues ret_C = GetIRValues(ActiveWheel::LEFT);
    IRValues ret_D = GetIRValues(ActiveWheel::REAR);
    ambient[0] = ret_A.sensor[0].ambient;
    ambient[1] = ret_A.sensor[1].ambient;
    reflective[0] = ret_A.sensor[0].reflective;
    reflective[1] = ret_A.sensor[1].reflective;
    proximity[0] = ret_A.sensor[0].proximity;
    proximity[1] = ret_A.sensor[1].proximity;
    beacon[0] = ret_A.sensor[0].docking;
    beacon[1] = ret_A.sensor[1].docking;
    ambient[3] = ret_B.sensor[0].ambient;
    ambient[2] = ret_B.sensor[0].ambient;
    reflective[3] = ret_B.sensor[0].reflective;
    reflective[2] = ret_B.sensor[0].reflective;
    proximity[3] = ret_B.sensor[0].proximity;
    proximity[2] = ret_B.sensor[0].proximity;
    beacon[3] = ret_B.sensor[0].docking;
    beacon[2] = ret_B.sensor[0].docking;
    ambient[4] = ret_C.sensor[0].ambient;
    ambient[5] = ret_C.sensor[1].ambient;
    reflective[4] = ret_C.sensor[0].reflective;
    reflective[5] = ret_C.sensor[1].reflective;
    proximity[4] = ret_C.sensor[0].proximity;
    proximity[5] = ret_C.sensor[1].proximity;
    beacon[4] = ret_C.sensor[0].docking;
    beacon[5] = ret_C.sensor[1].docking;
    ambient[7] = ret_D.sensor[0].ambient;
    ambient[6] = ret_D.sensor[0].ambient;
    reflective[7] = ret_D.sensor[0].reflective;
    reflective[6] = ret_D.sensor[0].reflective;
    proximity[7] = ret_D.sensor[0].proximity;
    proximity[6] = ret_D.sensor[0].proximity;
    beacon[7] = ret_D.sensor[0].docking;
    beacon[6] = ret_D.sensor[0].docking;


    //    for(int i=0;i<NUM_DOCKS;i++)
    //        color[i] = GetRGB(ActiveWheel::Side(i));

    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-reflective_calibrated[i]);
        ambient_hist[i].Push(ambient_calibrated[i] - ambient[i]);
    }
}

void RobotAW::UpdateActuators()
{
    CheckHingeMotor();
    SetSpeed(leftspeed, rightspeed,sidespeed); 
    printf("speed: %d %d %d\n", leftspeed, rightspeed, sidespeed);
}
void RobotAW::Avoidance()
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

    if(abs(sidespeed) < 20)
    {
        if((timestamp / 10 )%2 ==0)
            sidespeed = 20;
        else
            sidespeed =-20;
    }

    if(reflective_hist[1].Avg() > para.avoidance_threshold || reflective_hist[0].Avg()>para.avoidance_threshold)
        direction = BACKWARD;
    else if(reflective_hist[4].Avg() > para.avoidance_threshold || reflective_hist[5].Avg()>para.avoidance_threshold)
        direction = FORWARD;

    sidespeed = 0;
    leftspeed = 0;
    rightspeed = 0;

}


void RobotAW::Exploring()
{
    Avoidance();
}
void RobotAW::Resting()
{
    /*
    if(timestamp == 40)
    {
        mytree.Clear();
        if(og)
            delete og;
        //select predefined organism
        og = new Organism;
        RealDemoOrganism_AKAK(og);
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
    }*/
}
void RobotAW::Seeding() //the same as in RobotKIT
{
    mytree.Clear();
    //select predefined organism
    /*
    og = new Organism;
    RealDemoOrganism_AKAK(og);
    og->GraphToSequence(mytree);
    std::cout<<*og<<std::endl;
    std::cout<<mytree<<std::endl;*/
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
void RobotAW::Foraging() //the same as RobotKIT
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
void RobotAW::Waiting()//same as RobotKIT
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
void RobotAW::Assembly()
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
void RobotAW::LocateEnergy()//same as RobotKIT
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
void RobotAW::LocateBeacon()//same as RobotKIT
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

void RobotAW::Alignment()
{
    leftspeed = para.speed_forward;
    rightspeed = para.speed_forward;
    sidespeed = 0;

    int temp = beacon[0]-beacon[1];
    int temp2 = (reflective_hist[0].Avg())-(reflective_hist[1].Avg());

    if(abs(temp) > 40)
    {
        leftspeed = 0;
        rightspeed=0;
        sidespeed = 20 * sign(temp);
    }
    else if(abs(temp) > 20)
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
        if((timestamp/2)%2==0)
            sidespeed = 10;
        else
            sidespeed = -10;
    }
    if(abs(temp2)> 400)
    {
        leftspeed = -18 * sign(temp2);
        rightspeed = 18 * sign(temp2);
 
        //in case too much friction introduced by side wheels
        if((timestamp/2)%2==0)
            sidespeed = 20;
        else
            sidespeed = -20;
    }
    else if(abs(temp2)>200)
    {
        leftspeed = -13 * sign(temp2);
        rightspeed = 13 * sign(temp2);

        //in case too much friction introduced by side wheels
        if((timestamp/2)%2==0)
            sidespeed = 15;
        else
            sidespeed = -15;
    }
    else
    {
        if((timestamp/2)%2==0)
            sidespeed = 10;
        else
            sidespeed = -10;
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
    }

    printf("Alignment %d %d %d\n", leftspeed, rightspeed, sidespeed);
}
void RobotAW::Recover()
{
    SetRGBLED(1, RED, RED, RED, RED);
    SetRGBLED(3, RED, RED, RED, RED);

    //  if(reflective_hist[1].Avg() > AVOIDANCE_THRESHOLD || reflective_hist[0].Avg()>AVOIDANCE_THRESHOLD)
    //      direction = BACKWARD;
    //  else if(reflective_hist[4].Avg() > AVOIDANCE_THRESHOLD || reflective_hist[5].Avg()>AVOIDANCE_THRESHOLD)
    //      direction = FORWARD;


    leftspeed = -30;
    rightspeed = -30;
    if((timestamp/2)%2==0)
        sidespeed = 10;
    else
        sidespeed = -10;

    if(beacon_signals_detected & 0x3)
    {
        SetRGBLED(1, 0,0,0,0);
        SetRGBLED(3, 0,0,0,0);
        direction = FORWARD;
        current_state = ALIGNMENT;
        last_state = RECOVER;
    }
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;
}
void RobotAW::Docking()
{
    if(proximity[0] > 200 && proximity[1] > 200)
        SetRGBLED(0,0,0,0,0);
    //else if(proximity[0]<20 && proximity[1]< 20)
    //     SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);

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
        status = MOVE_FORWARD;
    }
    else if(timestamp % DOCKING_CHECKING_INTERVAL == DOCKING_CHECKING_INTERVAL/2 + 1)
    {
        printf("%d %d \n", temp_proximity, temp_reflective);
        if(abs(temp_proximity)> 400 || abs(temp_reflective) > 1000)
            status = MOVE_BACKWARD;
        else if(temp_proximity > 120 || temp_reflective > 500)
            status = TURN_LEFT;
        else if(temp_proximity< -120 || temp_reflective < -500)
            status = TURN_RIGHT;
        else
            status = MOVE_FORWARD;
    }

    if(in_locking_region_hist.Sum() > 2) // 4 successful predition out of 8 
    {
        in_locking_region_hist.Reset();
        leftspeed = 0;
        rightspeed= 0;  
        SetRGBLED(assembly_info.side2, WHITE, WHITE, WHITE, WHITE);
        SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED2, 0);
        RobotBase::SetIRRX(board_dev_num[assembly_info.side2], false);

        BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_LOCKME,  true);
        msg_locked_expected |= 1<<assembly_info.side2;

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

void RobotAW::Locking()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    int docking_side = assembly_info.side2;


    SetRGBLED(docking_side, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason

    if(msg_locked_received & (1<<docking_side)) //replace 0 with the correct docking face numer, currently 0 (FRONT)
    {
        msg_locked_received &= ~(1<<docking_side);
        docked[docking_side] = true;
        msg_organism_seq_expected = true;
        current_state = INORGANISM;
        last_state = LOCKING;
        //for(int i=0;i<NUM_DOCKS;i++)
        {
           // SetIRLED(docking_side, IRLEDOFF, LED0|LED2, 0); // switch off all ir pulse, using all LEDs  for communication as activewheel using the side receive
           // RobotBase::SetIRRX(board_dev_num[docking_side], false);
            SetRGBLED(docking_side, 0,0,0,0);
        }
    }
}
void RobotAW::Recruitment()
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
            //if((robot_in_range_detected & (0x3<<(2*i))) !=0)
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
            if(msg_guideme_received & (1<<i) ||((robot_in_range_detected & (0x3<<(2*i))) ==0
                        && ambient_hist[2*i].Avg()> para.recruiting_ambient_offset1
                        && ambient_hist[2*i+1].Avg()>para.recruiting_ambient_offset1
                        && reflective_hist[2*i].Avg()>20 && reflective_hist[2*i+1].Avg()>20))
            {
                recruitment_stage[i]=STAGE2;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                proximity_hist[2*i].Reset();
                proximity_hist[2*i+1].Reset();
                ambient_avg_threshold_hist.Reset();
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE2)
        {
            printf("%d %d\r", ambient_calibrated[2*i]-ambient[2*i], ambient_calibrated[2*i+1]-ambient[2*i+1]);

            proximity_hist[2*i].Push(proximity[2*i]);
            proximity_hist[2*i+1].Push(proximity[2*i+1]);
            msg_guideme_received &= ~(1<<i);

            uint8_t ambient_trigger = 0;
            if(ambient_hist[2*i].Avg() > para.recruiting_ambient_offset2)
                ambient_trigger |= 1<<(2*i);
            if(ambient_hist[2*i+1].Avg() > para.recruiting_ambient_offset2)
                ambient_trigger |= 1<<(2*i+1);
            ambient_avg_threshold_hist.Push2(ambient_trigger);
            if( ambient_avg_threshold_hist.Sum(2*i) > 3 && ambient_avg_threshold_hist.Sum(2*i+1)>3
                    && proximity_hist[2*i].Avg() > para.recruiting_proximity_offset1 && proximity_hist[2*i+1].Avg()> para.recruiting_proximity_offset2)
                //&& abs(ambient_hist[2*i].Avg() - ambient_hist[2*i+1].Avg()) < 250)          
            {
                recruitment_stage[i]=STAGE3;
                msg_locked_expected |= 1<<i;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                RobotBase::SetIRRX(board_dev_num[i], false);
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE3)
        {
            if(msg_locked_received & (1<<i))
            {
                msg_locked_received &=~(1<<i);
                docked[i]=true;

                SendBranchTree(i, mytree);

                recruitment_stage[i]=STAGE4;
                printf("%d -- Recruitment: switch to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
            else if(msg_guideme_received & (1<<i))
            {
                proximity_hist[2*i].Reset();
                proximity_hist[2*i+1].Reset();
                ambient_avg_threshold_hist.Reset();
                recruitment_stage[i] = STAGE2;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                printf("%d -- Recruitment: switch back to Stage%d\n\n", timestamp, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE4)
        {
            //recevied acks after sending sequence information?
            if(!MessageWaitingAck(i, IR_MSG_TYPE_ORGANISM_SEQ))
            {
                docked[i] = true;
                docking_done[i] = true;
                recruitment_stage[i] = STAGE0;
                docking_done[i] = false;

                //SetRGBLED(i, RED, 0, 0, 0);
                //SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                //RobotBase::SetIRRX(board_dev_num[i], false);

                if(seed)
                    num_robots_inorganism++;
                else
                {
                    //prepare the newrobot_joined messages
                    PropagateIRMessage(IR_MSG_TYPE_NEWROBOT_JOINED, NULL, 0, i);
                }

                erase_required = true;
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
void RobotAW::InOrganism()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

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
            msg_organism_seq_expected = 0;

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
void RobotAW::Disassembly()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    //no need to propagate disassembling messages?
    if(!MessageWaitingAck(IR_MSG_TYPE_PROPAGATED))
    {
        int num_docked = 0;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i])
            {
                num_docked++;
                if(msg_unlocked_received & (1<<i))
                {
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
void RobotAW::Undocking()
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
        
        if((timestamp/2)%2==0)
            sidespeed = 10;
        else
            sidespeed = -10;

    }

}

void RobotAW::Transforming()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    if(timestamp == 40)
    {
        docked[0]=true;
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
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

    if(msg_transforming_received)
    {
        transforming_count++;
    }

    if(transforming_count==2)
    {
       SetHingeMotor(UP); 
    }

    if(transforming_count >=50)
    {
        current_state = MACROLOCOMOTION;
        last_state = TRANSFORMING;
    }


}

void RobotAW::Reshaping()
{
}


void RobotAW::MacroLocomotion()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = -20;

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

    if(macrolocomotion_count >=30)
    {
        leftspeed = 0;
        rightspeed = 0;
        sidespeed = 0;
        if(macrolocomotion_count ==35)
            SetHingeMotor(DOWN);
    }

    if(seed && macrolocomotion_count >= 150)
    {
        PropagateIRMessage(IR_MSG_TYPE_DISASSEMBLY);

        current_state = DISASSEMBLY;
        last_state = MACROLOCOMOTION;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if(docked[i])
                msg_unlocked_expected |=1<<i;
        }
    }
    else if(msg_disassembly_received)
    {
        current_state = DISASSEMBLY;
        last_state = MACROLOCOMOTION;

        msg_disassembly_received = 0;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if(docked[i])
                msg_unlocked_expected |=1<<i;
        }
    }
}

void RobotAW::Debugging()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;
    printf("Debuging %d:\t", para.debug.mode);

    switch (para.debug.mode)
    {
        case 0: //locking region threshold detection ?
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
            }
            printf("%d %d %d %d\n",  proximity[0], proximity[1], reflective_hist[0].Avg(), reflective_hist[1].Avg());
            break;
        case 1: //simulating recruitment, stage 2
            if(timestamp ==40)
            {
                SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
            }
            printf("%d %d %d %d\n",  proximity[4], proximity[5], ambient_calibrated[4]-ambient[4], ambient_calibrated[4]-ambient[5]);
            break;
        case 2://docking region threshold detection
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IR_PULSE0|IR_PULSE1);
            }
            printf("%d %d %d %d\n", reflective[0]-reflective_calibrated[0], reflective[1] - reflective_calibrated[1], beacon[0], beacon[1]);
            break;
        case 3: //simulate recruitment, stage 1, guiding signals
            if(timestamp ==40)
            {
                SetIRLED(2, IRLEDDOCKING, LED1, IR_PULSE0|IR_PULSE1);
            }
            printf("%d %d %d %d\n", reflective[4]-reflective_calibrated[4], reflective[5] - reflective_calibrated[5], ambient_calibrated[4]-ambient[4], ambient_calibrated[4]-ambient[5]);
            break;
        case 4: 
            if(timestamp ==40)
            {
            }
            break;
        case 5: 
            if(timestamp ==40)
            {
            }
            break;
        case 9:
            if(timestamp ==40)
            {
                MoveWheelsFront(para.debug.para[0], para.debug.para[1]);
                MoveWheelsRear(para.debug.para[0], para.debug.para[1]);
            }
            break;
        case 10:
            if(timestamp == para.debug.para[6])
            {
                printf("lifting hinge\n");
                SetHingeMotor(UP);
            }
            else if(timestamp == para.debug.para[7])
            {
                printf("lowing hinge\n");
                SetHingeMotor(DOWN);
            }
            break;
        default:
            break;
    }

}

int RobotAW::in_docking_region(int x[4])
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

int RobotAW::in_locking_region(int x[4])
{
    if( x[0] > para.locking_proximity_offset1 
            && x[1] > para.locking_proximity_offset2)
        return 1;
    else 
        return 0;
}
void RobotAW::Log()
{
    if (logFile.is_open())
    {
        logFile << timestamp << "\t" << state_names[current_state] <<"\t";
        logFile << reflective_hist[0].Avg()<<"\t";
        logFile << reflective_hist[1].Avg()<<"\t";
        logFile << beacon[0]<<"\t";
        logFile << beacon[1]<<"\t";
        logFile << ambient_hist[0].Avg()<<"\t";
        logFile << ambient_hist[1].Avg()<<"\t";
        logFile << proximity[0]<<"\t";
        logFile << proximity[1]<<"\t";
        logFile << std::endl;
    }

}

