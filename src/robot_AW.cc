#include "robot_AW.hh"

RobotAW::RobotAW(ActiveWheel *robot):Robot()
{
    if(name)
        free(name);
    name = strdup("RobotAW");
    type = ROBOT_AW;

    SPIVerbose = VERBOSE;
    irobot = robot;

    board_dev_num[::FRONT] = ActiveWheel::RIGHT; 
    board_dev_num[::RIGHT] = ActiveWheel::REAR; 
    board_dev_num[::BACK] = ActiveWheel::LEFT; 
    board_dev_num[::LEFT] = ActiveWheel::FRONT;   

    robot_side_dev_num[ActiveWheel::RIGHT] = ::FRONT;
    robot_side_dev_num[ActiveWheel::REAR] = ::RIGHT;
    robot_side_dev_num[ActiveWheel::LEFT] = ::BACK;
    robot_side_dev_num[ActiveWheel::FRONT] = ::LEFT;

    LED0 = 0x1;
    LED1 = 0x4;
    LED2 = 0x2;

    free_move = false;
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
        irobot->SetPrintEnabled(i, false);
        irobot->enableAccelerometer(i, true);
    }

    irobot->EnableMotors(true);

    //RobotBase::SetIRRX(board_dev_num[2], false);

}

void RobotAW::Reset()
{
    RobotBase::MSPReset();
}

void RobotAW::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    int board = board_dev_num[channel];
    irobot->SetIRLED(ActiveWheel::Side(board), led);
    irobot->SetIRPulse(ActiveWheel::Side(board), 0);
    //irobot->SetIRPulse(ActiveWheel::Side(board), pulse_led |IRPULSE2|IRPULSE3|IRPULSE4|IRPULSE5);
    irobot->SetIRMode(ActiveWheel::Side(board), mode);

    if(mode !=IRLEDOFF)
        irobot->SetIRRX(ActiveWheel::Side(board), led & LED1 ? false : true);
    else
        irobot->SetIRRX(ActiveWheel::Side(board), led & LED1 ? true : false);

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
    irobot->SetLED(ActiveWheel::Side(board), tl, br, bl, tr);
    RGBLED_status[channel] = tl|tr|bl|br;
}

void RobotAW::SetSpeed(int left, int right, int side)
{
    if(!para.locomotion_motor_enabled)
        return;

    if(left > 100)
        left = 100;
    else if(left < -100)
        left = -100;
    if(right > 100)
        right = 100;
    else if(right < -100)
        right = -100;
    if(side > 100)
        side = 100;
    else if(side < -100)
        side = -100;

    int frontleft=0;
    int frontright=0;
    int rear = 0;
    //free move
    if(free_move)
    {
        frontleft = left;
        frontright = right;
        rear = side;
    }
    else
    {
        //side move
        if(side != 0)
        {
            frontleft = side;
            frontright = side;
            rear = 0;
        }
        //approx differential move
        else
        {
            frontleft = left;
            frontright = -left;
            rear = -right * para.aw_adjustment_ratio;
        }
    }
    irobot->MoveWheelsFront(-direction * frontright, direction * frontleft);
    irobot->MoveWheelsRear(direction * rear,0);

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

    irobot->MoveHinge(45 * status);
    printf("\n\n------ moving hinge motor ----\n\n");
    return true;
}

bool RobotAW::MoveHingeMotor(int command[4])
{
    if(!para.hinge_motor_enabled)
        return false;

    //a valid hinge command
    if(command[3] == 1)
    {
        irobot->MoveHingeToAngle(command[0], command[1]);
    }
    else
    {
        irobot->MoveHinge(0);
    }

    return true;
}

bool RobotAW::RotateDockingUnit(int channel, int8_t angle)
{
    bool ret = true;

    if(channel == FRONT)
        irobot->MoveDockingSideDToAngle(angle);
    else if(channel == BACK)
        irobot->MoveDockingSideBToAngle(angle);
    else
        ret = false;

    return ret;
}

void RobotAW::UpdateSensors()
{
    IRValues ret_A = irobot->GetIRValues(ActiveWheel::RIGHT);
    IRValues ret_B = irobot->GetIRValues(ActiveWheel::FRONT);
    IRValues ret_C = irobot->GetIRValues(ActiveWheel::LEFT);
    IRValues ret_D = irobot->GetIRValues(ActiveWheel::REAR);
    ambient[0] = ret_A.sensor[0].ambient;
    ambient[1] = ret_A.sensor[1].ambient;
    reflective[0] = ret_A.sensor[0].reflective;
    reflective[1] = ret_A.sensor[1].reflective;
    proximity[0] = ret_A.sensor[0].proximity;
    proximity[1] = ret_A.sensor[1].proximity;
    beacon[0] = ret_A.sensor[0].docking;
    beacon[1] = ret_A.sensor[1].docking;
    ambient[2] = ret_B.sensor[0].ambient;
    ambient[3] = ret_B.sensor[1].ambient;
    reflective[2] = ret_B.sensor[0].reflective;
    reflective[3] = ret_B.sensor[1].reflective;
    proximity[2] = ret_B.sensor[0].proximity;
    proximity[3] = ret_B.sensor[1].proximity;
    beacon[2] = ret_B.sensor[0].docking;
    beacon[3] = ret_B.sensor[1].docking;
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

    aux_ambient[0] = ret_B.sensor[3].ambient;
    aux_ambient[1] = ret_B.sensor[2].ambient;
    aux_ambient[2] = ret_B.sensor[4].ambient;
    aux_ambient[3] = ret_B.sensor[5].ambient;
    aux_reflective[0] = ret_B.sensor[3].reflective;
    aux_reflective[1] = ret_B.sensor[2].reflective;
    aux_reflective[2] = ret_B.sensor[4].reflective;
    aux_reflective[3] = ret_B.sensor[5].reflective;
    aux_proximity[0] = ret_B.sensor[3].proximity;
    aux_proximity[1] = ret_B.sensor[2].proximity;
    aux_proximity[2] = ret_B.sensor[4].proximity;
    aux_proximity[3] = ret_B.sensor[5].proximity;
    aux_beacon[0] = ret_B.sensor[3].docking;
    aux_beacon[1] = ret_B.sensor[2].docking;
    aux_beacon[2] = ret_B.sensor[4].docking;
    aux_beacon[3] = ret_B.sensor[5].docking;
    aux_ambient[4] = ret_D.sensor[3].ambient;
    aux_ambient[5] = ret_D.sensor[2].ambient;
    aux_ambient[6] = ret_D.sensor[4].ambient;
    aux_ambient[7] = ret_D.sensor[5].ambient;
    aux_reflective[4] = ret_D.sensor[3].reflective;
    aux_reflective[5] = ret_D.sensor[2].reflective;
    aux_reflective[6] = ret_D.sensor[4].reflective;
    aux_reflective[7] = ret_D.sensor[5].reflective;
    aux_proximity[4] = ret_D.sensor[3].proximity;
    aux_proximity[5] = ret_D.sensor[2].proximity;
    aux_proximity[6] = ret_D.sensor[4].proximity;
    aux_proximity[7] = ret_D.sensor[5].proximity;
    aux_beacon[4] = ret_D.sensor[3].docking;
    aux_beacon[5] = ret_D.sensor[2].docking;
    aux_beacon[6] = ret_D.sensor[4].docking;
    aux_beacon[7] = ret_D.sensor[5].docking;

    uint8_t ethernet_status=0;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        //    color[i] = GetRGB(ScoutBot::Side(i));
        if(irobot->isEthernetPortConnected(ActiveWheel::Side(board_dev_num[i])))
            ethernet_status |= 1<<i;
    }
    ethernet_status_hist.Push2(ethernet_status);

    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-para.reflective_calibrated[i]);
        ambient_hist[i].Push(para.ambient_calibrated[i] - ambient[i]);
        aux_reflective_hist[i].Push(aux_reflective[i]-para.aux_reflective_calibrated[i]);
        aux_ambient_hist[i].Push(para.aux_ambient_calibrated[i] - aux_ambient[i]);
    }

    aux_bumped = 0;
    org_bumped = 0;
    for(int i=0;i<NUM_IRS;i++)
    {
        if(aux_reflective_hist[i].Avg() > para.avoid_threshold_aux[i])
            aux_bumped |= 1<<i;
        if(reflective_hist[i].Avg() > para.avoid_threshold[i])
            org_bumped |= 1<<i;
    }

    PrintAuxReflective();
}

void RobotAW::UpdateActuators()
{
    //CheckHingeMotor();
    SetSpeed(speed[0], speed[1], speed[2]); 
}

// for self-repair
void RobotAW::UpdateFailures()
{

    static int failure_delay = 0;
    if( !module_failed )
    {
        if( current_state == para.fail_in_state  )
        {
            if( failure_delay++ > para.fail_after_delay )
            {
                // For testing - send spoof IR message to self
                //                msg_failed_received |= 1<<para.debug.para[0]; // side received on
                //                subog_id = para.debug.para[1];				  // side sent from
                //                parent_side = para.debug.para[0];
                //                heading = (parent_side + 2) % 4;
                //
                //                // Propagate lowering messages
                //                PropagateIRMessage(IR_MSG_TYPE_LOWERING);
                //
                //                last_state = MACROLOCOMOTION;
                //                current_state = LOWERING;
                //                lowering_count = 0;
                //
                //                msg_unlocked_received |= 1<<para.debug.para[0];
                /////////////////////////////////////////////

                module_failed = true;
            }
        }
    }
}

void RobotAW::Avoidance()
{
    //for demo 
    static bool triggered = false;
    if(ambient_hist[0].Avg() > 1000 || ambient_hist[1].Avg() > 1000)
        triggered = true;
    else if(ambient_hist[4].Avg() > 1000 || ambient_hist[5].Avg() > 1000)
        triggered = false;
    if(!triggered)
    {
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;
        return;
    }

    speed[0] = 30;
    speed[1] = 30;
    speed[2] = 0;

    if(org_bumped || aux_bumped)
    {
        //front and rear are bumped 
        if(((org_bumped & (1<<0 | 1<<1)) !=0 ||(aux_bumped & (1<<0|1<<6||1<<7))!=0) && ((org_bumped & (1<<4 | 1<< 5))!=0||(aux_bumped & (1<<3|1<<4))!=0))
        {
            speed[0] = 0;
            speed[1] = 0;
            direction = FORWARD;
        }
        //only front is bumped
        else if((org_bumped & (1<<0 | 1<<1)) !=0 ||(aux_bumped & (1<<0|1<<6||1<<7))!=0) 
            direction  = BACKWARD;
        //only rear is bumped
        else if((org_bumped & (1<<4 | 1<< 5)) !=0 ||(aux_bumped & (1<<3|1<<4))!=0) 
            direction  = FORWARD;

        //move sideway if necessary
        if(((org_bumped & (1<<2 | 1<<3)) !=0 || (aux_bumped & (1<<1 | 1<<2))!=0 )&& ((org_bumped & (1<<6 | 1<<7)) !=0 ||(aux_bumped & (1<<5 | 1<<6))!=0 ))
            speed[2] = 0;
        else if((org_bumped & (1<<2 | 1<<3)) !=0 || (aux_bumped & (1<<1 | 1<<2))!=0)
            speed[2] = -direction * 30;
        else if((org_bumped & (1<<6 | 1<<7)) !=0 || (aux_bumped & (1<<5 | 1<<6))!=0)
            speed[2] = direction * 30;
    }
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
        mytree = target = para.og_seq_list[0];
        std::cout<<mytree<<std::endl;
    }
    else
        printf("Warning: empty organism sequence info\n");


    for(int i=0;i<SIDE_COUNT;i++)
    {
        recruitment_count[i] = 0;
        recruitment_signal_interval_count[i] = DEFAULT_RECRUITMENT_COUNT;
    }
    
    //start IPC thread, as a server
    master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);

    current_state = RECRUITMENT;
    last_state = SEEDING;
    
    msg_docking_signal_req_received = 0;

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
        SetIRLED(branch_side, IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1);
        std::cout<<name<<" branch "<<*it<<std::endl;
    }
}
void RobotAW::Foraging() //the same as RobotKIT
{
    speed[0]=0;
    speed[1]=0;
    speed[2]=0;

    foraging_count++;
    //time up?
    //
    /*
       if(foraging_count >= para.foraging_time)
       {
       foraging_count = 0;//DEFAULT_FORAGING_COUNT;
       waiting_count=0;//DEFAULT_WAITING_COUNT;

    //switch off all ir leds
    for(uint8_t i=0; i< NUM_DOCKS; i++)
    {
    SetIRLED(i, IRLEDOFF, LED1, 0x0);
    SetRGBLED(i, 0,0,0,0);
    }
    current_state = WAITING;
    last_state = FORAGING;

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    }
    else
    */
    {
        Avoidance();

        if(powersource_found)
        {
            current_state = LOCATEENERGY;
            last_state = FORAGING;
        }
        else if(organism_found || beacon_signals_detected)
        {
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

            current_state = ASSEMBLY;
            last_state = FORAGING;
        }

    }
}
void RobotAW::Waiting()//same as RobotKIT
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    msg_unlockme_received = 0;
    msg_locked_received = 0;
    msg_lockme_received = 0;

    waiting_count++;

    if(waiting_count >= (uint32_t)para.waiting_time)
    {
        waiting_count = 0;//DEFAULT_WAITING_COUNT;
        foraging_count = 0;//DEFAULT_FORAGING_COUNT;

        for(int i=0;i< NUM_IRS;i++)
            reflective_hist[i].Reset();

        //switch on proximity ir leds and ir pulsing
        for(uint8_t i=0; i< NUM_DOCKS; i++)
            SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);

        current_state = FORAGING;
        last_state = WAITING;
    }
    else if(organism_found || beacon_signals_detected)
    {
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

        assembly_count = 0;

        current_state = ASSEMBLY;
        last_state = WAITING;
    }
}
void RobotAW::Assembly()
{
    speed[0]=0;
    speed[1]=0;
    speed[2]=0;

    assembly_count++;

    if(assembly_count >= (uint32_t)para.assembly_time)
    {
        printf("assembly_count %d, para.assembly_time %d\n", assembly_count, para.assembly_time);
        organism_found = false;

        assembly_count = 0;

        current_state = FORAGING;
        last_state = ASSEMBLY;
    }
    //right type of recruitment message recived, then locatebeacon
    else if (assembly_info.type2 == type)
    {
        std::cout<<"assembly_info: "<<assembly_info<<"\tdirection: "<<direction<<std::endl;

        free_move = false;
        if(assembly_info.side2 == FRONT)
        {
            docking_approaching_sensor_id[0] = 0;
            docking_approaching_sensor_id[1] = 1;
            direction = FORWARD;
        }
        else
        {
            docking_approaching_sensor_id[0] = 4;
            docking_approaching_sensor_id[1] = 5;
            direction = BACKWARD;
        }

        current_state = LOCATEBEACON;
        last_state = FORAGING;

        locatebeacon_count = 0;
    }
    else if(assembly_info == OrganismSequence::Symbol(0) )
    {
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(timestamp % 12 == 3 * i && (beacon_signals_detected & (0x3 << (2*i)))!=0)
                BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING_REQ,0); 
        }
    }
    else
        Avoidance();
}
void RobotAW::LocateEnergy()//same as RobotKIT
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if(1)
    {
        current_state = SEEDING;
        last_state = LOCATEENERGY;
        return;
    }
}
void RobotAW::LocateBeacon()
{
    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    locatebeacon_count++;

    int turning;
    // If beacon detected on correct side - don't turn
    if((beacon_signals_detected & (1<<id0 | 1<<id1)) !=0)
        turning = 0;
    else
        turning = 1;

    if( turning == 0 )
    {
        //two beacon signals
        if(beacon_signals_detected_hist.Sum(id0) > 2 && beacon_signals_detected_hist.Sum(id1) > 2)
        {
            //stay there and wait to transfer to state Alignment
            speed[0] = 35;
            speed[1] = 35;
            speed[2] = 0;
        }
        else if(beacon_signals_detected_hist.Sum(id0) > 2 || beacon_signals_detected_hist.Sum(id1) > 2)
        {
            printf("only one beacon detected, shift left and right a little bit\n");
            int temp = beacon[id0]-beacon[id1];

            speed[0] = 0;
            speed[1] = 0;
            speed[2] = -30 * sign(temp);
        }
        else
        {
            speed[0] = 0;
            speed[1] = 0;
            speed[2] = 0;
        }
    }
    else
    {
        if(locatebeacon_count >=100)
        {
            speed[0] = -45;
            speed[1] = 30;
        }
    }

    printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tturning: %d\n", beacon[0], beacon[1], beacon[2], beacon[3],beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, turning);
    //overwrite speed if bumped to anything
    if(org_bumped || aux_bumped)
    {
        printf("%d bumped (%#x %#x) while try to detect beacon, zero speed\n", timestamp, org_bumped, aux_bumped);
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;
    }



    //checking
    if((beacon_signals_detected_hist.Sum(id0) >= 6 || beacon_signals_detected_hist.Sum(id1) >=6) && ethernet_status_hist.Sum(assembly_info.side2) > 4) 
    {
        SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
        docking_count = 0;
        docking_failed_reverse_count = 0;

        docking_blocked = false;

        current_state = DOCKING;
        last_state = LOCATEBEACON;

        SetRGBLED(0,0,0,0,0);
    }
    else if(beacon_signals_detected_hist.Sum(id0) >= 6 && beacon_signals_detected_hist.Sum(id1) >= 6)
    {
        current_state = ALIGNMENT;
        last_state = LOCATEBEACON;


        //using reflective signals if not set
        for(int i=0; i< NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED1, IRPULSE0 | IRPULSE1);

        return;
    }
    else
    {
        if(locatebeacon_count >= para.locatebeacon_time)
        {
            current_state = ASSEMBLY;
            last_state = LOCATEBEACON;
            locatebeacon_count = 0;

            organism_found = false;
            assembly_count = 0;
            assembly_info = OrganismSequence::Symbol(0);

            for(int i=0;i<NUM_DOCKS;i++)
            {
                SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);
                SetRGBLED(i, 0, 0, 0, 0);
            }
        }
    }



}

void RobotAW::Alignment()
{

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    int temp = beacon[id0]-beacon[id1];
    int temp_max = std::max(beacon[id0], beacon[id1]);
    int reflective_diff = (reflective_hist[id0].Avg())-(reflective_hist[id1].Avg());
    int reflective_min = std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
    int reflective_max = std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());

    if(beacon_signals_detected)
    {
        //signal interference, moving with caution
        if(reflective_min < -30)
        {
            if( abs(temp) > 0.25 * temp_max )
            {
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = -25 * sign(temp);
            }
            else
            {
                speed[0] = 20;
                speed[1] = 20;
                speed[2] = 0;
            }
            printf("Reflective signal interference - beacon: %d %d, reflective: %d %d, speed: %d %d %d\n",beacon[id0], beacon[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), speed[0], speed[1], speed[2]);

        }
        // Far away from recruiting robot - move sideways or forward
        else if( reflective_max < 30 )
        {
            if( abs(temp) > 0.1 * temp_max )
            {
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = -30 * sign(temp) ;
            }
            else
            {
                speed[0] = 35;
                speed[1] = 35;
                speed[2] = 0;
            }
            printf("FAR - beacon: %d %d, reflective: %d %d, speed: %d %d %d\n",beacon[id0], beacon[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), speed[0], speed[1], speed[2]);
        }
        //getting close to robots, but not too close
        else if( reflective_max < 800 )
        {
            if( abs(reflective_diff) > 150)
            {           
                speed[0] = 10 * sign(reflective_diff) * direction;
                speed[1] = -25 * sign(reflective_diff) * direction;
                speed[2] = 0;
                printf("Zone 1, adjusting orientation - beacon: %d %d, reflective: %d %d, speed: %d %d %d\n",beacon[id0], beacon[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), speed[0], speed[1], speed[2]);
            }
            else
            {
                if( abs(temp) > 0.25 * temp_max )
                {
                    speed[0] = 0;
                    speed[1] = 0;
                    speed[2] = -25 * sign(temp);
                    printf("Zone 1, adjusting pose - beacon: %d %d, reflective: %d %d, speed: %d %d %d\n",beacon[id0], beacon[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), speed[0], speed[1], speed[2]);
                }
                else
                {
                    speed[0] = 30;
                    speed[1] = 30;
                    speed[2] = 0;
                }
            }

        }
        // very close to another robots
        else
        {
            speed[0]=0;
            speed[1]=0;
            speed[2]=0;
            printf("Blocked or very close- beacon: %d %d, reflective: %d %d, speed: %d %d %d\n",beacon[id0], beacon[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), speed[0], speed[1], speed[2]);

            {
                SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                docking_region_detected = false;
                in_docking_region_hist.Reset();
                docking_count = 0;
                docking_failed_reverse_count = 0;


                docking_blocked = false;

                current_state = DOCKING;
                last_state = ALIGNMENT;

                SetRGBLED(0,0,0,0,0);
            }
        }

        //bumped on left and right, better to stop
        if((org_bumped & 0xCC) !=0 || (aux_bumped & 0xCC)!=0)
        {
            para.print_reflective = true;
            printf("%d something on my left or right, better to stop\n", timestamp);
            speed[0] = 0;
            speed[1] = 0;
            speed[2] = 0;
        }
        else
            para.print_reflective = false;

    }
    else
    {
        current_state = LOCATEBEACON;
        last_state = ALIGNMENT;

        locatebeacon_count = 0;
    }

}

void RobotAW::Recover()
{
    recover_count++;

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];
    //flasing RGB LEDs
    if(timestamp % 4 ==0)
    {
        SetRGBLED(1, RED, RED, RED, RED);
        SetRGBLED(2, RED, RED, RED, RED);
        SetRGBLED(3, RED, RED, RED, RED);
    }
    else
    {
        SetRGBLED(1, 0, 0, 0, 0);
        SetRGBLED(2, 0, 0, 0, 0);
        SetRGBLED(3, 0, 0, 0, 0);
    }

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if(last_state == ALIGNMENT)
    {
        //turn left/right according to reflective value;
        //robot will stop there for 1 seconds
        if(recover_count < para.aligning_reverse_time)
        {
            //docked to the wrong robots
            if(assembly_info == OrganismSequence::Symbol(0))
            {
                //just reverse
                speed[0] = para.aligning_reverse_speed[0];
                speed[1] = para.aligning_reverse_speed[1];
            }
            //failed
            else
            {
                if(timestamp % 5 ==0)
                    Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_DOCKING_SIGNALS_REQ, 0);

                speed[0] = para.aligning_reverse_speed[0];
                speed[1] = para.aligning_reverse_speed[1];

             }
        }
        else
        {
            SetRGBLED(1, 0, 0, 0, 0);
            SetRGBLED(3, 0, 0, 0, 0);
            if(assembly_info == OrganismSequence::Symbol(0))
            {
                ResetAssembly();
                
                for(int i=0; i<SIDE_COUNT; i++)
                    SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);
                
                current_state = FORAGING;
                last_state = RECOVER;
            }
            else
            {
                int id0 = docking_approaching_sensor_id[0];
                int id1 = docking_approaching_sensor_id[1];

                if(docking_trials >= para.docking_trials)
                {
                    ResetAssembly();
                    
                    for(int i=0; i<SIDE_COUNT; i++)
                        SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);
                   
                    docking_trials = 0;

                    current_state = FORAGING;
                    last_state = RECOVER;
                }
                else if(beacon_signals_detected_hist.Sum(id0) >= 5 && beacon_signals_detected_hist.Sum(id1) >= 5)
                {

                    current_state = ALIGNMENT;
                    last_state = RECOVER;
                }
                else
                {
                    if(timestamp % 5 ==0)
                        Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_DOCKING_SIGNALS_REQ, 0);
                }
            }
        }
    }

}

#define MOVE_LEFT 5
#define MOVE_RIGHT 6
void RobotAW::Docking()
{
    const char *docking_status_name[] ={"turn left", "turn right", "move forward", "move backward", "check" };

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    //TODO, remove this two static variables as they may cause problems
    static bool synchronised = false;
    static  int status = 0;
    static int status_count=0;
    docking_count++;

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    //docking done
    if(docking_region_detected)
    {
        //request for assembly_info
        if(msg_assembly_info_expected && timestamp % 10 ==0)
            Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_ASSEMBLY_INFO_REQ, 0);

        if(msg_assembly_info_received)
        {
            msg_assembly_info_received = 0;
            msg_assembly_info_expected = 0;

            if(assembly_info == OrganismSequence::Symbol(0))
            {
                for(int i=0; i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                current_state = RECOVER;
                last_state = ALIGNMENT;
                recover_count = 0;
            }
            else
            {
                docking_region_detected =false;
                docking_count = 0;
                docking_failed_reverse_count = 0;

                docking_blocked = false;
                blocking_count=0;


                SetRGBLED(assembly_info.side2, WHITE, WHITE, WHITE, WHITE);
                SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[assembly_info.side2]), false);

                Robot::SendIRMessage(assembly_info.side2, IR_MSG_TYPE_LOCKME,  para.ir_msg_repeated_num);
                msg_locked_expected |= 1<<assembly_info.side2;

                synchronised = false;
                current_state = LOCKING;
                last_state = DOCKING;

                SetRGBLED(assembly_info.side2, 0,0,0,0);
            }
        }
    }
    else if(ethernet_status_hist.Sum(assembly_info.side2) > 4) 
    {
        docking_region_detected = true;
        SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, 0);
        SetRGBLED(assembly_info.side2, WHITE,WHITE,WHITE,WHITE);

        msg_assembly_info_expected |= 1 << assembly_info.side2;

    }
    //request for guiding signals for a while,
    else if(docking_count < 30 && !synchronised)
    {
        if(robots_in_range_detected_hist.Sum(id0) < 5 && robots_in_range_detected_hist.Sum(id1) < 5 )
        {
            if(timestamp % 5 ==0)
                BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_GUIDEME, 0);

            synchronised = false;
        }
        else
        {
            //switch on the reflective signals
            synchronised = true;
            SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
        }

    }
    else
    {

        int reflective_diff = reflective_hist[id1].Avg() - reflective_hist[id0].Avg();
        int proximity_diff = proximity[id1] - proximity[id0];
        int proximity_max = std::max(proximity[id1],proximity[id0]);
        int proximity_min = std::min(proximity[id1],proximity[id0]);
        //if no signals detected, marked as failure
        //TODO, more failure case

        //blocked or something goes wrong
        if(docking_blocked)
        {
            docking_blocked = false;
            blocking_count=0;
            synchronised = false;
            docking_trials++;
            current_state = RECOVER;
            last_state = ALIGNMENT;
            recover_count = 0;
            //  printf("reflective: %d %d\t beacon:%d %d\n",reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), beacon[id0], beacon[id1]);
        }
        else
        {
            if(docking_count >= para.docking_time)
            {
                printf("docking_count %d reaches threshold\n", docking_count);
                docking_blocked = true;
            }
            else if(robots_in_range_detected_hist.Sum(id0) < 5 && robots_in_range_detected_hist.Sum(id1) < 5)
            {
                printf("No proximity signals detected\n");
                docking_blocked = true;
            }
            else if(abs(proximity_diff) > 0.8 * std::max(proximity[id0], proximity[id1]) )
            {
                printf("proximity signals are significant different %d %d\n", proximity[id0], proximity[id1]);
                docking_blocked = true;
            }
            else
            {

                if(timestamp % (DOCKING_CHECKING_INTERVAL) == 0)
                {
                    if( status != MOVE_FORWARD )
                        status = CHECKING;
                }
                else if(timestamp % (DOCKING_CHECKING_INTERVAL) == 2)
                {
                    status = CHECKING;
                }
                else if(timestamp % DOCKING_CHECKING_INTERVAL == 5 )
                {
                    if(abs(reflective_diff) > 1200 ) 
                        status = MOVE_BACKWARD;
                    else if(proximity_min > 1450 && abs(proximity_diff < 50))
                    {
                        status_count++;
                        if(status_count % 4 ==0)
                            status = MOVE_FORWARD;
                        else if(status_count % 4 ==1)
                            status = TURN_LEFT;
                        else if(status_count % 4 ==2)
                            status = MOVE_FORWARD;
                        else if(status_count % 4 ==3)
                            status = TURN_RIGHT;
                    }
                    else
                        status = MOVE_FORWARD;
                }

                switch (status)
                {
                    case TURN_RIGHT:
                        speed[0] = para.docking_turn_right_speed[0];
                        speed[1] = para.docking_turn_right_speed[1];
                        speed[2] = para.docking_turn_right_speed[2];
                        break;
                    case TURN_LEFT:
                        speed[0] = para.docking_turn_left_speed[0];
                        speed[1] = para.docking_turn_left_speed[1];
                        speed[2] = para.docking_turn_left_speed[2];
                        break;
                    case MOVE_FORWARD:
                        speed[0] = para.docking_forward_speed[0];
                        speed[1] = para.docking_forward_speed[1];
                        speed[2] = para.docking_forward_speed[2];
                        break;
                    case MOVE_BACKWARD:
                        speed[0] = para.docking_backward_speed[0];
                        speed[1] = para.docking_backward_speed[1];
                        speed[2] = para.docking_backward_speed[2];
                        break;
                    case CHECKING:
                        speed[0] = 0;
                        speed[1] = 0;
                        speed[2] = 0;
                        break;
                    default:
                        break;
                }
                printf("%d Docking routine %#x (%s) speed (%d %d %d) proximity (%d %d) reflective (%d %d)\n", timestamp, status, docking_status_name[status], speed[0], speed[1], speed[2], proximity[id0], proximity[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
            }
        }
    }

}

void RobotAW::Locking()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    int docking_side = assembly_info.side2;

    SetRGBLED(docking_side, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason

    if(msg_locked_received & (1<<docking_side)) //replace 0 with the correct docking face numer, currently 0 (FRONT)
    {
        msg_locked_received &= ~(1<<docking_side);
        docked[docking_side] = assembly_info.type2 | assembly_info.side2 << 2 | assembly_info.type1 << 4 | assembly_info.side1 << 6;//true;
        msg_organism_seq_expected = true;
        msg_subog_seq_expected |= 1<<docking_side;
        current_state = INORGANISM;
        last_state = LOCKING;
        //for(int i=0;i<NUM_DOCKS;i++)
        {
            // SetIRLED(docking_side, IRLEDOFF, LED0|LED2, 0); // switch off all ir pulse, using all LEDs  for communication as activewheel using the side receive
            // RobotBase::SetIRRX(board_dev_num[docking_side], false);
            SetRGBLED(docking_side, 0,0,0,0);
        }

        //start IPC thread, as a client 
        commander_IPC.Start(IPToString(commander_IP), commander_port, false);
    }
}
void RobotAW::Recruitment()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    std::vector<OrganismSequence>::iterator it1 = mybranches.begin();
    while(it1 !=mybranches.end())
    {
        uint8_t i = it1->getSymbol(0).side1;

        bool erase_required = false;
        if(recruitment_stage[i]==STAGE0)
        {
            if( robots_in_range_detected_hist.Sum(2*i) > 14 ||
                    robots_in_range_detected_hist.Sum(2*i+1) >14 ||
                    (msg_docking_signal_req_received & (1<<i)) )
            {
                msg_docking_signal_req_received &= ~(1<<i);

                msg_assembly_info_req_expected |= 1<<i;
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDDOCKING, LED1, 0);
                robots_in_range_detected_hist.Print();
                printf("%d -- Recruitment: channel %d  switch to Stage%d %#x\n\n", timestamp,i, recruitment_stage[i], msg_docking_signal_req_received);
            }
            else
            {
                //or send recruitment message
                SetIRLED(i, IRLEDOFF, LED1, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), false); //using side receiver
                SetRGBLED(i, 0,0,0,0);
                if(timestamp % RECRUITMENT_SIGNAL_INTERVAL == i)
                {
                    Robot::BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING, it1->getSymbol(0).data, 0);
                }
            }
        }
        else if(recruitment_stage[i]==STAGE1)
        {
            if(msg_recruiting_req_received & (1<<i))
            {
                msg_recruiting_req_received &= ~(1<<i);
                Robot::BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING, it1->getSymbol(0).data, 0);
            }

            if( recruitment_count[i]++ > para.recruiting_beacon_signals_time )
            {
                recruitment_count[i]=0;
                recruitment_stage[i]=STAGE0;
                SetIRLED(i,IRLEDOFF,LED1,0);
                printf("%d -- Recruitment: timeout! channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else if(msg_docking_signal_req_received & (1<<i))
            {
                recruitment_count[i]=0;
                msg_docking_signal_req_received &= ~(1<<i);
                SetIRLED(i, IRLEDDOCKING, LED1, IRPULSE0 | IRPULSE1);
            }
            else if(msg_assembly_info_req_received & (1<<i))
            {
                recruitment_count[i]=0;
                msg_locked_expected |= 1<<i;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0); 
                //wait for ack
                if(!MessageWaitingAck(i, IR_MSG_TYPE_ASSEMBLY_INFO))
                {
                    msg_assembly_info_req_received &= ~(1<<i);
                    guiding_signals_count[i]=0;
                    recruitment_stage[i]=STAGE2;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
                }
            }
            else if((msg_guideme_received & (1<<i)) )
            {
                recruitment_count[i]=0;
                msg_locked_expected |= 1<<i;
                msg_guideme_received &= ~(1<<i);
                recruitment_stage[i]=STAGE2;
                guiding_signals_count[i]=0;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                proximity_hist[2*i].Reset();
                proximity_hist[2*i+1].Reset();
                ambient_avg_threshold_hist.Reset();
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE2)
        {
            guiding_signals_count[i]++;
            msg_guideme_received &= ~(1<<i);

            if(guiding_signals_count[i] > 20 && msg_locked_received &(1<<i))
            {
                recruitment_stage[i]=STAGE3;
                guiding_signals_count[i] = 0;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), false);
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }

            if(guiding_signals_count[i] > 50 && (msg_docking_signal_req_received & (1<<i)))
            {
                msg_docking_signal_req_received &=~(1<<i);
                guiding_signals_count[i] = 0;
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  waits too long, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            //nothing happens, assumed wrong robots docked, back to stage 0
            else if(guiding_signals_count[i] >= (uint32_t)para.recruiting_guiding_signals_time)
            {
                guiding_signals_count[i] = 0;
                recruitment_stage[i]=STAGE0;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  waits too long, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE3)
        {
            if(msg_locked_received & (1<<i))
            {
                msg_locked_expected &=~(1<<i);
                msg_locked_received &=~(1<<i);
                msg_subog_seq_expected |= 1<<i;
                docked[i] = it1->getSymbol(0).data;

                SendBranchTree(i, (*it1)); //new in AW

                recruitment_stage[i]=STAGE4;
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            // When performing parallel docking, occasionally a robot will lock on a recruiting side and
            // that recruiting side will then revert to an earlier STAGE I think the following two clauses
            // cause this problem by picking up reflected messages from the other docking robot - LM
            else if(msg_docking_signal_req_received & 1<<i)
            {
                msg_docking_signal_req_received &=~(1<<i);
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  received docking signals req, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else if(msg_guideme_received & (1<<i))
            {
                msg_guideme_received &= ~(1<<i);
                proximity_hist[2*i].Reset(); //new in AW
                proximity_hist[2*i+1].Reset(); //new in AW
                ambient_avg_threshold_hist.Reset();
                recruitment_stage[i] = STAGE2;
                guiding_signals_count[i] = 0;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                printf("%d -- Recruitment: channel %d  switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE4) //new in AW
        {
            //recevied acks after sending sequence information?
            if(!MessageWaitingAck(i, IR_MSG_TYPE_ORGANISM_SEQ))
            {
                recruitment_stage[i] = STAGE5;
                docking_done[i] = false;

                if(seed)
                    num_robots_inorganism++;

                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i] == STAGE5)
        {
            //sending a ip_req first
            if(timestamp % 10 == i)
            {
                //request IP addr
                uint8_t data[5];
                data[0] = it1->getSymbol(0).data; //TODO: remove this as it is already included when using SendIRMessage
                memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP.i32, 4);
                Robot::SendIRMessage(i, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, 0);
            }
            //get new ip address?
            else if(msg_ip_addr_received & (1<<i))
            {
                std::vector<uint8_t> root_IPs;
                root_IPs.push_back(uint8_t((my_IP.i32 >>24) & 0xFF));
                root_IPs.push_back(uint8_t((neighbours_IP[i].i32>>24) & 0xFF));
                mytree.setBranchRootIPs(robot_side(i),root_IPs);
                printf("%d: set root IPs %d %d\n", timestamp,root_IPs[0], root_IPs[1]);

                //remove branches since it has been sent to newly joined robot
                erase_required = true;

                // Reset stage variable
                recruitment_stage[i] = STAGE0;
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
        msg_docking_signal_req_received = 0;

        printf("my IP is %s\n", IPToString(my_IP));
        for(int i=0;i<NUM_DOCKS;i++)
        {
            printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
            SetRGBLED(i, 0, 0, 0, 0);
        }

        if(seed)
        {
            commander_IP = my_IP;
            commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;
            commander_IPC.Start(IPToString(commander_IP), commander_port, false);
        }
    }
}
void RobotAW::InOrganism()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    //seed robot monitoring total number of robots in the organism
    if(seed)
    {
        if( mytree.Edges() + 1 == (unsigned int)num_robots_inorganism)
            printf("%d total number of robots in organism is %d\n", timestamp, mytree.Edges() + 1);

        if(mytree.isAllIPSet() && !IP_collection_done)
        {
            organism_formed = true;
            IP_collection_done = true;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            //prepare organism_formed_messages
            /*uint8_t buf[target.Encoded_Seq().size() + 2];
            for(int i = 0; i< target.Encoded_Seq().size(); i++)
                buf[i] = target.Encoded_Seq()[i].data;
            buf[target.Encoded_Seq().size()] = (my_IP.i32>>24)& 0xFF; //IP
            buf[target.Encoded_Seq().size() + 1] = COMMANDER_PORT;//commander port
            PropagateIRMessage(MSG_TYPE_ORGANISM_FORMED, buf, target.Encoded_Seq().size() + 2);
            PropagateEthMessage(MSG_TYPE_ORGANISM_FORMED, buf, target.Encoded_Seq().size() + 2);*/
            /*uint8_t data[2];
            data[0] = (my_IP.i32>>24) & 0xFF;
            data[1] =COMMANDER_PORT;
            PropagateIRMessage(MSG_TYPE_ORGANISM_FORMED, data, 2);
            PropagateEthMessage(MSG_TYPE_ORGANISM_FORMED, data, 2);
            */
            uint8_t buf[target.Encoded_Seq().size()];
            for(int i = 0; i< target.Encoded_Seq().size(); i++)
                buf[i] = target.Encoded_Seq()[i].data;
            IPCSendMessage(MSG_TYPE_ORGANISM_FORMED, buf, sizeof(buf));


            //init the client list and the acks
            pthread_mutex_lock(&IPC_data_mutex);
            for(unsigned int i=0;i<mytree.Encoded_Seq().size();i++)
            {
                if(mytree.Encoded_Seq()[i] != OrganismSequence::Symbol(0))
                    commander_acks[getFullIP(mytree.Encoded_Seq()[i].child_IP).i32] = 0;
            }
            pthread_mutex_unlock(&IPC_data_mutex);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %s\n", IPToString(my_IP));
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
            }

        }
    }
    //otherwise check if new info received
    else
    {
        if(msg_organism_seq_expected && msg_organism_seq_received)
        {
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

                msg_docking_signal_req_received = 0;
            }
        }
        else if(msg_organism_seq_received && (mytree.isAllIPSet()  && mytree.Size() !=0) && !IP_collection_done)
        {
            IP_collection_done = true;
            //send the IPs to its parent
            std::vector<uint8_t> IPs;
            mytree.getAllIPs(IPs);
            uint8_t data[IPs.size()+1];
            data[0]=IPs.size();
            for(int i=0;i<IPs.size();i++)
                data[i+1]=IPs[i];
            SendIRMessage(parent_side, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1, para.ir_msg_repeated_num);
            //SendEthMessage(parent_side, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1, true);
            IPCSendMessage(neighbours_IP[parent_side].i32, MSG_TYPE_IP_ADDR_COLLECTION, data, IPs.size() + 1);

        }
        else if(organism_formed)
        {
            msg_organism_seq_received  = 0;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %s\n", IPToString(my_IP));
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
            }

        }
    }
}
void RobotAW::Disassembly()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    //no need to propagate disassembling messages?
    if(!MessageWaitingAck(MSG_TYPE_PROPAGATED))
    {
        int num_docked = 0;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(docked[i])
            {
                num_docked++;
                if(msg_unlocked_received & (1<<i))
                {
                    docked[i]=0;//false;
                    num_docked--;
                }
            }

        }

        //only one  or less 
        if(num_docked ==0)
        {
            undocking_count = 0;
            current_state = UNDOCKING;
            last_state = DISASSEMBLY;
        }

    }

}
void RobotAW::Undocking()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(timestamp % 4 ==0)
            SetRGBLED(i, 0,0,0,0);
        else
            SetRGBLED(i, RED,RED,RED,RED);
    }

    if( undocking_count == 0 )
    {
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
    }
    else if(undocking_count < 100)
    {
        // Simply move forward
        //        speed[0] = 15;
        //        speed[1] = 15;
        //        speed[2] = 0;
    }
    else
    {
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;

        // Turn off LEDs
        for(int i=0;i<NUM_DOCKS;i++)
            SetRGBLED(i, 0,0,0,0);

        last_state = UNDOCKING;
        current_state = FORAGING;
        ResetAssembly(); // reset variables used during assembly
                
        current_state = fsm_state_t(para.init_state);

        for(int i=0; i<SIDE_COUNT; i++)
            SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);
    }

    undocking_count++;


}

void RobotAW::Lowering()
{
    lowering_count++;
    
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    if(seed)
    {
        int lowering_delay = 10;
        if(lowering_count < lowering_delay)
        {
            //waiting;
            if(broken_eth_connections > 0)
                IPC_health = false;

            memset(hinge_command, 0, sizeof(hinge_command));
        }
        else if(IPC_health)
        {
            hinge_motor_operating_count++;

            if(hinge_motor_operating_count < (uint32_t)para.hinge_motor_lowing_time)
            {
                hinge_command[0] = 7;
                hinge_command[1] = para.hinge_motor_speed;
                hinge_command[2] = hinge_motor_operating_count ;
                hinge_command[3] = 1; //this indicates the validation of command
                IPCSendMessage(IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)hinge_command, sizeof(hinge_command));
            }
            else
            {
                hinge_motor_operating_count = 0;

                //IPCSendMessage(MSG_TYPE_DISASSEMBLY, NULL, 0);
                //
                if(!msg_raising_start_received) //this will prevent the message being sent twice 
                {
                    IPCSendMessage(IPC_MSG_RAISING_START, NULL, 0);
                    msg_raising_start_received = true;
                }
                
                lowering_count = 0;
            }

            //check if all robot 
            if(lowering_count % 5 == 4)
            {
                std::map<uint32_t, int>::iterator it;
                for(it = commander_acks.begin(); it != commander_acks.end(); it++)
                {
                    //check if lost received some messages
                    if(it->second < 2) //2 out of 5
                    {
                        IPC_health = false;
                        printf("%d : ip: %s acks %d\n", timestamp, IPToString(it->first), it->second );
                    }
                    //reset the count
                    it->second = 0;
                }
            }

        }
        else
        {
            printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
            memset(hinge_command, 0, sizeof(hinge_command));
        }
    }


    if(msg_disassembly_received)
    {
        current_state = DISASSEMBLY;
        last_state = LOWERING;
        lowering_count = 0;

        msg_disassembly_received = false;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if(docked[i])
                msg_unlocked_expected |= 1<<i;
        }
        
        memset(hinge_command, 0, sizeof(hinge_command));
    }
    else if(msg_raising_start_received)
    {
        msg_raising_start_received = false;
        macrolocomotion_count = 0;
        raising_count = 0;
        lowering_count = 0;
        current_state = RAISING;
        last_state = LOWERING;
        
        memset(hinge_command, 0, sizeof(hinge_command));
    }

    MoveHingeMotor(hinge_command);
    
    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

 }


void RobotAW::Raising()
{
    //wait until all propapagated messages are gone
    if(MessageWaitingAck(MSG_TYPE_PROPAGATED))
        return;

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    
    raising_count++;

    // Leds symbolise the raising process
    bool flash_leds = false;

    // Wait longer with larger structures
    int raising_delay = 10;//(mytree.Size()/2+1)*30;

    if(seed)
    {
        //wait for a while until the propagated messages are done within the organism
        if(raising_count < raising_delay)
        {
            //waiting;
            if(broken_eth_connections > 0)
                IPC_health = false;

            memset(hinge_command, 0, sizeof(hinge_command));
        }
        else if(IPC_health)
        {
            hinge_motor_operating_count++;

            if(hinge_motor_operating_count < (uint32_t)para.hinge_motor_lifting_time)
            {
                hinge_command[0] = para.hinge_motor_angle;
                hinge_command[1] = para.hinge_motor_speed;
                hinge_command[2] = hinge_motor_operating_count ;
                hinge_command[3] = 1; //this indicates the validation of command
                IPCSendMessage(IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)hinge_command, sizeof(hinge_command));
            }
            else
            {
                if(!msg_raising_stop_received) //this will prevent the message being sent twice 
                {
                    printf("%d: send raising stop\n", timestamp);
                    IPCSendMessage(IPC_MSG_RAISING_STOP,NULL, 0);
                    msg_raising_stop_received = true;
                    IPCSendMessage(IPC_MSG_RESET_POSE_REQ,NULL, 0);
                }


                InitRobotPoseInOrganism();

                IPC_health = true;
                
             }

            //check if all robot 
            if(raising_count % 5 == 4)
            {
                std::map<uint32_t, int>::iterator it;
                for(it = commander_acks.begin(); it != commander_acks.end(); it++)
                {
                    //check if lost received some messages
                    if(it->second < 2) //2 out of 5
                    {
                        IPC_health = false;
                        printf("%d : ip: %s acks %d\n", timestamp, IPToString(it->first), it->second );
                    }
                    //reset the count
                    it->second = 0;
                }
            }

        }
        else
        {
            printf("%d: not all robots are reachable through ethernet, something is wrong, stop moving\n", timestamp);
            flash_leds = true;
            memset(hinge_command, 0, sizeof(hinge_command));
        }
    }


    if( msg_raising_stop_received )
    {
        printf("%d: recieved raising stop\n", timestamp);

        msg_raising_stop_received = false;
        current_state = MACROLOCOMOTION;
        last_state = RAISING;
        raising_count = 0;
        flash_leds = false;
        hinge_motor_operating_count=0;
        macrolocomotion_count = 0;

        memset(hinge_command, 0, sizeof(hinge_command));


        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED2, IRPULSE0|IRPULSE1);

    }
    else if( msg_raising_start_received )
    {
        msg_raising_start_received = false;
        flash_leds = true;
    }

    
    MoveHingeMotor(hinge_command);
    
    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

    if(flash_leds)
    {
        //flashing RGB leds
        static int index = 0;
        index = (timestamp / 2) % 6;
        for(int i=0;i<NUM_DOCKS;i++)
        {
            switch (index)
            {
                case 0:
                    SetRGBLED(i, RED, RED, 0, 0);
                    break;
                case 1:
                    SetRGBLED(i, 0, 0, 0, 0);
                    break;
                case 2:
                    SetRGBLED(i, 0, 0, RED, RED);
                    break;
                case 3: //
                case 4: // short delay to better symbolise raising
                case 5: //
                    SetRGBLED(i, 0, 0, 0, 0);
                    break;
                default:
                    break;
            }
        }
    }
}

/*
 * Can be used outside of self-repair too. To convert an organism to another shape just make
 *  sure that only one robot is the seed and that that robot's tree (mytree) reflects the
 *  desired shape. Can also be used to initiate disassembly if the seed robot's tree is empty.
 */
void RobotAW::Reshaping()
{

    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    // If this is the seed or branch received from other module
    if( seed || msg_organism_seq_received )
    {
        // Prepare branch sequences
        rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
        if(ret.status >= RT_ERROR)
        {
            std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        // disable all LEDs
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0x0);

        for(int i=0; i<SIDE_COUNT; i++)
        {
            recruitment_stage[i]=STAGE0;
            recruitment_count[i] = 0;
            recruitment_signal_interval_count[i] = DEFAULT_RECRUITMENT_COUNT;

            // unless this is the seed do not
            // send messages to parent_side
            if( !seed && i == parent_side )
                continue;

            // check if branch needs to be sent
            uint8_t branch_side = SIDE_COUNT;
            OrganismSequence next_branch;
            std::vector<OrganismSequence>::iterator it;
            for(it = mybranches.begin() ; it != mybranches.end(); it++)
            {
                if( it->getSymbol(0).side1 == i )
                {
                    branch_side = it->getSymbol(0).side1;
                    next_branch = (*it);
                    break;
                }
            }

            // TODO: Check that neighbour is correct type and orientation!

            // if there is a neighbour and there should be
            if( docked[i] && branch_side == i )
            {
                // send branch
                SendBranchTree(i, next_branch);
                recruitment_stage[i]=STAGE4;
                printf("%d Sending branch to side %d\n",timestamp, i);
            }
            // if there is a neighbour but there shouldn't be
            else if( docked[i] && branch_side == SIDE_COUNT )
            {
                // send disassembly
                PropagateSingleIRMessage(MSG_TYPE_DISASSEMBLY,i);
                printf("%d Instructing module on side %d to disassemble\n",timestamp, i);
            }
            // if there isn't a neighbour but there should be
            else if( docked[i]==0 && branch_side == i )
            {
                // start recruiting
                SetIRLED(branch_side, IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1);
                printf("%d Preparing to recruit upon side %d\n",timestamp,i);
            }

        }

        // If there is no new tree - disassemble
        if( seed && mytree.Size() <= 0 )
        {
            current_state = DISASSEMBLY;
            printf("%d No new tree to assemble, entering disassembly\n",timestamp);
        }
        else
        {
            current_state = RECRUITMENT;

            msg_docking_signal_req_received = 0;

            // turn off LEDs
            for(int i=0; i<NUM_DOCKS; i++)
                SetRGBLED(i, 0, 0, 0, 0);
        }

        last_state = RESHAPING;

    }
    else if( msg_disassembly_received )
    {
        printf("%d: received disassembly start\n", timestamp);
        current_state = DISASSEMBLY;
        last_state = RESHAPING;

        msg_disassembly_received = false;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if( docked[i] )
                msg_unlocked_expected |=1<<i;
        }
    }

}


void RobotAW::MacroLocomotion()
{
    // Stop moving
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    macrolocomotion_count++;

    if(seed)
    {
     //   PrintOGIRSensor(IR_REFLECTIVE_DATA);
        //request IRSensors
        RequestOGIRSensors(IR_REFLECTIVE_DATA);

        int cmd_speed[3] = {0,0,0};

        //make a decision for the speed of organism
        direction = FORWARD;

        uint8_t organism_bumped = 0;
        //check front and back side
        for(int i=0;i<2;i++)
        {
            if(og_reflective_sensors.front[i] > 2000)
                organism_bumped |= 1;
            if(og_reflective_sensors.back[i] > 2000)
                organism_bumped |= 1<<2;
        }

        if((organism_bumped & 0x5) == 0x5) //both front and back are bumped
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            direction = FORWARD;
        }
        else
        {
            cmd_speed[0] = 30;
            cmd_speed[1] = 30;

            if((organism_bumped & 0x5) == 0x1)//front bumped
                direction = BACKWARD;
            else if((organism_bumped & 0x5) == 0x4)//back bumped
                direction = FORWARD;
        }

        //check left and right side
        for(int i=0;i<og_reflective_sensors.left.size();i++)
        {
            if(og_reflective_sensors.left[i] > 2000)
                organism_bumped |= 1<<1;
            if(og_reflective_sensors.right[i] > 2000)
                organism_bumped |= 1<<3;
        }

        //left and right
        if((organism_bumped & 0xA) == 0xA) //both left and right are bumped
            cmd_speed[2] = 0;
        else if((organism_bumped & 0xA) == 0x2) //left bumped
            cmd_speed[2] = -60;
        else if((organism_bumped & 0xA) == 0x8) //right bumped
            cmd_speed[2] = 60;
        else
            cmd_speed[2] = 0;


        /*
        if(macrolocomotion_count < 50)
        {
            cmd_speed[0] = 30;
            cmd_speed[1] = 30;
            cmd_speed[2] = 0;
        }
        else if(macrolocomotion_count < 100)
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            cmd_speed[2] = 60;
        }
        else if(macrolocomotion_count < 150)
        {
            cmd_speed[0] = -30;
            cmd_speed[1] = -30;
            cmd_speed[2] = 0;
        }
        else if(macrolocomotion_count < 200)
        {
            cmd_speed[0] = 0;
            cmd_speed[1] = 0;
            cmd_speed[2] = -60;
        }
        else
        */
        if(macrolocomotion_count > 50)
        {
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;
            cmd_speed[0] = 0;

            if(!msg_climbing_start_received) //this will prevent the message being sent twice 
            {
                printf("%d: send climbing start\n", timestamp);
                IPCSendMessage(IPC_MSG_CLIMBING_START, NULL, 0);
                //IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                msg_climbing_start_received = true; //a dirty fix to prevent message being sent twice as ethernet delay
            }

            IPC_health = true;
            climbing_count =0;
            macrolocomotion_count = 0;
            hinge_motor_operating_count = 0;

        }


        //set the speed of all AW robots in the organism
        std::map<uint32_t, robot_pose>::iterator it;
        for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
        {
            int motor_command[4];
            if(it->second.type == ROBOT_AW)
            {
                motor_command[0] = direction * it->second.direction;
                motor_command[1] = motor_command[0] >0 ? cmd_speed[0] : cmd_speed[1];
                motor_command[2] = motor_command[0] >0 ? cmd_speed[1] : cmd_speed[0];
                motor_command[3] = cmd_speed[2];
                IPCSendMessage(it->first, IPC_MSG_LOCOMOTION_2D_REQ, (uint8_t*)motor_command, sizeof(motor_command));
            }
            else
            {
                motor_command[0] = direction * it->second.direction;
                motor_command[1] = 0;
                motor_command[2] = 0;
                motor_command[3] = 0;
            }
        }
    }
    
    if( msg_lowering_received )
    {       
        printf("%d: received lowing start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        msg_lowering_received = false;
        last_state = MACROLOCOMOTION;
        current_state = LOWERING;
        lowering_count = 0;
        macrolocomotion_count=0;
    }
    else if (msg_climbing_start_received)
    {
        printf("%d: received climbing start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        msg_climbing_start_received = false;
        last_state = MACROLOCOMOTION;
        current_state = CLIMBING;
        climbing_count = 0;
        macrolocomotion_count=0;
        hinge_motor_operating_count = 0;
    }

    direction = locomotion_command[0] == 0 ? FORWARD : locomotion_command[0];
    speed[0] = locomotion_command[1];
    speed[1] = locomotion_command[2];
    speed[2] = locomotion_command[3];

    //reset
    if(timestamp - timestamp_locomotion_motors_cmd_received > 3)
        memset(locomotion_command, 0, sizeof(locomotion_command));

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


}

void RobotAW::Climbing()
{
    climbing_count++;

    if(seed)
    {
        direction = FORWARD;

        if(current_action_sequence_index < organism_actions.size())
        {
            action_sequence * as_ptr = &organism_actions[current_action_sequence_index];
            as_ptr->counter++;

            //check if front_aw_ip is initialised fron the beginning
            if(current_action_sequence_index == 0 && front_aw_ip ==0)
            {
                std::map<uint32_t, robot_pose>::iterator it;
                bool flag = false;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    if(it->second.og_irsensor_index == 0)
                    {
                        flag = true;
                        front_aw_ip = it->first;
                        break;
                    }

                }
                if(!flag)
                    front_aw_ip = 0;
            }

            //end of life?
            if(as_ptr->counter >= as_ptr->duration)
            {
                current_action_sequence_index++;
                as_ptr->counter = 0; //reset the counter
                printf("%d the finished command is %s (%d)\n", timestamp, as_ptr->cmd_type == 0 ? "PUSH_DRAG":"LIFT_ONE", as_ptr->sequence_index);
              //  memset(hinge_command, 0, sizeof(hinge_command));
              //  memset(locomotion_command, 0, sizeof(locomotion_command));
                if(current_action_sequence_index < organism_actions.size())
                {
                    //if the latest action sequence is lifting, then next one should be push-drag
                    if(as_ptr->cmd_type == action_sequence::CMD_LIFT_ONE)
                    {
                        std::map<uint32_t, robot_pose>::iterator it;
                        bool flag = false;
                        for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                        {
                            if(it->second.og_irsensor_index == robot_pose_in_organism[front_aw_ip].og_irsensor_index + 1)
                            {
                                flag = true;
                                front_aw_ip = it->first;
                                break;
                            }
                        }

                        if(!flag)
                            front_aw_ip = 0;
                    }
                    printf("%d next command is %s (%d)\n", timestamp, organism_actions[current_action_sequence_index].cmd_type == 0 ? "PUSH_DRAG":"LIFT_ONE", organism_actions[current_action_sequence_index].sequence_index);
                }
                else
                    front_aw_ip = 0;

            }
            else
            {

                //request og_front_aux_reflective_sensor
                if(front_aw_ip != 0 && as_ptr->cmd_type == action_sequence::CMD_PUSH_DRAG)
                {
                    //printf("I am request aux_reflective_data from %s\n", IPToString(front_aw_ip));
                    RequestOGIRSensors(front_aw_ip, IR_AUX_REFLECTIVE_DATA);
                    //PrintOGIRSensor(IR_AUX_REFLECTIVE_DATA);
                }
                else
                {
                    memset(og_front_aux_reflective_sensors, 0 , sizeof(og_front_aux_reflective_sensors));
                }

                //set the command for each robots in the organism
                for(int i=0;i<as_ptr->robots_in_action.size();i++)
                {
                    uint32_t robot_ip=robot_in_organism_index_sorted[as_ptr->robots_in_action[i].index];
                    //printf("Send command [%d] to %s\n",as_ptr->cmd_type, IPToString(robot_ip));
                    int motor_command[4];


                    if(as_ptr->cmd_type == action_sequence::CMD_PUSH_DRAG)
                    {
                        if(og_front_aux_reflective_sensors[0] > 3000 ||
                           og_front_aux_reflective_sensors[1] > 3000 ||
                           og_front_aux_reflective_sensors[2] > 3000 ||
                           og_front_aux_reflective_sensors[3] >3000)
                        {
                            motor_command[0] = 0;
                            motor_command[1] = 0;
                            motor_command[2] = 0;
                            motor_command[3] = 0;
                            printf("%d: stop! no need to move forward as AW detects the edge of the stairs\n");
                        }
                        else
                        {
                            motor_command[0] = direction * robot_pose_in_organism[robot_ip].direction;
                            motor_command[1] = motor_command[0] >0 ? as_ptr->robots_in_action[i].cmd_data[0] : as_ptr->robots_in_action[i].cmd_data[1] ;
                            motor_command[2] = motor_command[0] >0 ? as_ptr->robots_in_action[i].cmd_data[1] : as_ptr->robots_in_action[i].cmd_data[0] ;
                            motor_command[3] = as_ptr->robots_in_action[i].cmd_data[2];
                        }
                        IPCSendMessage(robot_ip, IPC_MSG_LOCOMOTION_2D_REQ, (uint8_t*)motor_command, sizeof(motor_command));
                    }
                    else if(as_ptr->cmd_type == action_sequence::CMD_LIFT_ONE)
                    { 
                        if(as_ptr->counter + 1 >= as_ptr->duration)
                        {
                            //send a stop hinge command;
                            memset(motor_command, 0, sizeof(motor_command));
                        }
                        else
                        {
                            motor_command[0] = as_ptr->robots_in_action[i].cmd_data[0];
                            motor_command[1] = as_ptr->robots_in_action[i].cmd_data[1];
                            motor_command[2] = as_ptr->robots_in_action[i].cmd_data[2];
                            motor_command[3] = 1; //this indicates the validation of command
                        }

                        IPCSendMessage(robot_ip, IPC_MSG_HINGE_3D_MOTION_REQ, (uint8_t*)motor_command, sizeof(motor_command));
                    }

                }
            }
        }
        else
        {
            // Stop moving
            memset(hinge_command, 0, sizeof(hinge_command));
            memset(locomotion_command, 0, sizeof(locomotion_command));

            front_aw_ip = 0; //reset it to the right value 
            
            if(!msg_lowering_received)
            {
                printf("%d: send lowering start\n", timestamp);
                IPCSendMessage(MSG_TYPE_LOWERING, NULL, 0);
                msg_lowering_received = true;
            }
        }
    }


    if( msg_lowering_received )
    {
        printf("%d: received lowering start\n", timestamp);
        // Stop moving
        memset(hinge_command, 0, sizeof(hinge_command));
        memset(locomotion_command, 0, sizeof(locomotion_command));

        current_action_sequence_index =0;
        msg_lowering_received = false;
        last_state = CLIMBING;
        current_state = LOWERING;
        climbing_count =0;
        macrolocomotion_count=0;
        hinge_motor_operating_count = 0;
    }

    //2d locomotion will be called automatially
    direction = locomotion_command[0] == 0 ? FORWARD: locomotion_command[0];
    speed[0] = locomotion_command[1];
    speed[1] = locomotion_command[2];
    speed[2] = locomotion_command[3];

    //move hinge
    MoveHingeMotor(hinge_command);

    //reset if no cmd received, to be used to stop the motor automatically
    if(timestamp - timestamp_hinge_motor_cmd_received > 3)
        memset(hinge_command, 0, sizeof(hinge_command));

    if(timestamp - timestamp_locomotion_motors_cmd_received > 3)
        memset(locomotion_command, 0, sizeof(locomotion_command));
}

void RobotAW::Debugging()
{
    //speed[0] = 0;
    //speed[1] = 0;
    //speed[2] = 0;
    //  printf("Debuging %d:\n", para.debug.mode);

    static int clock=0;
    static bool log = false;
    Log();

    switch (para.debug.mode)
    {
        case 0: //locking region threshold detection ?
            if(timestamp ==2)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);//IRPULSE0|IRPULSE1);
            }
            printf("%d %d %d %d\n",  proximity[0], proximity[1], reflective_hist[0].Avg(), reflective_hist[1].Avg());
            break;
        case 1: //simulating recruitment, stage 2, 64Hz helper signals
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[9], IRLEDPROXIMITY, LED0|LED2, IRPULSE0|IRPULSE1); //switch docking signals 2 on left and right leds
            }
            printf("%d %d %d %d\n",  proximity[4], proximity[5], para.ambient_calibrated[4]-ambient[4], para.ambient_calibrated[4]-ambient[5]);
            break;
        case 2://docking region threshold detection
            if(timestamp ==2)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            }
            printf("%d %d %d %d\n", reflective[0]-para.reflective_calibrated[0], reflective[1] - para.reflective_calibrated[1], beacon[0], beacon[1]);
            break;
        case 3: //simulate recruitment, stage 1, 32Hz guiding signals
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[9], IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1);
            }
            printf("%d %d %d %d\n", reflective[0]-para.reflective_calibrated[0], reflective[1] - para.reflective_calibrated[1], para.ambient_calibrated[0]-ambient[0], para.ambient_calibrated[1]-ambient[1]);
            break;
        case 4:// recruiting stage 2 -> stage 3 detection
            if(timestamp ==2)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED2, IRPULSE0|IRPULSE1); //switch docking signals 2 on left and right leds
            }
            break;
        case 5:// simulate locking stage, turn on RGB led to be bright 
            if(timestamp ==2)
            {
                SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason
                SetIRLED(0, IRLEDOFF, LED0|LED2, 0);
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[0]), false);
            }
            break;
        case 6: //testing request ip via ircomm
            if(timestamp == 40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), false);
                }
            }
            break;
        case 9:
            if(timestamp ==para.debug.para[6])
            {
                printf("lifting hinge\n");
                irobot->MoveHingeToAngle( hinge_start_pos+5, hinge_speed );
            }
            else if(timestamp == para.debug.para[7])
            {
                printf("lifting hinge\n");
                irobot->MoveHingeToAngle( hinge_start_pos, hinge_speed );
            }

            break;
        case 10:
            if(timestamp % 70 == para.debug.para[5])
            {
                printf("lifting hinge\n");
                SetHingeMotor(UP);
            }
            else if(timestamp % 70 == para.debug.para[6])
            {
                printf("lowing hinge\n");
                SetHingeMotor(DOWN);
            }
            break;
            // For testing self-repair - starting from MacroLocomotion
        case 11:
            if( timestamp < 40 )
                return;

            if(timestamp == 40)
            {
                // Setup LEDs and receivers
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), true);
                }

                int num_neighbours = para.debug.para[2];
                msg_subog_seq_expected = 0;
                msg_unlocked_expected = 0;
                for( int i=0; i<num_neighbours; i++ )
                {
                    uint8_t side = para.debug.para[3+(i*3)];
                    uint8_t n_type = para.debug.para[4+(i*3)];
                    uint8_t n_side = para.debug.para[5+(i*3)];
                    docked[side] = n_type | n_side << 2 | type << 4 | side << 6;
                    msg_subog_seq_expected |= 1 << side;
                    msg_unlocked_expected |= 1 << side;
                    printf("%d neighbour %c docked on side %c using side %c\n",timestamp,robottype_names[n_type],side_names[side],side_names[n_side]);
                }

                target = para.og_seq_list[0];
                std::cout << timestamp << " Target Shape: " << target << std::endl;

                current_state = LOWERING;
            }
            break;
        case 12:
            if(timestamp==40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    irobot->SetIRRX(ActiveWheel::Side(board_dev_num[i]), false);
                }
            }
            break;
        case 13:
            if(timestamp==40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    printf("%d - Side %d connected: %s activated: %s\n", timestamp, i, irobot->isEthernetPortConnected(ActiveWheel::Side(board_dev_num[i])) ? "true":"false",irobot->isSwitchActivated()?"true":"false" );
                }
            }
#define NEIGHBOUR_IP "192.168.0.7"
            if(timestamp % 10 ==0)
            {
                uint8_t data[9]={'h','e','l','l','o','-','A','W',0};
                irobot->SendEthMessage(StringToIP(NEIGHBOUR_IP), data, sizeof(data));
            }
            while (irobot->HasEthMessage() > 0)
            {
                uint8_t rx[32];
                auto_ptr<Message> m = irobot->ReceiveEthMessage();
                memcpy(rx, m->GetData(), m->GetDataLength());
                printf("%d -- received data: %s\n", timestamp, rx);
            }
            break;
        case 14: //as recruting robot for measuring
            if(timestamp ==32)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("AFKF");
                docked[0]=sym.data;
                //using reflective signals if not set
                for(int i=0; i< NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0 | IRPULSE1);
            }

            //received IP_REQ as synchronisation signals
            if(neighbours_IP[0] != 0)
            {
                clock++;
            }

            //start flashing ir led
            if(clock == 2)
            {
                //SetIRLED(0, IRLEDDOCKING, LED1, IRPULSE0 | IRPULSE1);
                SetIRLED(0, IRLEDPROXIMITY, LED0|LED2, 0);
            }

            if(clock == para.debug.para[5])
                log = true;
            else if(clock == para.debug.para[6])
                log = false;

            if(log)
                Log();
            break;
        case 15: //as docking robot for measuring
            if(timestamp ==32)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("AFKB");
                docked[0]=sym.data;
                //using reflective signals if not set
                for(int i=0; i< NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0 | IRPULSE1);

                SetRGBLED(2, RED,RED,RED,RED);
                //                SetRGBLED(0, WHITE,WHITE,WHITE,WHITE);
            }

            //send synchronisation signals, using ip_req
            if(msg_ip_addr_received==0)
            {
                if(timestamp % 20 ==5)
                {
                    uint8_t data[5];
                    data[0] = docked[0]; //TODO: remove this as it is already included when using SendIRMessage
                    memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP.i32, 4);
                    Robot::SendIRMessage(0, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, 0);
                }
            }
            else
                clock++;

            if(clock == para.debug.para[5])
            {
                log = true;
                speed[0] = -15;
                speed[1] = -15;
                speed[2] = 0;
            }
            else if(clock == para.debug.para[6])
            {
                log = false;
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 0;
            }
            if(log)
                Log();
            break;
        case 16://Test IRComm as sender
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[0], IRLEDOFF, LED0|LED1|LED2, IRPULSE0 | IRPULSE1); //TODO: better to switch off ir pulse
                SetRGBLED(para.debug.para[0], 0,0,0,0);
            }
            if(timestamp % RECRUITMENT_SIGNAL_INTERVAL == 0)
            {
                OrganismSequence::Symbol sym(0);
                sym.reBuild("AFSF");
                Robot::BroadcastIRMessage(para.debug.para[0], IR_MSG_TYPE_RECRUITING, sym.data, 0);
            }
            break;
        case 17://Test IRComm as listener
            if(timestamp ==2)
                irobot->SetIRRX(ActiveWheel::Side(board_dev_num[para.debug.para[0]]), para.debug.para[1]);
            break;
        case 18://print out sensor data
            if(timestamp ==2)
            {
                for(int i=0;i<SIDE_COUNT;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            }
            break;
        case 19://print out sensor data
            if(timestamp ==2)
            {
                for(int i=0;i<SIDE_COUNT;i++)
                    SetIRLED(i, IRLEDPROXIMITY, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            }
            break;
        case 20://testing motors
            // if(timestamp  == 2)
            { 
                speed[0] =para.debug.para[3];
                speed[1] =  para.debug.para[4];
                speed[2] =para.debug.para[5];
                printf("Move motors at speed (%d %d %d)\n", para.debug.para[3], para.debug.para[4], para.debug.para[5]);
            }
            break;
        case 21://test RGB
            if(timestamp==2)
                SetRGBLED(para.debug.para[4], para.debug.para[0] * BLUE, para.debug.para[1]*BLUE, para.debug.para[2]*BLUE, para.debug.para[3]*BLUE);
            break;
        case 22:
            if(timestamp == 2)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("ABSF");
                docked[2] = sym.data;
                neighbours_IP[2]=StringToIP("192.168.2.224");
            }
            break;
        case 23:
            if(timestamp == 2)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("AFSB");
                docked[0] = sym.data;
                neighbours_IP[0]=StringToIP("192.168.2.224");
            }
            break;
        case 24:
            if(timestamp >=10)
            {
                if(timestamp <=60)
                    irobot->MoveHinge(para.debug.para[0]);
                else
                    irobot->MoveHinge(0);
            }
            break;
        case 25:
            if(timestamp == 2)
            {
                printf("1\n");
                mytree.reBuild("SFAFABSFSBABAFSFSBAFABSF000000000000000000000000");
                std::cout<<mytree<<std::endl;
                printf("2\n");
                
                std::vector<uint8_t>IPs;
                std::vector<uint8_t>rootIPs;
                rootIPs.push_back(10);
                rootIPs.push_back(11);
                IPs.push_back(11);
                IPs.push_back(12);
                IPs.push_back(12);
                IPs.push_back(13);
                IPs.push_back(13);
                IPs.push_back(14);
                IPs.push_back(14);
                IPs.push_back(15);
                IPs.push_back(15);
                IPs.push_back(16);
                mytree.setBranchIPs(FRONT, IPs);
                mytree.setBranchRootIPs(FRONT, rootIPs);

                std::cout<<mytree<<std::endl;

                InitRobotPoseInOrganism();

                std::map<uint32_t, robot_pose>::iterator it;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    printf("%s's index: %d pose: %d type: %c og_irsensor_index: %d tail_header: %d\n", IPToString(it->first), it->second.index,
                                it->second.direction, robottype_names[it->second.type], it->second.og_irsensor_index, it->second.tail_header);
                }

            }
            else if(timestamp == 3)
            {
                robot_pose_in_organism.clear();
                printf("1\n");
                mytree.reBuild("ABSBSFABAFSB000000000000AFSFSBAFABSF000000000000");
                std::cout<<mytree<<std::endl;
                printf("2\n");
                
                std::vector<uint8_t>IPs;
                std::vector<uint8_t>rootIPs;
                rootIPs.push_back(13);
                rootIPs.push_back(12);
                IPs.push_back(12);
                IPs.push_back(11);
                IPs.push_back(11);
                IPs.push_back(10);
                mytree.setBranchIPs(BACK, IPs);
                mytree.setBranchRootIPs(BACK, rootIPs);
                IPs.clear();
                rootIPs.clear();
                rootIPs.push_back(13);
                rootIPs.push_back(14);
                IPs.push_back(14);
                IPs.push_back(15);
                IPs.push_back(15);
                IPs.push_back(16);
                mytree.setBranchIPs(FRONT, IPs);
                mytree.setBranchRootIPs(FRONT, rootIPs);

                std::cout<<mytree<<std::endl;

                InitRobotPoseInOrganism();

                std::map<uint32_t, robot_pose>::iterator it;
                for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
                {
                    printf("%s's index: %d pose: %d type: %c og_irsensor_index: %d tail_header: %d\n", IPToString(it->first), it->second.index,
                                it->second.direction, robottype_names[it->second.type], it->second.og_irsensor_index, it->second.tail_header);
                }

            }
            break;

        case 26:
            {
                if(timestamp % 10 ==0)
                {
                    uint8_t data[4];
                    data[0] = 159; //id
                    data[1] = CMD_LOCKING_MOTOR; //cmd
                    data[2] = para.debug.para[0]; //side
                    data[3] = para.debug.para[9]; //cmd
                    BroadcastIRMessage(0, IR_MSG_TYPE_REMOTE_DEBUG, data, 4, 0);
                }
            }
            break;
        
        case 27:
            {
                acceleration_t acc = irobot->GetAcceleration();
                hingeData hd = irobot->GetHingeStatus();
                if(timestamp <= 100 )
                {
                    irobot->MoveHingeToAngle(para.debug.para[9],40);
                }
                else if(timestamp <= 150)
                {
                    irobot->MoveHingeToAngle(para.hinge_motor_default_pos/10, para.hinge_motor_default_pos%10, 40);
                }
                /*
                else if(timestamp == 160)
                {
                    irobot->MoveDockingLeft(-30);
                    irobot->MoveDockingRight(-30);
                }
                else if(timestamp == 170)
                {
                    irobot->MoveDockingLeft(0);
                    irobot->MoveDockingRight(0);
                }*/
                double angle = 360 * atan((acc.x * 1.0) / (-acc.z * 1.0)) /2 *M_PI;
                printf("Hinge: %d %d acc: %d %d %d \ncurrent angle %f\n", hd.currentAngle, hd.targetAngle, acc.x, acc.y, acc.z, angle);
            }
            break;

        case 30:
            {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(fileno(stdin), &fds); 
                char buf[256];

                if(select(fileno(stdin)+1, &fds, NULL, NULL, NULL)>0)
                {
                    if (FD_ISSET(fileno(stdin), &fds))
                    {
                        if(fgets(buf,256,stdin)!=NULL)
                        {
                            ParseCMD(buf); 
                            fprintf(stderr,"cmd>");
                        }
                    }
                }
            }
            break;
        case 31:
            {
                if(timestamp ==2)
                {
                    IP_collection_done = false;

                    printf("Set neighbour's IP\n");
                    neighbours_IP[0] = getFullIP(para.debug.para[0]);
                    neighbours_IP[1] = getFullIP(para.debug.para[1]);
                    neighbours_IP[2] = getFullIP(para.debug.para[2]);
                    neighbours_IP[3] = getFullIP(para.debug.para[3]);


                    //fill tree and branches
                    printf("Set mytree and all recruiting side ip\n");

                    if(!para.og_seq_list.empty())
                    {
                        mytree = target = para.og_seq_list[0];
                        std::cout<<mytree<<std::endl;
                    }

                    rt_status ret=OrganismSequence::fillBranches(mytree, mybranches);
                    if(ret.status >= RT_ERROR)
                    {
                        std::cout<<ClockString()<<" : "<<name<<" : ERROR in filling branches !!!!!!!!!!!!!!!!!!!!"<<std::endl;
                    }

                    //set all recruiting side
                    std::vector<OrganismSequence>::iterator it;
                    for(it = mybranches.begin() ; it != mybranches.end(); it++)
                    {
                        //check the first symbol that indicates the parent and child side of the connection
                        uint8_t branch_side = it->getSymbol(0).side1;
                        //enalbe docking signals
                        std::cout<<name<<" branch "<<*it<<std::endl;

                        docked[branch_side]= it->getSymbol(0).data;

                        std::vector<uint8_t> root_IPs;
                        root_IPs.push_back(uint8_t((my_IP.i32 >>24) & 0xFF));
                        root_IPs.push_back(uint8_t((neighbours_IP[branch_side].i32>>24) & 0xFF));
                        mytree.setBranchRootIPs(robot_side(branch_side),root_IPs);

                    }

                    seed = para.debug.para[7];

                    //set parent side;
                    if(!seed)
                    {
                        printf("Set parent side\n");
                        parent_side = para.debug.para[4];
                        docked[parent_side] = type| parent_side <<2| para.debug.para[5] << 4 | para.debug.para[6] << 6;
                        msg_organism_seq_received = true;

                        commander_IP = getFullIP(para.debug.para[8]);
                        commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;
                        
                        commander_IPC.Start(IPToString(commander_IP), commander_port, false);
                    }
                    else
                        master_IPC.Start("localhost", COMMANDER_PORT_BASE + COMMANDER_PORT, true);
                }
                else if(timestamp == 9)
                {
                    if(seed)
                    {
                        commander_IP = my_IP;
                        commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;
                        commander_IPC.Start(IPToString(commander_IP), commander_port, false);
                    }
                }
                else if(timestamp == 10)
                {
                    current_state = INORGANISM;
                    last_state = DEBUGGING;
                }

            }
            break;

        default:
            break;
    }

}

void RobotAW::Calibrating()
{
    memset(speed, 0, 3 * sizeof(int));

    static int32_t temp1[8]={0,0,0,0,0,0,0,0};
    static int32_t temp2[8]={0,0,0,0,0,0,0,0};
    static int32_t temp3[8]={0,0,0,0,0,0,0,0};
    static int32_t temp4[8]={0,0,0,0,0,0,0,0};
    static int32_t count = 0;
    count++;
    for(int i=0;i<NUM_IRS;i++)
    {
        temp1[i] += reflective[i];
        temp2[i] += ambient[i];
        temp3[i] += aux_reflective[i];
        temp4[i] += aux_ambient[i];
    }

    if (timestamp == 100)
    {
        printf("Calibrating done (%d): ", count);
        printf("reflective: ");
        for(int i=0;i<NUM_IRS;i++)
        {
            para.reflective_calibrated[i]=temp1[i]/count;
            para.ambient_calibrated[i]=temp2[i]/count;
            printf("%d\t", para.reflective_calibrated[i]);
        }
        printf("\n");
        printf("aux_reflective: ");
        for(int i=0;i<NUM_IRS;i++)
        {
            para.aux_reflective_calibrated[i]=temp3[i]/count;
            para.aux_ambient_calibrated[i]=temp4[i]/count;
            printf("%d\t", para.aux_reflective_calibrated[i]);
        }
        printf("\n");


        if(optionfile)
        {

            for( int entity = 1; entity < optionfile->GetEntityCount(); ++entity )
            {
                const char *typestr = (char*)optionfile->GetEntityType(entity);          
                if( strcmp( typestr, "Global" ) == 0 )
                {
                    char default_str[64];
                    for(int i=0;i<NUM_IRS;i++)
                    {
                        snprintf(default_str, sizeof(default_str), "%d", para.reflective_calibrated[i]);
                        optionfile->WriteTupleString(entity, "reflective_calibrated", i, default_str);
                        snprintf(default_str, sizeof(default_str), "%d", para.ambient_calibrated[i]);
                        optionfile->WriteTupleString(entity, "ambient_calibrated", i, default_str);
                        snprintf(default_str, sizeof(default_str), "%d", para.aux_reflective_calibrated[i]);
                        optionfile->WriteTupleString(entity, "aux_reflective_calibrated", i, default_str);
                        snprintf(default_str, sizeof(default_str), "%d", para.aux_ambient_calibrated[i]);
                        optionfile->WriteTupleString(entity, "aux_ambient_calibrated", i, default_str);
                    }
                }
            }
        }

        optionfile->Save("/flash/morph/aw_option.cfg");


    }

}

void RobotAW::PrintAuxReflective()
{
    if(!para.print_reflective)
        return;

    printf("%d: aux_reflec\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", aux_reflective_hist[i].Avg());
    printf("\n");
}

void RobotAW::PrintAuxAmbient()
{
    if(!para.print_ambient)
        return;

    printf("%d: aux_ambient\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", aux_ambient_hist[i].Avg());
    printf("\n");
}



void RobotAW::Log()
{
    int id0=para.debug.para[7];
    int id1=para.debug.para[8];
    if (logFile.is_open())
    {
        logFile << timestamp << "\t" << state_names[current_state] <<"\t";
        logFile << reflective_hist[id0].Avg()<<"\t";
        logFile << reflective_hist[id1].Avg()<<"\t";
        logFile << beacon[id0]<<"\t";
        logFile << beacon[id1]<<"\t";
        logFile << ambient_hist[id0].Avg()<<"\t";
        logFile << ambient_hist[id1].Avg()<<"\t";
        logFile << proximity[id0]<<"\t";
        logFile << proximity[id1]<<"\t";
        logFile << std::endl;
    }

}

