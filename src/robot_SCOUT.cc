#include "robot_SCOUT.hh"
#include "og/organism_sample.hh"

RobotSCOUT::RobotSCOUT(ScoutBot * robot):Robot()
{

    SPIVerbose = QUIET;

    if(name)
        free(name);
    name = strdup("RobotSCOUT");
    type = ROBOT_SCOUT;

    irobot = robot;

    //point to right SPI device number
    board_dev_num[::FRONT] = ScoutBot::FRONT;
    board_dev_num[::RIGHT] = ScoutBot::RIGHT;
    board_dev_num[::BACK] = ScoutBot::REAR;
    board_dev_num[::LEFT] = ScoutBot::LEFT;

    robot_side_dev_num[ScoutBot::FRONT] = ::FRONT;
    robot_side_dev_num[ScoutBot::RIGHT] = ::RIGHT;
    robot_side_dev_num[ScoutBot::REAR] = ::BACK;
    robot_side_dev_num[ScoutBot::LEFT] = ::LEFT;

    LED0 = 0x1;
    LED1 = 0x2;
    LED2 = 0x4;

    printf("Consctruction RobotSCOUT\n");
}

RobotSCOUT::~RobotSCOUT()
{
    printf("Desctruction RobotSCOUT\n");
}

void RobotSCOUT::InitHardware()
{
    for(int i=0;i<NUM_DOCKS;i++)
    {
        irobot->SetPrintEnabled(i, false); 
        irobot->enableDockingSense(i, true);
    }

    irobot->EnableMotors(true);

 //   Ethernet::disableSwitch();
}

void RobotSCOUT::Reset()
{
    RobotBase::MSPReset();
}

void RobotSCOUT::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    int board = board_dev_num[channel];
    irobot->SetIRLED(ScoutBot::Side(board), led);
    irobot->SetIRPulse(ScoutBot::Side(board), pulse_led|IRPULSE2);
    irobot->SetIRMode(ScoutBot::Side(board), mode);

    if(mode !=IRLEDOFF)
        irobot->SetIRRX(ScoutBot::Side(board), led & LED1 ? false : true);
    else
        irobot->SetIRRX(ScoutBot::Side(board), led & LED1 ? true : false);


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

void RobotSCOUT::SetRGBLED(int channel, uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br)
{
    int board = board_dev_num[channel];
    irobot->SetLED(ScoutBot::Side(board), tr, bl, br, tl);
    RGBLED_status[channel] = tl|tr|bl|br;

}

void RobotSCOUT::SetSpeed(int8_t leftspeed, int8_t rightspeed, int8_t speed3)
{
    if(leftspeed > 100)
        leftspeed = 100;
    else if(leftspeed < -100)
        leftspeed = -100;
    if(rightspeed > 100)
        rightspeed = 100;
    else if(rightspeed < -100)
        rightspeed = -100;
    irobot->Move(para.scout_wheels_direction[0]*leftspeed, para.scout_wheels_direction[1]*rightspeed);
}



bool RobotSCOUT::SetDockingMotor(int channel, int status)
{
    CPrintf4(SCR_BLUE,"%d -- side %d set motor %#x, (status: %#x)", timestamp, channel, status, docking_motors_status[channel]);

    if(status != STOP)
    {
        //closed -> open?
        if(docking_motors_status[channel] == CLOSED  && status == OPEN)
        {
            //change status to be opening
            docking_motors_status[channel] = OPENING; //clear first
            if(para.locking_motor_enabled[channel])
                irobot->OpenDocking(ScoutBot::Side(board_dev_num[channel]));
            printf("open docking\n");
        }
        //or open -> close
        else if(docking_motors_status[channel]== OPENED &&  status == CLOSE )
        {
            //change status to be closing
            docking_motors_status[channel] = CLOSING; //clear first
            if(para.locking_motor_enabled[channel])
                irobot->CloseDocking(ScoutBot::Side(board_dev_num[channel]));
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
        irobot->MoveDocking(ScoutBot::Side(board_dev_num[channel]), 0);
    }

    //TODO: checking this
    //MoveDocking(ScoutBot::Side(board_dev_num[channel]), status);
    SetRGBLED(1, GREEN, GREEN, RED, RED);
    printf("\n------ moving docking motor ----\n");
    return true;
}

bool RobotSCOUT::SetHingeMotor(int status)
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



void RobotSCOUT::UpdateSensors()
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
    IRValues ret_A = irobot->GetIRValues(ScoutBot::FRONT);
    IRValues ret_B = irobot->GetIRValues(ScoutBot::LEFT);
    IRValues ret_C = irobot->GetIRValues(ScoutBot::REAR);
    IRValues ret_D = irobot->GetIRValues(ScoutBot::RIGHT);
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
    uint8_t isense=0;
//    printf("Isense: ");
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(irobot->isEthernetPortConnected(ScoutBot::Side(board_dev_num[i])))
            ethernet_status |= 1<<i;
        
        if(irobot->GetDScrewISense(ScoutBot::Side(board_dev_num[i])) > 220)
            isense |= 1<<i;
        
//        printf("%d\t", irobot->GetDScrewISense(ScoutBot::Side(board_dev_num[i])));
    }
//    printf("\n");

    ethernet_status_hist.Push2(ethernet_status);
    docking_motor_isense_hist.Push2(isense);



    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-para.reflective_calibrated[i]);
        ambient_hist[i].Push(para.ambient_calibrated[i] - ambient[i]);
    }
}

void RobotSCOUT::UpdateActuators()
{
    CheckDockingMotor();
    CheckHingeMotor();
    SetSpeed(speed[0], speed[1],speed[2]); 
}

// for self-repair
void RobotSCOUT::UpdateFailures()
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
				failure_delay = 0;
        	}
        }
    }
}

void RobotSCOUT::Avoidance()
{
    speed[0] = 40;
    speed[1] = 40;


    for(int i=0;i<NUM_IRS;i++)
    {
           speed[0] +=(para.avoid_weightleft[i] * (reflective_hist[i]).Avg())>>3;
           speed[1] += (para.avoid_weightleft[i] * (reflective_hist[i]).Avg())>>3;
    }

    if(reflective_hist[1].Avg() > para.avoidance_threshold || reflective_hist[0].Avg()>para.avoidance_threshold)
        direction = BACKWARD;
    else if(reflective_hist[4].Avg() > para.avoidance_threshold || reflective_hist[5].Avg()>para.avoidance_threshold)
        direction = FORWARD;

    speed[0] = 0;
    speed[1] = 0;


}

void RobotSCOUT::Exploring()
{
    Avoidance();
}


void RobotSCOUT::Resting()
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
void RobotSCOUT::Seeding()
{
    mytree.Clear();
    //select predefined organism
    //og = new Organism;
    //RealDemoOrganism_KAK(og);
    //og->GraphToSequence(mytree);
    //std::cout<<*og<<std::endl;
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
        SetIRLED(branch_side, IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1); 
        std::cout<<name<<" branch "<<*it<<std::endl;
    }
}
void RobotSCOUT::Foraging()
{
    speed[0]=0;
    speed[1]=0;
    /*
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

        speed[0] = 0;
        speed[1] = 0;
    }
    else*/
    {
        Avoidance();

        if(powersource_found)
        {
            current_state = LOCATEENERGY;
            last_state = FORAGING;
        }
        else if(organism_found)
        {
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

            assembly_count = DEFAULT_ASSEMBLY_COUNT;
            current_state = ASSEMBLY;
            last_state = WAITING;
        }

    }


}
void RobotSCOUT::Waiting()
{
    speed[0] = 0;
    speed[1] = 0;

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
            SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);

        current_state = FORAGING;
        last_state = WAITING;
    }
    else if(organism_found)
    {
        for(int i=0;i<NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

        current_state = ASSEMBLY;
        last_state = WAITING;

        assembly_count = DEFAULT_ASSEMBLY_COUNT;
    }
}

void RobotSCOUT::Assembly()
{
    speed[0]=0;
    speed[1]=0;

    if(assembly_count--<=0)
    {
        organism_found = false;

        current_state = FORAGING;
        last_state = ASSEMBLY;
    }
    //right type of recruitment message recived, then locatebeacon
    else if (assembly_info.type2 == type)
    {
        std::cout<<"assembly_info: "<<assembly_info<<"\tdirection: "<<direction<<std::endl;

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
    else
        Avoidance();
}

void RobotSCOUT::LocateEnergy()
{
    speed[0] = 0;
    speed[1] = 0;

    if(1)
    {
        current_state = SEEDING;
        last_state = LOCATEENERGY;
        return;
    }
}


void RobotSCOUT::LocateBeacon()
{
    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    speed[0] = 0;
    speed[1] = 0;

    locatebeacon_count++;

    int turning = 0;
    if(beacon_signals_detected)
    {
        speed[0] = direction *para.locatebeacon_forward_speed[0];
        speed[1] = direction *para.locatebeacon_forward_speed[1];

        if(id0==0)
        {
            if((beacon_signals_detected & 0x3) != 0)
                turning = 0;
            //no signals on side 1, turn left 
            else if((beacon_signals_detected & 0xC) ==0)
                turning = -1;
            //no signals on side 3, turn right 
            else if((beacon_signals_detected & 0xC0) ==0)
                turning = 1;
            else 
                turning = 0;
        }
        else
        {

            if((beacon_signals_detected & 0x30) != 0)
                turning = 0;
            //no signals on side 1, turn right
            else if((beacon_signals_detected & 0xC) ==0)
                turning = 1;
            //no signals on side 3, turn left
            else if((beacon_signals_detected & 0xC0) ==0)
                turning = -1;
            else 
                turning = 0;
            printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tturning: %d\n", beacon[0], beacon[1], beacon[2], beacon[3],
                    beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, turning);
        }

        //printf("max_beacon: %d %d %d %d\tturning: %d\n", max_beacon[0], max_beacon[1], max_beacon[2], max_beacon[3], turning);
        if(turning != 0)
        {
            speed[0] = turning * 30;
            speed[1] = -turning * 30;
        }
        else
        {
            if((beacon_signals_detected & (1<<id0 | 1<<id1)) != 0)
            {
                for(int i=0;i<NUM_IRS;i++)
                {
                    speed[0] += (beacon[i] * direction * para.locatebeacon_weightleft[i]) >>1;
                    speed[1] += (beacon[i] * direction * para.locatebeacon_weightright[i]) >>1;
                }
            }
            else
            {
                speed[0] = 0;
                speed[1] = 0;
            }
        }
    }

    printf("beacon: (%d %d) -- speed: (%d %d %d %d)\n", beacon[id0], beacon[id1], speed[0], speed[1], para.locatebeacon_forward_speed[0], para.locatebeacon_forward_speed[1]);
    //switch on ir led at 64Hz so the recruitment robot can sensing it
    //and turn on its docking signals, the robot need to switch off ir 
    //led for a while to check if it receives docking signals
    static int flag=0;
    if(timestamp % 5 ==0)
    {
        flag++;
        switch (flag % 6)
        {
            case 3:
            case 4:
                {
                    //switch off all ir led, 
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                        SetRGBLED(i, 0,0,0,0);
                    }
                }
                break;
            case 0:
            case 1:
            case 2:
            case 5:
                {
                    if((beacon_signals_detected == (1<<id0| 1<<id1)) && beacon[id0] >= 30 && beacon[id1] >= 30)  
                    {
                        current_state = ALIGNMENT;
                        last_state = LOCATEBEACON;

                        //using reflective signals if not set
                        for(int i=0; i< NUM_DOCKS;i++)
                            SetIRLED(i, IRLEDOFF, LED1, IRPULSE0 | IRPULSE1);

                    } 
                    else if (beacon_signals_detected ==0 )
                    {
                        if(locatebeacon_count >=150)
                        {
                            current_state = ASSEMBLY;
                            last_state = LOCATEBEACON;

                            organism_found = false;
                            assembly_count = DEFAULT_ASSEMBLY_COUNT;
                            assembly_info = OrganismSequence::Symbol(0);
                            
                            for(int i=0;i<NUM_DOCKS;i++)
                            {
                                SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);
                                SetRGBLED(i, 0, 0, 0, 0);
                            }
                        }
                        else
                        {
                            //then swith on all ir led at 64Hz frequency
                            for(int i=0;i<NUM_DOCKS;i++)
                                SetIRLED(i, IRLEDPROXIMITY, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                        }
                    }
                }
                break;
            default:
                break;
        }
    }


}

//TODO: cleanup the code
void RobotSCOUT::Alignment()
{
    speed[0] = direction * para.aligning_forward_speed[0];
    speed[1] = direction * para.aligning_forward_speed[1];

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    int reflective_diff = abs(reflective_hist[id0].Avg() - reflective_hist[id1].Avg());
    int reflective_max = std::max(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
    int reflective_min = std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());

    if(docking_region_detected)
    {
        speed[0] = 0;
        speed[1] = 0;

        //request for assembly_info
        if(msg_assembly_info_expected && timestamp % 10 ==0)
            Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_ASSEMBLY_INFO_REQ, 0);

        if(msg_assembly_info_received)
        {
            msg_assembly_info_received = 0;
            msg_assembly_info_expected = 0;

            if(assembly_info == OrganismSequence::Symbol(0))
            {
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


                current_state = DOCKING;
                last_state = ALIGNMENT;

                SetRGBLED(assembly_info.side2, 0,0,0,0);
            }
        }

     } 
    else
    {
        //check if it is aligned well and also closed enough for docking
        //this is done by checking if ehternet port is connected
        if(ethernet_status_hist.Sum(assembly_info.side2) > 2) 
        {
            printf("Docking region detected\n");
            docking_region_detected = true;
            in_docking_region_hist.Reset();
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            SetRGBLED(assembly_info.side2, WHITE,WHITE,WHITE,WHITE);

            msg_assembly_info_expected |= 1 << assembly_info.side2;
        }
        else
        {
            //  printf("reflective (%d %d) beacon (%d %d)\n", reflective_diff, reflective_max, beacon[id0], beacon[id1]);
            //TODO sometimes not in good position, need to reverse and try again
            // define the bad case
            // case 1: difference between two front reflective_calibrated reading is significant 
            // case 2: some reflective_calibrated readings but two beacon readings are diff
            if(reflective_hist[id0].Avg() > 300 && reflective_hist[id1].Avg() > 300)
                blocking_count++;

            //3 second is allowed until fully docked, otherwise, treated as blocked.
            if((reflective_diff > 300 && reflective_diff > reflective_max * 0.5 && reflective_min > 0) || blocking_count > 30)
                //if((reflective_diff > 1000 && reflective_diff > reflective_max * 0.7) ||blocking_count > 30)
                docking_blocked = true;

            if(docking_blocked || beacon_signals_detected == 0)
            {
                docking_blocked = false;
                blocking_count=0;
                speed[0] = 0;
                speed[1] = 0;

                docking_trials++;

                current_state = RECOVER;
                last_state = ALIGNMENT;
                recover_count = 0;
                printf("reflective (%d %d) beacon (%d %d)\n", reflective_diff, reflective_max, beacon[id0], beacon[id1]);
                printf("blocking_count %d reflective: %d %d\t beacon:%d %d\n",blocking_count, reflective_hist[id0].Avg(), reflective_hist[id1].Avg(), beacon[id0], beacon[id1]);
            }
            else
            {
                for(int i=id0;i<=id1;i++)
                {
                    speed[0] += (beacon[i] * para.aligning_weightleft[i]) >>4;
                    speed[1] += (beacon[i] * para.aligning_weightright[i]) >>4;
                }
            }
        }
    }
}

void RobotSCOUT::Recover()
{
    recover_count++;

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
    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    if(last_state == ALIGNMENT)
    {
        //turn left/right according to reflective value;
        //robot will stop there for 1 seconds
        if(recover_count < para.aligning_reverse_count)
        {
            //docked to the wrong robots
            if(assembly_info == OrganismSequence::Symbol(0))
            {
                //just reverse
                if(reflective_hist[0].Avg() + reflective_hist[1].Avg() >
                reflective_hist[3].Avg() + reflective_hist[4].Avg() )
                {
                    speed[0] = para.aligning_reverse_speed[0];
                    speed[1] = para.aligning_reverse_speed[1];
                }
                else
                {
                    speed[0] = -para.aligning_reverse_speed[0];
                    speed[1] = -para.aligning_reverse_speed[1];
                }


            }
            //blocked
            else
            {
                speed[0] = direction * para.aligning_reverse_speed[0];
                speed[1] = direction * para.aligning_reverse_speed[1];

                if(timestamp % 5 ==0)
                    Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_DOCKING_SIGNALS_REQ, 0);

                //TODO: test reverse behaviour with new robots
                const int weight_left[8] = {-3,3, 0,0,-3,3,0,0};
                const int weight_right[8] = {3,-3,0,0,3,-3,0,0};
                for(int i=id0;i<=id1;i++)
                {
                    speed[0] += reflective_hist[i].Avg() * weight_left[i]>>8;
                    speed[1] += reflective_hist[i].Avg() * weight_right[i]>>8;
                }
            }
        }
        else if( recover_count == para.aligning_reverse_count )
        {
            for(int i=0; i<SIDE_COUNT; i++)
            	SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,0);
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
                
                if(docking_trials >= para.docking_trials)
                {
                    ResetAssembly();

                    for(int i=0; i<SIDE_COUNT; i++)
                    	SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);

                    current_state = FORAGING;
                    last_state = RECOVER;
                }
                else if(beacon_signals_detected_hist.Sum(id0) >= 5 && beacon_signals_detected_hist.Sum(id1) >= 5)
                {
                    for(int i=0; i<SIDE_COUNT; i++)
                    	SetIRLED(i,IRLEDOFF,LED0|LED1|LED2,IRPULSE0|IRPULSE1);

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

void RobotSCOUT::Docking()
{
    speed[0] = direction * para.docking_forward_speed[0];
    speed[1] = direction * para.docking_forward_speed[1];  

    docking_count++;

    if(ethernet_status_hist.Sum(assembly_info.side2) > 8) 
    {
        speed[0] = 0;
        speed[1]= 0;  
        SetRGBLED(assembly_info.side2, WHITE, WHITE, WHITE, WHITE);
        SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED2, 0);
        irobot->SetIRRX(ScoutBot::Side(board_dev_num[assembly_info.side2]), false);

        SetDockingMotor(assembly_info.side2, CLOSE);

        current_state = LOCKING;
        last_state = DOCKING;
    }

}

void RobotSCOUT::Locking()
{
    speed[0] = 0;
    speed[1] = 0;

    int docking_side = assembly_info.side2;

    SetRGBLED(docking_side, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason

    //docking motor is done?
    if(docking_motors_status[docking_side] == CLOSED)
    {
        if(docked[docking_side]==0)
        {
            docked[docking_side] = assembly_info.type2  | assembly_info.side2 << 2 | assembly_info.type1 << 4 | assembly_info.side1 << 6;
            unlocking_required[docking_side] = true;
            Robot::BroadcastIRMessage(docking_side, IR_MSG_TYPE_LOCKED, para.ir_msg_repeated_num);
        }
        else if(docked[docking_side] && !MessageWaitingAck(docking_side, IR_MSG_TYPE_LOCKED))
        {
            msg_organism_seq_expected = true;
            msg_subog_seq_expected |= 1<<docking_side;
            current_state = INORGANISM;
            last_state = LOCKING;
        }
    }

}

void RobotSCOUT::Recruitment()
{
    speed[0] = 0;
    speed[1] = 0;
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
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else
            {
                //or send recruitment message
                SetIRLED(i, IRLEDOFF, LED1, 0);
                SetRGBLED(i, 0,0,0,0);
                if(timestamp % RECRUITMENT_SIGNAL_INTERVAL == i)
                {
                    Robot::BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING, it1->getSymbol(0).data, 0);
                }
            }
        }
        else if(recruitment_stage[i]==STAGE1)
        {
            if( recruitment_count[i]++ > para.recruiting_beacon_signals_time )
            {
                recruitment_count[i]=0;
                recruitment_stage[i]=STAGE0;
                SetIRLED(i,IRLEDOFF,LED1,0);
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);

            }
            else if(msg_docking_signal_req_received & (1<<i))
            {
                recruitment_count[i]=0;
                msg_docking_signal_req_received &= ~(1<<i);
                SetIRLED(i, IRLEDDOCKING, LED1, 0); 
            }
            else if(msg_assembly_info_req_received & (1<<i))
            {
                recruitment_count[i]=0;
                if(it1->getSymbol(0).type2 != ROBOT_AW)
                    msg_locked_expected |= 1<<i;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                //wait for ack
                if(!MessageWaitingAck(i, IR_MSG_TYPE_ASSEMBLY_INFO))
                {
                    msg_assembly_info_req_received &= ~(1<<i);
                    guiding_signals_count[i]=0;
                    recruitment_stage[i]=STAGE2;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
                }
            }
            else if(msg_guideme_received & (1<<i))
            {
                if(it1->getSymbol(0).type2 == ROBOT_SCOUT || it1->getSymbol(0).type2 == ROBOT_KIT)
                    msg_locked_expected |= 1<<i;
                else if(it1->getSymbol(0).type2 == ROBOT_AW)
                    msg_lockme_expected |= 1<<i;
                msg_guideme_received &= ~(1<<i);
                guiding_signals_count[i]=0;
                recruitment_stage[i]=STAGE2;
                recruitment_count[i]=0;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE2)
        {
            guiding_signals_count[i]++;

            msg_guideme_received &= ~(1<<i);
            msg_lockme_expected |=1<<i;

            //received lockme or locked message
            if(guiding_signals_count[i] > 20 && (msg_lockme_received & (1<<i) || msg_locked_received &(1<<i)))
            {
                recruitment_stage[i]=STAGE3;
                guiding_signals_count[i] = 0;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                SetRGBLED(i, 0,0,0,0);
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), false);
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }

            if(msg_assembly_info_req_received & (1<<i))
            {
                if(it1->getSymbol(0).type2 != ROBOT_AW)
                    msg_locked_expected |= 1<<i;
                else
                    msg_lockme_expected |= 1<<i;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0); //switch left and right leds
                //wait for ack
                if(!MessageWaitingAck(i, IR_MSG_TYPE_ASSEMBLY_INFO))
                {
                    msg_assembly_info_req_received &= ~(1<<i);
                }

                //reset the clock as it happens that another robot just docked before it gives up recruiting
                guiding_signals_count[i] = 0;
            }

            //received docking_signals req, back to stage 1
            if((msg_docking_signal_req_received & (1<<i)))
            {
                msg_docking_signal_req_received &=~(1<<i);
                guiding_signals_count[i] = 0;
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  recieved docking signals request, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            //nothing happens, assumed wrong robots docked, back to stage 0
            else if(guiding_signals_count[i] >= (uint32_t)para.recruiting_guiding_signals_time)
            {
                guiding_signals_count[i] = 0;
                recruitment_stage[i]=STAGE0;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  waits too long, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }

        }
        else if(recruitment_stage[i]==STAGE3)
        {
            if(msg_lockme_received & (1<<i))
            {
                msg_lockme_received &=~(1<<i);
                SetDockingMotor(i, CLOSE); 

                recruitment_stage[i]=STAGE4;
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else if(msg_locked_received & (1<<i))
            {
                msg_locked_received &=~(1<<i);
                msg_locked_expected &=~(1<<i);
                msg_subog_seq_expected |= 1<<i;
                docked[i]= it1->getSymbol(0).data;
                recruitment_stage[i]=STAGE4;
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else if(msg_docking_signal_req_received & 1<<i)
            {
                msg_docking_signal_req_received &=~(1<<i);
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  received docking signals req, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else if(msg_guideme_received & (1<<i))
            {
                msg_guideme_received &= ~(1<<i);
                recruitment_stage[i] = STAGE2;
                guiding_signals_count[i] = 0;
                SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
                printf("%d -- Recruitment: channel %d  switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
        }
        else if(recruitment_stage[i]==STAGE4)
        {
            if(docking_motors_status[i] == CLOSED && docked[i]==0)
            {
                unlocking_required[i] = true;
                msg_subog_seq_expected |= 1<<i;
                docked[i]= it1->getSymbol(0).data;
                Robot::BroadcastIRMessage(i, IR_MSG_TYPE_LOCKED, para.ir_msg_repeated_num);
            }
            else if(docked[i] && !docking_done[i])
            {
                if(!MessageWaitingAck(i, IR_MSG_TYPE_LOCKED))
                {
                    docking_done[i] = true;
                    SendBranchTree(i, (*it1)); // was mytree
                }
            }
            else if(docking_done[i])
            {
                //recevied acks after sending sequence information?
                if(!MessageWaitingAck(i, IR_MSG_TYPE_ORGANISM_SEQ))
                {
                    docked[i]= it1->getSymbol(0).data;
                    msg_subog_seq_expected |= 1<<i;
                    recruitment_stage[i] = STAGE5;
                    docking_done[i] = false;

                    //SetRGBLED(i, RED, 0, 0, 0);
                    //SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    //RobotBase::SetIRRX(board_dev_num[i], false);

                    num_robots_inorganism++;

                    msg_ip_addr_expected |= 1<<i;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
                }
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
                memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP, 4);
                Robot::SendIRMessage(i, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, 0);
            }
            //get new ip address?
            else if(msg_ip_addr_received & (1<<i))
            {
                //prepare the newrobot_joined messages
                if(!seed)
                    PropagateIRMessage(IR_MSG_TYPE_NEWROBOT_JOINED, NULL, 0, i);

                //    msg_ip_addr_received &= ~(1<<i);

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

        printf("my IP is %#x (%d.%d.%d.%d)\n", my_IP,
                my_IP & 0xFF,
                (my_IP >> 8) & 0xFF,
                (my_IP >> 16) & 0xFF,
                (my_IP >> 24) & 0xFF);
        for(int i=0;i<NUM_DOCKS;i++)
        {
            printf("neighbour %d's IP is %#x (%d.%d.%d.%d)\n", i, neighbours_IP[i],
                    neighbours_IP[i] & 0xFF,
                    (neighbours_IP[i] >> 8) & 0xFF,
                    (neighbours_IP[i] >> 16) & 0xFF,
                    (neighbours_IP[i] >> 24) & 0xFF);
            SetRGBLED(i, 0, 0, 0, 0);
        }
    }
}


void RobotSCOUT::InOrganism()
{
    speed[0] = 0;
    speed[1] = 0;

    //seed robot monitoring total number of robots in the organism
    if(seed)
    {
        if( mytree.Edges() + 1 == (unsigned int)num_robots_inorganism)
        {
            organism_formed = true;

            textcolor(BRIGHT, SCR_RED, SCR_BLACK);  
            printf("%d -- organism formed !!\n", timestamp);
            textcolor(RESET, SCR_WHITE, SCR_BLACK); 
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            //prepare organism_formed_messages
            PropagateIRMessage(IR_MSG_TYPE_ORGANISM_FORMED);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %#x (%d.%d.%d.%d)\n", my_IP,
                    my_IP & 0xFF,
                    (my_IP >> 8) & 0xFF,
                    (my_IP >> 16) & 0xFF,
                    (my_IP >> 24) & 0xFF);
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %#x (%d.%d.%d.%d)\n", i, neighbours_IP[i],
                        neighbours_IP[i] & 0xFF,
                        (neighbours_IP[i] >> 8) & 0xFF,
                        (neighbours_IP[i] >> 16) & 0xFF,
                        (neighbours_IP[i] >> 24) & 0xFF);
            }

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
            for(int i=0;i<NUM_DOCKS;i++)
                SetRGBLED(i, WHITE, WHITE, WHITE, WHITE);

            macrolocomotion_count = 0;
            raising_count = 0;
            current_state = RAISING;
            last_state = INORGANISM;

            printf("my IP is %#x (%d.%d.%d.%d)\n", my_IP,
                    my_IP & 0xFF,
                    (my_IP >> 8) & 0xFF,
                    (my_IP >> 16) & 0xFF,
                    (my_IP >> 24) & 0xFF);
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("neighbour %d's IP is %#x (%d.%d.%d.%d)\n", i, neighbours_IP[i],
                        neighbours_IP[i] & 0xFF,
                        (neighbours_IP[i] >> 8) & 0xFF,
                        (neighbours_IP[i] >> 16) & 0xFF,
                        (neighbours_IP[i] >> 24) & 0xFF);
            }

        }
    }

}

void RobotSCOUT::Disassembly()
{
    speed[0] = 0;
    speed[1] = 0;

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
                //TODO: how about two Scout robots docked to each other
                else if(docking_motors_status[i]==OPENED)
                {
                    Robot::SendIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
                    docked[i]=0;
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

            // for demo only - turn on back proximity sensors
            SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0);

        }

    }


}

void RobotSCOUT::Undocking()
{
    speed[0] = 0;
    speed[1] = 0;

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
    else if( undocking_count < 100 )
    {

    	bool f = isNeighbourConnected(FRONT);
    	bool b = isNeighbourConnected(BACK);
    	bool l = isNeighbourConnected(LEFT);
    	bool r = isNeighbourConnected(RIGHT);

		if( !f && !r && !l && b )
		{
			speed[0] = 30;
			speed[1] = 30;
		}
		else if( !b && !l && !r && f )
		{
			speed[0] = -30;
			speed[1] = -30;
		}
		else
		{
			speed[0] = 0;
			speed[1] = 0;
		}
    }
	else
	{
		speed[0] = 0;
		speed[1] = 0;

		// Turn off LEDs
		for(int i=0;i<NUM_DOCKS;i++)
			SetRGBLED(i, 0,0,0,0);

		if( last_state ==  FAILED )
			current_state = RESTING;
		else
			current_state = FORAGING;

		last_state = UNDOCKING;

		RemoveFromQueue(IR_MSG_TYPE_UNLOCKED);
		ResetAssembly(); // reset variables used during assembly
	}

    undocking_count++;

}


void RobotSCOUT::Lowering()
{
    lowering_count++;

	//flashing RGB leds
	static int index = 0;
	index = (timestamp / 2) % 6;
	for(int i=0;i<NUM_DOCKS;i++)
	{
		switch (index)
		{
			case 0:
				SetRGBLED(i, 0, 0, YELLOW, YELLOW);
				break;
			case 1:
				SetRGBLED(i, 0, 0, 0, 0);
				break;
			case 2:
				SetRGBLED(i, YELLOW, YELLOW, 0, 0);
				break;
            case 3: //
            case 4: // short delay to better symbolise lowering
            case 5: //
				SetRGBLED(i, 0, 0, 0, 0);
				break;
			default:
				break;
		}
	}


    return; // for testing - do not allow to enter disassembly

    //else if(seed && lowering_count >= 150)
    if(seed && lowering_count >= 150)
    {
        PropagateIRMessage(IR_MSG_TYPE_DISASSEMBLY);

        current_state = DISASSEMBLY;
        last_state = LOWERING;
        lowering_count = 0;

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
        last_state = LOWERING;
        lowering_count = 0;

        msg_disassembly_received = 0;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if(docked[i])
                msg_unlocked_expected |=1<<i;
        }
    }

}

void RobotSCOUT::Raising()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    // Leds symbolise the raising process
    bool flash_leds = false;

    // Wait longer with larger structures
    int raising_delay = (mytree.Size()/2+1)*30;

    if(seed)
    {
        raising_count++;

        if(raising_count == raising_delay )
        {
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
            PropagateIRMessage(IR_MSG_TYPE_RAISING_START);
            PropagateEthMessage(ETH_MSG_TYPE_RAISING_START);
            flash_leds = true;
        }
        else if( raising_count >= raising_delay + 30 )
        {
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
            PropagateIRMessage(IR_MSG_TYPE_RAISING_STOP);
            PropagateEthMessage(ETH_MSG_TYPE_RAISING_STOP);

            current_state = MACROLOCOMOTION;
            last_state = RAISING;
            raising_count = 0;
            flash_leds = false;
        }
        else if( raising_count >= raising_delay )
        {
            flash_leds = true;
        }
    }
    else if( msg_raising_stop_received )
    {
        msg_raising_start_received = false;
        msg_raising_stop_received = false;
        current_state = MACROLOCOMOTION;
        last_state = RAISING;
        raising_count = 0;
        flash_leds = false;
    }
    else if( msg_raising_start_received )
    {
        flash_leds = true;
    }

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
                    SetRGBLED(i, YELLOW, YELLOW, 0, 0);
                    break;
                case 1:
                    SetRGBLED(i, 0, 0, 0, 0);
                    break;
                case 2:
                    SetRGBLED(i, 0, 0, YELLOW, YELLOW);
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
void RobotSCOUT::Reshaping()
{

	static uint8_t waiting_for_undock = 0;
	static uint8_t unlock_sent = 0;

    // If this is the seed or branch received from other module
    if( seed || msg_organism_seq_received )
    {
    	// If still waiting for some robots to undock
    	if( waiting_for_undock )
		{
    		std::cout << "waiting for someone to undock" << std::endl;
    		for( int i=0; i<SIDE_COUNT; i++ )
    		{
    			if( (waiting_for_undock & 1<<i) && docking_motors_status[i]==OPENED )
    			{
    				std::cout << i << " docking motors open" << std::endl;
    				if( !(unlock_sent & 1<<i) )
					{
						BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
						unlock_sent |= 1<<i;
					}
					else if( (msg_unlocked_received & 1<<i) || !Ethernet::switchIsPortConnected(i+1) )
					{
	    				std::cout << i << " unlock sent" << std::endl;
						docked[i]=0;
						waiting_for_undock &= ~(1<<i);
					}
    			}
    		}

    		// If no longer waiting for any robots to undock
    		if( !waiting_for_undock )
    		{
				current_state = RECRUITMENT;

				// Reset to prevent entering STAGE1 immediately
				msg_docking_signal_req_received = 0;

				// turn off LEDs
				for(int i=0; i<NUM_DOCKS; i++)
					SetRGBLED(i, 0, 0, 0, 0);
    		}
		}
    	else
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
				OrganismSequence::Symbol next_symbol = OrganismSequence::Symbol(0);
				std::vector<OrganismSequence>::iterator it;
				for(it = mybranches.begin() ; it != mybranches.end(); it++)
				{
					if( it->getSymbol(0).side1 == i )
					{
						next_symbol = it->getSymbol(0);
						branch_side = it->getSymbol(0).side1;
						next_branch = (*it);
						break;
					}
				}

				if( docked[i] )
				{
					// Check that neighbour is of correct type and orientation
					if( branch_side == i && (OrganismSequence::Symbol(docked[i]) == next_symbol))
					{
						// if there is a neighbour and there should be
						// send branch
						SendBranchTree(i, next_branch);
						recruitment_stage[i]=STAGE4;
						docking_done[i] = true; 		// may not be necessary
						printf("%d Sending branch to side %d\n",timestamp, i);
					}
					else
					{
						// if there is a neighbour but there shouldn't be
						// send disassembly
						PropagateSingleIRMessage(IR_MSG_TYPE_DISASSEMBLY,i);
						printf("%d Instructing module on side %d to disassemble\n",timestamp, i);

						if( unlocking_required[i] )
						{
							SetDockingMotor(i, OPEN);
							unlocking_required[i]=false;
							waiting_for_undock |= 1<<i;
						}
						else
						{
							BroadcastIRMessage(i, IR_MSG_TYPE_UNLOCKED, para.ir_msg_repeated_num);
							waiting_for_undock |= 1<<i;
							unlock_sent |= 1<<i;
						}
					}
				}
				// if there isn't a neighbour but there should be
				else if( docked[i]==0 && branch_side == i )
				{
					// start recruiting
					SetIRLED(branch_side, IRLEDDOCKING, LED1, IRPULSE0|IRPULSE1);
					printf("%d Preparing to recruit upon side %d\n",timestamp,i);
				}
				else if( waiting_for_undock == 0 )
				{
					current_state = RECRUITMENT;

					// Reset to prevent entering STAGE1 immediately
					msg_docking_signal_req_received = 0;

					// turn off LEDs
					for(int i=0; i<NUM_DOCKS; i++)
						SetRGBLED(i, 0, 0, 0, 0);
				}

			}

			// If there is no new tree - disassemble
			if( seed && mytree.Size() <= 0 )
			{
				current_state = DISASSEMBLY;
				last_state = RESHAPING;
				printf("%d No new tree to assemble, entering disassembly\n",timestamp);
			}
    	}

    }
    else if( msg_disassembly_received )
    {
        current_state = DISASSEMBLY;
        last_state = RESHAPING;

        msg_disassembly_received = 0;

        for(int i=0;i<NUM_DOCKS;i++)
        {
            SetRGBLED(i, 0, 0, 0, 0);
            if( docked[i] )
                msg_unlocked_expected |=1<<i;
        }
    }

}

void RobotSCOUT::MacroLocomotion()
{
    speed[0] = 0;
    speed[1] = 0;

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

    if( msg_lowering_received )
    {
        // Stop moving
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;

        last_state = MACROLOCOMOTION;
        current_state = LOWERING;
        lowering_count = 0;
        seed = false;
    }

}

void RobotSCOUT::Debugging()
{
    // speed[0] = 0;
    // speed[1] = 0;

    //printf("%d Debuging %d:\t", timestamp,para.debug.mode);
    static int clock=0;
    static bool log=false;
    Log();

    switch (para.debug.mode)
    {
        case 0: //simulating recruitment, stage 2, 64Hz helper signals
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[9], IRLEDPROXIMITY, LED0|LED2, 0);
            }

            printf("%d %d %d %d\n",  proximity[4], proximity[5], para.ambient_calibrated[4]-ambient[4], para.ambient_calibrated[4]-ambient[5]);

            break;
        case 1: // locking region threshold detection
            if(timestamp ==2)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetRGBLED(i,0,0,0,0);
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);//IRPULSE0|IRPULSE1);
                }
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n",  proximity[0], proximity[1], beacon[0], beacon[1], reflective_hist[0].Avg(), reflective_hist[1].Avg());
            break;
        case 2: // simulate recruitment, stage 1, 32Hz guiding signals
            if(timestamp == 2)
            {
              //  SetIRLED(para.debug.para[9], IRLEDDOCKING, LED1, 0);
                SetIRLED(para.debug.para[9], IRLEDDOCKING, LED1, 0);
            }
            printf("%d %d %d %d\n", reflective[4]-para.reflective_calibrated[4], reflective[5] - para.reflective_calibrated[5], para.ambient_calibrated[4]-ambient[4], para.ambient_calibrated[4]-ambient[5]);
            break;
        case 3: // docking region threshold detection
            if(timestamp ==2)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n", reflective[0]-para.reflective_calibrated[0], reflective[1] - para.reflective_calibrated[1], beacon[0], beacon[1], proximity[0], proximity[1]);
            break;
        case 4:// simulate locking stage, turn on RGB led to be bright 
            if(timestamp ==2)
            {
                SetRGBLED(0, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason
                SetIRLED(0, IRLEDOFF, LED0|LED2, 0);
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[0]), false);
            }
            break;
        case 5:// recruiting stage 2 -> stage 3 detection
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDPROXIMITY, LED0|LED2, 0); //switch docking signals 2 on left and right leds
            }
            break;
        case 6: //testing request ip via ircomm
            if(timestamp == 4)
            {
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    SetIRLED(i, IRLEDOFF, LED0|LED2, 0);
                    irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), false);
                }
        
                printf("my IP is %#x (%d.%d.%d.%d)\n", my_IP,
                my_IP & 0xFF,
                (my_IP >> 8) & 0xFF,
                (my_IP >> 16) & 0xFF,
                (my_IP >> 24) & 0xFF);

                OrganismSequence::Symbol sym;
                sym.type1 = ROBOT_SCOUT;
                sym.side1 = ::FRONT;
                sym.type2 = ROBOT_SCOUT;
                sym.side2 = ::FRONT;
                uint8_t data[5];
                data[0] = sym.data;
                docked[0]=sym.data;
                memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP, 4);
                Robot::SendIRMessage(::FRONT, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, para.ir_msg_repeated_num);
            }

            break;
        case 7: // measuring beacon signals
            if(timestamp ==40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                speed[0] = para.debug.para[3];
                speed[1] = para.debug.para[4];
                printf("set speed %d %d\n", speed[0], speed[1]);
            }
            else if(timestamp == (uint32_t)para.debug.para[2])
            {
                speed[0] = 0;
                speed[1] = 0;
                current_state = RESTING;
                last_state = DEBUGGING;
            }
            printf("%d\t%d\t%d\t%d\t%d\t%d\n", reflective[0]-para.reflective_calibrated[0], reflective[1] - para.reflective_calibrated[1], beacon[0], beacon[1], proximity[0], proximity[1]);
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
        case 8: 
            printf("\n");
            if(timestamp ==40)
            {
            }

            if(irobot->isEthernetPortConnected(ScoutBot::FRONT))
            {
                SetRGBLED(2, RED, RED,RED,RED);
            }
            else
                SetRGBLED(2, 0,0,0,0);
            break;

        case 9:
            if(timestamp > 40)
            {
                Robot::SendIRMessage(::FRONT, IR_MSG_TYPE_SCORE, para.ir_msg_repeated_num);
            }
            break;
        case 10:
//            if(timestamp == (uint32_t)para.debug.para[8])
//            {
//                printf("lock motor\n");
//                SetDockingMotor(para.debug.para[0], CLOSE);
//            }
//            else
            if(timestamp == 2 )
            {
                ((ScoutBot*)irobot)->OpenDocking(ScoutBot::Side(para.debug.para[9]));

            }
            else if( timestamp == para.docking_motor_opening_time + 2 || (timestamp > 12 && docking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
            {
                ((ScoutBot*)irobot)->MoveDocking((ScoutBot::Side(para.debug.para[9])),0);
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
                    irobot->SetIRRX(ScoutBot::Side(board_dev_num[i]), false);
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

                //module_failed = true;
                //current_state = LOWERING;
            }
            break;
        case 13: //testing ethernet
            if(timestamp==40)
            {
            for(int i=0;i<NUM_DOCKS;i++)
            {
                printf("%d - Side %d connected: %s activated: %s\n", timestamp, i, irobot->isEthernetPortConnected(ScoutBot::Side(board_dev_num[i])) ? "true":"false",irobot->isSwitchActivated()?"true":"false" );
            }
            }
#define NEIGHBOUR_IP "192.168.0.4"
            if(timestamp % 10 ==0)
            {
                uint8_t data[10]={'h','e','l','l','o','-','K','I','T',0};
                irobot->SendEthMessage(Ethernet::StringToIP(NEIGHBOUR_IP), data, sizeof(data));
            }
            while (irobot->HasEthMessage() > 0)
            {
                uint8_t rx[32];
                auto_ptr<Message> m = irobot->ReceiveEthMessage();
                memcpy(rx, m->GetData(), m->GetDataLength());
                printf("%d -- received data: %s\n", timestamp, rx);
            }
            break;
        case 14: //as docking robot for measureing
            if(timestamp ==32)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("SFSF");
                docked[0]=sym.data;
                //using reflective signals if not set
                for(int i=0; i< NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0 | IRPULSE1);

                SetRGBLED(2, RED,RED,RED,RED);
                //SetRGBLED(0, WHITE,WHITE,WHITE,WHITE);
            }
           
            //send synchronisation signals, using ip_req
            if(msg_ip_addr_received==0)
            {
                if(timestamp > 60 && timestamp % 20 ==5)
                {
                    uint8_t data[5];
                    data[0] = docked[0]; //TODO: remove this as it is already included when using SendIRMessage
                    memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP, 4);
                    Robot::SendIRMessage(0, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, 0);
                }
            }
            else
                clock++;


            if(clock == para.debug.para[5])
            {
                log = true;
                speed[0] = -20;
                speed[1] = -20;
            }
            else if(clock == para.debug.para[6])
            {
                log = false;
                speed[0] = 0;
                speed[1] = 0;
            }

            if(log)
                Log();

            break;
        case 15: //as recruiting robot for measuring
            if(timestamp ==32)
            {
                OrganismSequence::Symbol sym;
                sym.reBuild("KBAF");
                docked[2]=sym.data;
                //using reflective signals if not set
                for(int i=0; i< NUM_DOCKS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0 | IRPULSE1);
            }

            //received IP_REQ as synchronisation signals
            if(neighbours_IP[2] != 0)
            {
                clock++;
            }

            //start flashing ir led
            if(clock == 2)
            {
                //SetIRLED(2, IRLEDDOCKING, LED1, IRPULSE0 | IRPULSE1);
                SetIRLED(2, IRLEDPROXIMITY, LED0|LED2, 0);
            }


            if(clock == para.debug.para[5])
                log = true;
            else if(clock == para.debug.para[6])
                log = false;

            if(log)
                Log();

            break;
        case 16://Test IRComm as sender
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[0], IRLEDOFF, LED1, IRPULSE0 | IRPULSE1); //TODO: better to switch off ir pulse
                SetRGBLED(para.debug.para[0], 0,0,0,0);
            }
            if(timestamp % RECRUITMENT_SIGNAL_INTERVAL == 0)
            {
                OrganismSequence::Symbol sym(0);
                sym.reBuild("SFSB");
                Robot::BroadcastIRMessage(para.debug.para[0], IR_MSG_TYPE_RECRUITING, sym.data, 0);
            }
            break;
        case 17://Test IRComm as listener
            if(timestamp ==2)
                irobot->SetIRRX(ScoutBot::Side(board_dev_num[para.debug.para[0]]), para.debug.para[1]);
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
                speed[0] = para.debug.para[4];
                speed[1] = para.debug.para[5];
                //  printf("Move motors at speed (%d %d)\n", para.debug.para[4], para.debug.para[5]);
            }
            break;
        case 21://test RGB
            if(timestamp==2)
                SetRGBLED(para.debug.para[4], para.debug.para[0] * BLUE, para.debug.para[1]*BLUE, para.debug.para[2]*BLUE, para.debug.para[3]*BLUE);
            break;

        case 22://calibrate docking units
            if(timestamp ==2)
            {
                SetIRLED(para.debug.para[9], IRLEDOFF, 0, IRPULSE0|IRPULSE1);
                irobot->enableDockingSense(para.debug.para[9], true);
            }
            else if(timestamp == 30)
            {
                ((ScoutBot*)irobot)->CalibrateDocking(ScoutBot::Side(para.debug.para[9]));
                printf("start to calibrate docking unit %d\n", para.debug.para[9]);
            }
            break;
        case 23://open docking units
            {
                if(timestamp ==2)
                {
                    //SetIRLED(para.debug.para[9], IRLEDOFF, 0, IRPULSE0|IRPULSE1);
                    irobot->enableDockingSense(para.debug.para[9], true);
                }
                else if(timestamp == 30)
                {
                    ((ScoutBot*)irobot)->OpenDocking(ScoutBot::Side(para.debug.para[9]));
                    //SetDockingMotor(para.debug.para[9], OPEN);
                    printf("open docking unit %d\n", para.debug.para[9]);
                }
                uint8_t rev = ((ScoutBot*)irobot)->GetDScrewRevolutions(ScoutBot::Side(para.debug.para[9]));
                uint8_t ise = ((ScoutBot*)irobot)->GetDScrewISense(ScoutBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );
            }
            break;
        case 24://close docking units
            {
                if(timestamp ==2)
                {
                    //SetIRLED(para.debug.para[9], IRLEDOFF, 0, IRPULSE0|IRPULSE1);
                    irobot->enableDockingSense(para.debug.para[9], true);
                }
                else if(timestamp == 30)
                {
                    ((ScoutBot*)irobot)->CloseDocking(ScoutBot::Side(para.debug.para[9]));
                    //SetDockingMotor(para.debug.para[9], CLOSE);
                    printf("close docking unit %d\n", para.debug.para[9]);
                }
                uint8_t rev = ((ScoutBot*)irobot)->GetDScrewRevolutions(ScoutBot::Side(para.debug.para[9]));
                uint8_t ise = ((ScoutBot*)irobot)->GetDScrewISense(ScoutBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );
            }
            break;

        default:
            break;
    }

}

int RobotSCOUT::in_docking_region(int x[4])
{
    if(   x[0] > para.docking_reflective_offset1 
            && x[1] > para.docking_reflective_offset2) 
        return 1;
    else
        return 0;
}

int RobotSCOUT::in_locking_region(int x[4])
{

    if( x[0] > para.locking_proximity_offset1 
            && x[0] < para.locking_proximity_offset1 + para.locking_proximity_diff1
            && x[1] > para.locking_proximity_offset2 
            && x[1] < para.locking_proximity_offset2 + para.locking_proximity_diff2)
        return 1;
    else 
        return 0;
}

void RobotSCOUT::Log()
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

