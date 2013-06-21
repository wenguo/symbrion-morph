#include "robot_KIT.hh"
#include "og/organism_sample.hh"

RobotKIT::RobotKIT(KaBot * robot):Robot()
{
    SPIVerbose = QUIET;

    if(name)
        free(name);
    name = strdup("RobotKIT");
    type = ROBOT_KIT;

    irobot = robot;

    //point to right SPI device number
    board_dev_num[::FRONT] = KaBot::FRONT;
    board_dev_num[::RIGHT] = KaBot::RIGHT;
    board_dev_num[::BACK] = KaBot::REAR;
    board_dev_num[::LEFT] = KaBot::LEFT;

    robot_side_dev_num[KaBot::FRONT] = ::FRONT;
    robot_side_dev_num[KaBot::RIGHT] = ::RIGHT;
    robot_side_dev_num[KaBot::REAR] = ::BACK;
    robot_side_dev_num[KaBot::LEFT] = ::LEFT;

    LED0 = 0x1;
    LED1 = 0x2;
    LED2 = 0x4;

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
        irobot->SetPrintEnabled(i, false); 
        irobot->enableDockingSense(i, true);
    }

    irobot->EnableMotors(true);
    

}

void RobotKIT::Reset()
{
    irobot->MSPReset();
}

void RobotKIT::EnablePowerSharing(int side, bool on)
{
    irobot->EnablePowerSharing(KaBot::Side(board_dev_num[side]), on);
}

void RobotKIT::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    KaBot::Side board = (KaBot::Side)board_dev_num[channel];
    irobot->SetIRLED(board, led);
    irobot->SetIRPulse(board, pulse_led | IRPULSE2);
    irobot->SetIRMode(board, mode);

    if(mode !=IRLEDOFF)
        irobot->SetIRRX(KaBot::Side(board), led & LED1 ? false : true);
    else
        irobot->SetIRRX(KaBot::Side(board), led & LED1 ? true : false);

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
    irobot->SetLED(KaBot::Side(board), tl, tr, bl, br);
    RGBLED_status[channel] = tl|tr|bl|br;

}

void RobotKIT::SetSpeed(int speed0, int speed1, int speed2)
{

    if(!para.locomotion_motor_enabled)
        return;

    if(speed0 > 100)
        speed0 = 100;
    if(speed0 < -100)
        speed0 = -100;
    if(speed1 > 100)
        speed1 = 100;
    if(speed1 < -100)
        speed1 = -100;
    if(speed2 > 100)
        speed2 = 100;
    if(speed2 < -100)
        speed2 = -100;
    
    printf("Move motors at speed: (%d %d %d) direction: %d ", speed[0], speed[1], speed[2], direction);

    if(fabs(speed2) > 10)
    {
        printf("1: real value: %d %d\n ",para.speed_sideward * speed2 * direction, -para.speed_sideward* speed2 * direction);
        irobot->MoveScrewFront(para.speed_sideward * speed2 * direction);
        irobot->MoveScrewRear(-para.speed_sideward* speed2 * direction);
    }
    else
    {
        printf("2: real value: %d %d\n", speed0 * direction, speed1 * direction);
        irobot->MoveScrewFront(speed0 * direction);
        irobot->MoveScrewRear(speed1 * direction);
    }


}



bool RobotKIT::SetDockingMotor(int channel, int status)
{
    CPrintf4(SCR_BLUE,"%d -- side %d set motor %#x, (status: %#x)", timestamp, channel, status, locking_motors_status[channel]);

    if(status != STOP)
    {
        //closed -> open?
        if(locking_motors_status[channel] == CLOSED  && status == OPEN)
        {
            //change status to be opening
            locking_motors_status[channel] = OPENING; //clear first
            if(para.locking_motor_enabled[channel])
            {
                if(para.locking_motor_nonreg[channel])
                {
                    CPrintf1(SCR_RED,"%d: open docking motor -- non regulated\n", timestamp);
                    irobot->MoveDocking(KaBot::Side(board_dev_num[channel]), -1);        
                }
                else
                {
                    CPrintf1(SCR_RED,"%d: open docking motor -- regulated\n",timestamp);
                    irobot->OpenDocking(KaBot::Side(board_dev_num[channel]));
                }
            }
        }
        //or open -> close
        else if(locking_motors_status[channel]== OPENED &&  status == CLOSE )
        {
            //change status to be closing
            locking_motors_status[channel] = CLOSING; //clear first
            if(para.locking_motor_enabled[channel])
            {
                if(para.locking_motor_nonreg[channel])
                {
                    CPrintf1(SCR_RED,"%d: close docking motor -- non regulated\n", timestamp);
                    irobot->MoveDocking(KaBot::Side(board_dev_num[channel]), 1);        
                }
                else
                {
                    CPrintf1(SCR_RED,"%d: close docking motor -- regulated\n",timestamp);
                    irobot->CloseDocking(KaBot::Side(board_dev_num[channel]));
                }
            }
        }
        else
            return false;
        
        SetRGBLED((channel+1)%4, GREEN, GREEN, RED, RED);
    }
    else
    {
        // opeing/open -->stop
        if(locking_motors_status[channel] == OPENED || 
                locking_motors_status[channel] == OPENING )
        {
            locking_motors_status[channel]=OPENED;
        }
        // closing/closed -> stop ?
        else
        {
            locking_motors_status[channel] = CLOSED;
        }
        irobot->MoveDocking(KaBot::Side(board_dev_num[channel]), 0);        
        SetRGBLED((channel+1)%4, 0, 0, 0, 0);
    }

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

bool RobotKIT::MoveHingeMotor(int command[4])
{
//    printf("%d: hinge command: %d %d %d %d\n", timestamp, command[0], command[1], command[2], command[3]);
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
    IRValues ret_A = irobot->GetIRValues(KaBot::FRONT);
    IRValues ret_B = irobot->GetIRValues(KaBot::LEFT);
    IRValues ret_C = irobot->GetIRValues(KaBot::REAR);
    IRValues ret_D = irobot->GetIRValues(KaBot::RIGHT);
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

    uint8_t ethernet_status=0;
    uint8_t isense=0;
  //      printf("Isense: ");
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(irobot->isEthernetPortConnected(KaBot::Side(board_dev_num[i])))
            ethernet_status |= 1<<i;

        if(irobot->GetDScrewISense(KaBot::Side(board_dev_num[i])) > para.locking_motor_isense_threshold)
            isense |= 1<<i;

               //     printf("%d\t", irobot->GetDScrewISense(KaBot::Side(board_dev_num[i])));
    }
 //       printf("\n");

    ethernet_status_hist.Push2(ethernet_status);
    locking_motor_isense_hist.Push2(isense);

    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-para.reflective_calibrated[i]);
        ambient_hist[i].Push(para.ambient_calibrated[i] - ambient[i]);
    }
}

void RobotKIT::UpdateActuators()
{
    CheckDockingMotor();
    //CheckHingeMotor();
    SetSpeed(speed[0], speed[1],speed[2]); 
}

// for self-repair
void RobotKIT::UpdateFailures()
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

void RobotKIT::Avoidance()
{
    /*
    static bool triggered = false;
    if(ambient_hist[0].Avg() > 1000 || ambient_hist[1].Avg() > 1000)
        triggered = true;
    else if(ambient_hist[4].Avg() > 1000 || ambient_hist[5].Avg() > 1000)
        triggered = false;
    if(!triggered)
        return;*/

    Robot::Avoidance();
}

void RobotKIT::Exploring()
{
    Avoidance();
}


void RobotKIT::Resting()
{
    resting_count++;
    if(resting_count > 100)
    {
        current_state = FORAGING;
        last_state = RESTING;
        resting_count = 0;
    }

}


void RobotKIT::Foraging()
{
    speed[0]=0;
    speed[1]=0;

    foraging_count++;
    /*
    //time up?
    if(foraging_count >= para.foraging_time)
    {
    foraging_count = 0;//DEFAULT_FORAGING_COUNT;
    waiting_count = 0;//DEFAULT_WAITING_COUNT;

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
        else if(organism_found || beacon_signals_detected)
        {
            for(int i=0;i<NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);

            assembly_count = 0;
            current_state = ASSEMBLY;
            last_state = FORAGING;
        }

    }


}
void RobotKIT::Waiting()
{
    speed[0] = 0;
    speed[1] = 0;

    msg_unlockme_received = 0;
    msg_locked_received = 0;
    msg_lockme_received = 0;

    waiting_count++;

    if(waiting_count >= (unsigned int)para.waiting_time)
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

        current_state = ASSEMBLY;
        last_state = WAITING;

        assembly_count = 0;
    }
}

void RobotKIT::Assembly()
{
    speed[0]=0;
    speed[1]=0;
    speed[2]=0;

    assembly_count++;

    if(assembly_count >= (unsigned int) para.assembly_time)
    {
        organism_found = false;

        assembly_count = 0;
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

void RobotKIT::LocateEnergy()
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

void RobotKIT::LocateBeacon()
{
    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];


    int turning = 0;
    if(beacon_signals_detected)
    {
        //set the direction in case it is changed in Avoidance
        direction = assembly_info.side2 == FRONT ? FORWARD : BACKWARD;

        if(id0==0)
        {
            //no signals on other 3 side, except the docking side
            if((beacon_signals_detected & 0xFC) == 0)
            {
                printf("forward or side way\n");
                turning = 0;
            }
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
            //no signals on other 3 side, except the docking side
            if((beacon_signals_detected & 0xCF) == 0x0)
            {
                printf("forward or side way\n");
                turning = 0;
            }
            //no signals on side 1, turn right
            else  if((beacon_signals_detected & 0xC) ==0)
                turning = 1;
            //no signals on side 3, turn left
            else if((beacon_signals_detected & 0xC0) ==0)
                turning = -1;
            else 
                turning = 0;
            //printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tturning: %d\n", beacon[0], beacon[1], beacon[2], beacon[3], beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, turning);
        }

        //printf("max_beacon: %d %d %d %d\tturning: %d\n", max_beacon[0], max_beacon[1], max_beacon[2], max_beacon[3], turning);
        if(turning == -1)
        {
            printf("turn %s\n", direction ? "left" :"right");
            speed[0] = direction > 0 ? 10 : 40;
            speed[1] = direction > 0 ? -40 : -10;
            speed[2] = 0;
        }
        else if(turning == 1)
        {
            printf("turn %s\n", -direction ? "left" :"right");
            speed[0] = direction > 0 ? 40 : 10;
            speed[1] = direction > 0 ? -10 : -40;
            speed[2] = 0;
        }
        else
        {
            //using hist will filter out the noise from IRComm
            if(beacon_signals_detected_hist.Sum(id0) > 3 || beacon_signals_detected_hist.Sum(id1) > 3)
            {
                int temp = beacon[id1] - beacon[id0];
                speed[0] = para.locatebeacon_forward_speed[0];
                speed[1] = para.locatebeacon_forward_speed[1];
                speed[2] = 35 * sign(temp);
            }
            else
            {
                speed[0] = 25;
                speed[1] = 25;
                speed[2] = 0;
            }
        }
    }
    else
    {
        Avoidance();
    }

    printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tid: %d %d\n", beacon[0], beacon[1], beacon[2], beacon[3], beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, id0, id1);


    //checking
    if((beacon_signals_detected_hist.Sum(id0) >= 6 || beacon_signals_detected_hist.Sum(id1) >=6)) 
    {
        if(ethernet_status_hist.Sum(assembly_info.side2) > 4)
        {
            SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            docking_count = 0;
            docking_failed_reverse_count = 0;

            docking_blocked = false;

            current_state = DOCKING;
            last_state = LOCATEBEACON;

            SetRGBLED(0,0,0,0,0);
        }
        else if((beacon_signals_detected == (1<<id0| 1<<id1)) && beacon[id0] >= 10 && beacon[id1] >= 10)  
        {
            current_state = ALIGNMENT;
            last_state = LOCATEBEACON;

            //using reflective signals if not set
            for(int i=0; i< NUM_DOCKS;i++)
                SetIRLED(i, IRLEDOFF, LED1, IRPULSE0 | IRPULSE1);

        } 
    }
    else if (beacon_signals_detected ==0 )
    {
        locatebeacon_count++;

        if(locatebeacon_count >= para.locatebeacon_time)
        {
            current_state = ASSEMBLY;
            last_state = LOCATEBEACON;

            organism_found = false;
            assembly_count = 0;
            assembly_info = OrganismSequence::Symbol(0);
            locatebeacon_count = 0;

            for(int i=0;i<NUM_DOCKS;i++)
            {
                SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);
                SetRGBLED(i, 0, 0, 0, 0);
            }
        }
    }

}

void RobotKIT::Alignment()
{
    speed[0] =  para.aligning_forward_speed[0];
    speed[1] =  para.aligning_forward_speed[1];
    speed[2] = 0;

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    int temp = beacon[id1]-beacon[id0]; 
    int temp2 = (reflective_hist[id1].Avg())-(reflective_hist[id0].Avg());
    int temp_max = std::max(beacon[id1], beacon[id0]);

    if(beacon_signals_detected)
    {
        // Far away from recruiting robot - move sideways or forward
        if( std::max(reflective_hist[id0].Avg(), reflective_hist[id1].Avg()) < 200 )
        {
            std::cout << "FAR " << " beacon: " << beacon[id0] << "\t" << beacon[id1]
                << " reflective: " << reflective_hist[id0].Avg() << "\t" << reflective_hist[id1].Avg() << std::endl;
            if( abs(temp) > 0.1 * temp_max )
            {
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 35 * sign(temp);
            }
            else
            {
                speed[0] =  para.aligning_forward_speed[0];
                speed[1] =  para.aligning_forward_speed[1];
                speed[2] = 0;
            }
        }
        //getting close to robot, but not too close
        else if( (assembly_info.type1 == ROBOT_AW && (std::max(reflective_hist[id0].Avg(), reflective_hist[id1].Avg()) < 500 || abs(temp2) > 250))
                || (assembly_info.type1 != ROBOT_AW && std::max(reflective_hist[id0].Avg(), reflective_hist[id1].Avg()) < 800 ))
        {
            if(std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg()) < 0)
            {
                speed[0]=0;
                speed[1]=0;
                speed[2]=0;
                for(int i=0;i<NUM_DOCKS;i++)
                {
                    if(timestamp % 2 ==0)
                        SetRGBLED(i, RED,RED,RED,RED);
                    else
                        SetRGBLED(i, 0,0,0,0);
                }
            }
            else if( abs(temp2) > 150)
            {           
                std::cout << " Zone 1, adjusting orientantion " << " beacon: " << beacon[id0] << "\t" << beacon[id1]
                    << " reflective: " << reflective_hist[id0].Avg() << "\t" << reflective_hist[id1].Avg() << std::endl;
                if(temp2 > 0)
                {
                    speed[0] = direction > 0 ? para.docking_turn_left_speed[0] : (direction * para.docking_turn_left_speed[1]);
                    speed[1] = direction > 0 ? para.docking_turn_left_speed[1] : (direction * para.docking_turn_left_speed[0]);
                    speed[2] = 0;
                }
                else
                {   
                    speed[0] = direction > 0 ? para.docking_turn_right_speed[0] : (direction * para.docking_turn_right_speed[1]);
                    speed[1] = direction > 0 ? para.docking_turn_right_speed[1] : (direction * para.docking_turn_right_speed[0]);
                    speed[2] = 0;
                }
            }
            else
            {
                if( abs(temp) > 0.2 * temp_max )
                {
                    std::cout << " Zone 1, adjusting pose " << " beacon: " << beacon[id0] << "\t" << beacon[id1]
                        << " reflective: " << reflective_hist[id0].Avg() << "\t" << reflective_hist[id1].Avg() << std::endl;
                    speed[0] = 0;
                    speed[1] = 0;
                    speed[2] = 35 * sign(temp);
                }
                else
                {
                    speed[0] = 35;
                    speed[1] = 35;
                    speed[2] = 0;
                }
            }

        }
        // very close to another robot
        else
        {
            std::cout << " Blocked or very close " << " beacon: " << beacon[id0] << "\t" << beacon[id1]
                << " reflective: " << reflective_hist[id0].Avg() << "\t" << reflective_hist[id1].Avg() << std::endl;
            speed[0]=0;
            speed[1]=0;
            speed[2]=0;

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
    }
    else
    {
        current_state = LOCATEBEACON;
        last_state = ALIGNMENT;

        locatebeacon_count = 0;
    }

    printf("speed: %d %d %d\n", speed[0], speed[1], speed[2]);
}

void RobotKIT::Recover()
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

    if(last_state == DOCKING)
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
            //blocked
            else
            {
                speed[0] = para.aligning_reverse_speed[0];
                speed[1] = para.aligning_reverse_speed[1];

                if(timestamp % 5 ==0)
                    Robot::BroadcastIRMessage(assembly_info.side2, IR_MSG_TYPE_DOCKING_SIGNALS_REQ, 0);

                //TODO rewrite this 
                const int weight_left[8] = {-3,3, 0,0,-3,3,0,0};
                const int weight_right[8] = {3,-3,0,0,3,-3,0,0};
                for(int i=id0;i<=id1;i++)
                {
                    speed[0] += reflective_hist[i].Avg() * weight_left[i]>>8;
                    speed[1] += reflective_hist[i].Avg() * weight_right[i]>>8;
                }
            }
        }
        else if( recover_count == para.aligning_reverse_time )
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
                    docking_trials = 0;

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

#define MOVE_LEFT 5
#define MOVE_RIGHT 6
void RobotKIT::Docking()
{
    const char *docking_status_name[] ={"turn left", "turn right", "move forward", "move backward", "check", "move left", "move right" };

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    //TODO, remove this two static variables as they may cause problems
    static bool synchronised = false;
    static  int status = MOVE_FORWARD;
    static  int last_status = MOVE_FORWARD;
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
                last_state = DOCKING;
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
                irobot->SetIRRX(KaBot::Side(board_dev_num[assembly_info.side2]), false);

                //Robot::SendIRMessage(assembly_info.side2, IR_MSG_TYPE_LOCKME,  para.ir_msg_repeated_num);
                //msg_locked_expected |= 1<<assembly_info.side2;
                SetDockingMotor(assembly_info.side2, CLOSE);

                synchronised = false;
                current_state = LOCKING;
                last_state = DOCKING;

                SetRGBLED(assembly_info.side2, 0,0,0,0);
            }
        }
    }
    else if(ethernet_status_hist.Sum(assembly_info.side2) > 0) 
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

        //blocked or something goes wrong
        if(docking_blocked)
        {
            docking_blocked = false;
            blocking_count=0;
            synchronised = false;
            docking_trials++;
            current_state = RECOVER;
            last_state = DOCKING;
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
                docking_count +=36; // 18x2
                //docking_blocked = true;
            }
            else if(abs(proximity_diff) > 0.5 * std::max(proximity[id0], proximity[id1]) )
            {
                printf("proximity signals are significant different %d %d\n", proximity[id0], proximity[id1]);
                docking_count +=36;
                //docking_blocked = true;
            }
            else if(std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg())<0)
            {
                printf("signal interference, reflective gives negative values %d %d\n", reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
                docking_count +=36;
                //docking_blocked = true;
            }
            else if(docking_count >=36) 
            {
                //skip first few step for fix pattern behaviour (checking -- forward -- moving1 --checking -- moving2)
                //make a fix pattern movement
                if(docking_count % 6 == 0)
                {
                    status = CHECKING;
                }
                else if(docking_count % 18 == 3)
                {
                    status = MOVE_FORWARD;
                }
                else if(docking_count % 18 == 9 || docking_count % 18 == 15)
                {
                    if(assembly_info.type1 == ROBOT_AW)
                    {
                        if(abs(proximity_diff)> 400 || abs(reflective_diff) > 1200)
                            status = MOVE_BACKWARD;
                        else if(reflective_diff > 300)
                            status = TURN_LEFT;
                        else if(reflective_diff < -300)
                            status = TURN_RIGHT;
                        else
                            status = MOVE_FORWARD;
                    }
                    else if(assembly_info.type1 == ROBOT_KIT || assembly_info.type1 == ROBOT_SCOUT)
                    {

                        if(std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg())<0)
                            status = MOVE_BACKWARD;
                        else if(reflective_diff > 300)
                            status = TURN_LEFT;
                        else if(reflective_diff < -300)
                            status = TURN_RIGHT;
                        else
                            status = MOVE_FORWARD;

                        last_status = status;

                    }
                }

                switch (status)
                {
                    case TURN_RIGHT:
                        speed[0] = direction > 0 ? para.docking_turn_right_speed[0] : 
                            (direction * para.docking_turn_right_speed[1]);
                        speed[1] = direction > 0 ? para.docking_turn_right_speed[1] : 
                            (direction * para.docking_turn_right_speed[0]);
                        speed[2] = 0;
                        break;
                    case TURN_LEFT:
                        speed[0] = direction > 0 ? para.docking_turn_left_speed[0] : 
                            (direction * para.docking_turn_left_speed[1]);
                        speed[1] = direction > 0 ? para.docking_turn_left_speed[1] : 
                            (direction * para.docking_turn_left_speed[0]);
                        speed[2] = 0;
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
                    case MOVE_LEFT:
                        speed[0] = 0;
                        speed[1] = 0;
                        speed[2] = -12;
                        break;
                    case MOVE_RIGHT:
                        speed[0] = 0;
                        speed[1] = 0;
                        speed[2] = -16;
                        break;
                    default:
                        break;
                }
                printf(" %d Docking routine %#x (%s) speed (%d %d %d) proximity (%d %d) reflective(%d %d)\n", docking_count %18, status, docking_status_name[status], speed[0], speed[1], speed[2], proximity[id0], proximity[id1], reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
            }
        }
    }
}

void RobotKIT::Locking()
{
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;

    int docking_side = assembly_info.side2;

    SetRGBLED(docking_side, WHITE, WHITE, WHITE, WHITE);//sometimes, rgb leds are switched off for unknow reason


    //docking motor is done?
    if(locking_motors_status[docking_side] == CLOSED)
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

            EnablePowerSharing(docking_side, true);

            //start IPC thread, as a client 
            commander_IPC.Start(IPToString(commander_IP), commander_port, false);
        }
    }
}

void RobotKIT::Recruitment()
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
                    (msg_docking_signal_req_received & (1<<i)) || (msg_assembly_info_req_received & (1<<i)))
            {
                msg_docking_signal_req_received &= ~(1<<i);

                msg_assembly_info_req_expected |= 1<<i;
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDDOCKING, LED1, 0); 
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            else
            {
                if(ethernet_status_hist.Sum(i) > 1)
                    msg_assembly_info_req_expected |= 1<<i;

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
                    recruitment_count[i]=0;
                    printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
                }
            }
            else if(msg_guideme_received & (1<<i))
            {
                recruitment_count[i]=0;
                if(it1->getSymbol(0).type2 == ROBOT_SCOUT)
                    msg_locked_expected |= 1<<i;
                else if(it1->getSymbol(0).type2 == ROBOT_AW)
                    msg_lockme_expected |= 1<<i;
                msg_guideme_received &= ~(1<<i);
                msg_guideme_received &= ~(1<<i);
                guiding_signals_count[i]=0;
                recruitment_stage[i]=STAGE2;
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
                irobot->SetIRRX(KaBot::Side(board_dev_num[i]), false);
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }

            if(msg_assembly_info_req_received & (1<<i))
            {
                if(it1->getSymbol(0).type2 != ROBOT_AW)
                    msg_locked_expected |= 1<<i;
                else
                    msg_lockme_expected |= 1<<i;
                SetIRLED(i, IRLEDOFF, LED0|LED2, 0); //switch docking signals 2 on left and right leds
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
                irobot->SetIRRX(KaBot::Side(board_dev_num[i]), true);
                printf("%d -- Recruitment: channel %d  recieved docking signals request, switch back to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
            }
            //nothing happens, assumed wrong robots docked, back to stage 0
            else if(guiding_signals_count[i] >= (uint32_t)para.recruiting_guiding_signals_time)
            {
                guiding_signals_count[i] = 0;
                recruitment_stage[i]=STAGE0;
                SetIRLED(i, IRLEDOFF, 0, 0);
                irobot->SetIRRX(KaBot::Side(board_dev_num[i]), true);
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
                irobot->SetIRRX(KaBot::Side(board_dev_num[i]), true);
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
            if(locking_motors_status[i] == CLOSED && docked[i]==0)
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
                memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP.i32, 4);
                Robot::SendIRMessage(i, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, 0);
            }
            //get new ip address?
            else if(msg_ip_addr_received & (1<<i))
            {
                msg_ip_addr_received &= ~(1<<i);

                std::vector<uint8_t> root_IPs;
                root_IPs.push_back(uint8_t((my_IP.i32 >>24) & 0xFF));
                root_IPs.push_back(uint8_t((neighbours_IP[i].i32>>24) & 0xFF));
                mytree.setBranchRootIPs(robot_side(i),root_IPs);
                printf("%d: set root IPs %d %d\n", timestamp,root_IPs[0], root_IPs[1]);

                //remove branches since it has been sent to newly joined robot
                erase_required = true;

                EnablePowerSharing(i, true);

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

        printf("my IP is %s\n", IPToString(my_IP));
        for(int i=0;i<NUM_DOCKS;i++)
        {
            printf("neighbour %d's IP is %s\n", i, IPToString(neighbours_IP[i]));
            SetRGBLED(i, 0, 0, 0, 0);
        }

        if(seed)
            commander_IPC.Start(IPToString(commander_IP), commander_port, false);
    }

}

void RobotKIT::Undocking()
{
    Robot::Undocking();
}

void RobotKIT::Debugging()
{
    // speed[0] = 0;
    // speed[1] = 0;

    //printf("%d Debuging %d:\t", timestamp,para.debug.mode);
    static int clock=0;
    static bool log=false;

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
                irobot->SetIRRX(KaBot::Side(board_dev_num[0]), false);
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
                    irobot->SetIRRX(KaBot::Side(board_dev_num[i]), false);
                }

                printf("my IP is %s\n", IPToString(my_IP));

                OrganismSequence::Symbol sym;
                sym.type1 = ROBOT_SCOUT;
                sym.side1 = ::FRONT;
                sym.type2 = ROBOT_SCOUT;
                sym.side2 = ::FRONT;
                uint8_t data[5];
                data[0] = sym.data;
                docked[0]=sym.data;
                memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP.i32, 4);
                Robot::SendIRMessage(::FRONT, IR_MSG_TYPE_IP_ADDR_REQ, data, 5, para.ir_msg_repeated_num);
            }

            break;
        case 7: // measuring beacon signals
            //skip the fist few seconds
            if(timestamp <40)
                return;

            if(timestamp == 40)
            {
                for(int i=0;i<NUM_IRS;i++)
                    SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);
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
            if(timestamp>40)
                Log();
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
        case 8: 
            printf("\n");
            if(timestamp ==40)
            {
            }

            if(irobot->isEthernetPortConnected(KaBot::FRONT))
            {
                SetRGBLED(2, RED, RED,RED,RED);
            }
            else
                SetRGBLED(2, 0,0,0,0);
            break;

        case 9:
            if(timestamp > 40)
            {
                Robot::SendIRMessage(::FRONT, MSG_TYPE_SCORE, para.ir_msg_repeated_num);
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
                ((KaBot*)irobot)->OpenDocking(KaBot::Side(para.debug.para[9]));

            }
            else if( timestamp == para.locking_motor_opening_time + 2 || (timestamp > 12 && locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
            {
                ((KaBot*)irobot)->MoveDocking((KaBot::Side(para.debug.para[9])),0);
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
                    irobot->SetIRRX(KaBot::Side(board_dev_num[i]), false);
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
                    printf("%d - Side %d connected: %s activated: %s\n", timestamp, i, irobot->isEthernetPortConnected(KaBot::Side(board_dev_num[i])) ? "true":"false",irobot->isSwitchActivated()?"true":"false" );
                }
            }
#define NEIGHBOUR_IP "192.168.0.4"
            if(timestamp % 10 ==0)
            {
                uint8_t data[10]={'h','e','l','l','o','-','K','I','T',0};
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
                    memcpy((uint8_t*)&data[1], (uint8_t*)&my_IP.i32, 4);
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
                irobot->SetIRRX(KaBot::Side(board_dev_num[para.debug.para[0]]), para.debug.para[1]);
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
            if(timestamp <= 20 )
            { 
                speed[0] = 0;//para.debug.para[4];
                speed[1] = 0;//para.debug.para[5];
                speed[2] = -20;//para.debug.para[6];
            }
            else if(timestamp <=22)
            { 
                speed[0] = -20;//para.debug.para[4];
                speed[1] = -20;//para.debug.para[5];
                speed[2] = 0;//para.debug.para[6];
            }
            else if(timestamp <=40)
            { 
                speed[0] = 0;//para.debug.para[4];
                speed[1] = 0;//para.debug.para[5];
                speed[2] = 20;//para.debug.para[6];
            }
            else if(timestamp <=42)
            {
                speed[0] = 20;//para.debug.para[4];
                speed[1] = 20;//para.debug.para[5];
                speed[2] = 0;//para.debug.para[6];
            }
            else
                timestamp = 0;


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
                ((KaBot*)irobot)->CalibrateDocking(KaBot::Side(para.debug.para[9]));
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
                    ((KaBot*)irobot)->OpenDocking(KaBot::Side(para.debug.para[9]));
                    printf("open docking unit %d\n", para.debug.para[9]);
                }
                else if(timestamp ==70 || (timestamp > 40 &&locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
                {
                    ((KaBot*)irobot)->MoveDocking(KaBot::Side(para.debug.para[9]), 0);
                    printf("stop docking unit %d\n", para.debug.para[9]);
                }

                uint8_t rev = ((KaBot*)irobot)->GetDScrewRevolutions(KaBot::Side(para.debug.para[9]));
                uint8_t ise = ((KaBot*)irobot)->GetDScrewISense(KaBot::Side(para.debug.para[9]));
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
                    ((KaBot*)irobot)->CloseDocking(KaBot::Side(para.debug.para[9]));
                    printf("close docking unit %d\n", para.debug.para[9]);
                }
                else if(timestamp ==70 || (timestamp > 40 &&locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
                {
                    ((KaBot*)irobot)->MoveDocking(KaBot::Side(para.debug.para[9]), 0);
                    printf("stop docking unit %d\n", para.debug.para[9]);
                }
                uint8_t rev = ((KaBot*)irobot)->GetDScrewRevolutions(KaBot::Side(para.debug.para[9]));
                uint8_t ise = ((KaBot*)irobot)->GetDScrewISense(KaBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );
            }
            break;
        case 25:
            {
                if(timestamp ==2)
                {
                    irobot->enableDockingSense(para.debug.para[9], true);
                    SetDockingMotor(KaBot::Side(para.debug.para[9]), CLOSE);
                }
                else if(timestamp == 100)
                {
                    irobot->enableDockingSense(para.debug.para[9], true);
                    SetDockingMotor(KaBot::Side(para.debug.para[9]), OPEN);
                }
                
                uint8_t rev = ((KaBot*)irobot)->GetDScrewRevolutions(KaBot::Side(para.debug.para[9]));
                uint8_t ise = ((KaBot*)irobot)->GetDScrewISense(KaBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );

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
                if(timestamp ==7)
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

                        if(it->getSymbol(0).type2 == ROBOT_AW)
                        {
                            unlocking_required[branch_side] = true;
                            locking_motors_status[branch_side] = CLOSED;
                        }

                        msg_subog_seq_expected |= 1<<branch_side;

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
                        
                        unlocking_required[parent_side] = true;
                        locking_motors_status[parent_side] = CLOSED;
                        msg_subog_seq_expected |= 1<<parent_side;

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

void RobotKIT::Log()
{
    int id0=para.debug.para[7];
    int id1=para.debug.para[8];
    if (logFile.is_open())
    {
        //logFile << timestamp << "\t" << state_names[current_state] <<"\t";
        //logFile << reflective_hist[id0].Avg()<<"\t";
        //logFile << reflective_hist[id1].Avg()<<"\t";
        //logFile << beacon[id0]<<"\t";
        //logFile << beacon[id1]<<"\t";
        //logFile << ambient_hist[id0].Avg()<<"\t";
        //logFile << ambient_hist[id1].Avg()<<"\t";
        //logFile << proximity[id0]<<"\t";
        //logFile << proximity[id1]<<"\t";
        //logFile << std::endl;
        logFile << timestamp << "\t";
        for(int i=0;i<NUM_IRS;i++)
            logFile << beacon[i]<<"\t";
        logFile<<std::endl;
    }

}
