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

    action_demo_index[0][0] = 0;
    action_demo_index[0][1] = 8;
    action_demo_index[1][0] = 9;
    action_demo_index[1][1] = 10;
    action_demo_index[2][0] = 11;
    action_demo_index[2][1] = 14;

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
    irobot->EnableHallSensors(true);

    //   Ethernet::disableSwitch();
}

void RobotSCOUT::Reset()
{
    irobot->MSPReset();
}

void RobotSCOUT::Pause(bool flag)
{
    irobot->pauseSPI(flag);
}

bool RobotSCOUT::isPaused()
{
    return irobot->isSPIPaused();
}

void RobotSCOUT::EnablePowerSharing(int side, bool on)
{
    irobot->EnablePowerSharing(ScoutBot::Side(board_dev_num[side]), on);
}

void RobotSCOUT::SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)
{
    int board = board_dev_num[channel];
    irobot->SetIRLED(ScoutBot::Side(board), led);
   // irobot->SetIRPulse(ScoutBot::Side(board), 0);
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

void RobotSCOUT::SetSpeed(int leftspeed, int rightspeed, int speed3)
{
    if(!para.locomotion_motor_enabled)
    {
        return;
    }

    //note that the leftspeed is in fact set to the right motor of scouts, rightspeed is on the left motor
    if(leftspeed > 100)
        leftspeed = 100;
    else if(leftspeed < -100)
        leftspeed = -100;
    if(rightspeed > 100)
        rightspeed = 100;
    else if(rightspeed < -100)
        rightspeed = -100;

    if(leftspeed - rightspeed > 100)
    {
        leftspeed = 50;
        rightspeed = -50;
    }
    else if(rightspeed - leftspeed > 100)
    {
        rightspeed = 50;
        leftspeed = -50;
    }

    irobot->Move(direction * para.scout_wheels_direction[0] * leftspeed, direction * para.scout_wheels_direction[1] * rightspeed * para.aw_adjustment_ratio);
}



bool RobotSCOUT::SetDockingMotor(int channel, int status)
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
                    irobot->MoveDocking(ScoutBot::Side(board_dev_num[channel]), -1);        
                }
                else
                {
                    CPrintf1(SCR_RED,"%d: open docking motor -- regulated\n",timestamp);
                    irobot->OpenDocking(ScoutBot::Side(board_dev_num[channel]));
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
                    CPrintf1(SCR_RED,"%d: open docking motor -- non regulated\n", timestamp);
                    irobot->MoveDocking(ScoutBot::Side(board_dev_num[channel]), 1);        
                }
                else
                {
                    CPrintf1(SCR_RED,"%d: open docking motor -- regulated\n",timestamp);
                    irobot->CloseDocking(ScoutBot::Side(board_dev_num[channel]));
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
        irobot->MoveDocking(ScoutBot::Side(board_dev_num[channel]), 0);

        SetRGBLED((channel+1)%4, 0, 0, 0, 0);
    }

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

bool RobotSCOUT::MoveHingeMotor(int command[4])
{
   // printf("%d: hinge command: %d %d %d %d\n", timestamp, command[0], command[1], command[2], command[3]);
    return true;
}

void RobotSCOUT::UpdateSensors()
{
    if(isPaused())
        return;

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
#if 1
    ambient[6] = ret_D.sensor[0].ambient;
    ambient[7] = ret_D.sensor[1].ambient;
    reflective[6] = ret_D.sensor[0].reflective;
    reflective[7] = ret_D.sensor[1].reflective;
    proximity[6] = ret_D.sensor[0].proximity;
    proximity[7] = ret_D.sensor[1].proximity;
    beacon[6] = ret_D.sensor[0].docking;
    beacon[7] = ret_D.sensor[1].docking;
#else
    ambient[6] = 0;
    ambient[7] = 0;
    reflective[6] = 0;
    reflective[7] = 0;
    proximity[6] = 0;
    proximity[7] = 0;
    beacon[6] = 0;
    beacon[7] = 0;

#endif

    uint8_t ethernet_status=0;
    uint8_t isense=0;
    //    printf("Isense: ");
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(irobot->isEthernetPortConnected(ScoutBot::Side(board_dev_num[i])))
            ethernet_status |= 1<<i;

        if(irobot->GetDScrewISense(ScoutBot::Side(board_dev_num[i])) > para.locking_motor_isense_threshold)
            isense |= 1<<i;

        //        printf("%d\t", irobot->GetDScrewISense(ScoutBot::Side(board_dev_num[i])));
    }
    //    printf("\n");

    ethernet_status_hist.Push2(ethernet_status);
    locking_motor_isense_hist.Push2(isense);

    for(int i=0;i<NUM_IRS;i++)
    {
        reflective_hist[i].Push(reflective[i]-para.reflective_calibrated[i]);
        ambient_hist[i].Push(para.ambient_calibrated[i] - ambient[i]);
    }

    last_encoders = encoders;
    encoders = irobot->GetHallSensorValues2D();

    uint8_t stalled = 0;
    if(speed[0] !=0 && abs(encoders.right - last_encoders.right) < 2)
        stalled |= 0x1;

    if(speed[1] !=0 && abs(encoders.left - last_encoders.left) < 2)
        stalled |= 0x2;

    stalled_hist.Push2(stalled);

    powersource_found = false;
    if(fabs(timestamp_blob_info_updated  - timestamp) < 1)
    {
        if(blob_info[0].blobs[0].size.x > para.blob_threshold[0])
            powersource_found = true;
    }

    if(powersource_found)
        SetRGBLED(1, RED,RED,RED,RED);
    else
        SetRGBLED(1, 0,0,0,0);

}

void RobotSCOUT::UpdateActuators()
{
    if(isPaused())
        return;

    CheckDockingMotor();
    //CheckHingeMotor();
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
    //for demo 
/*    static bool triggered = false;
    if(ambient_hist[0].Avg() > 1000 || ambient_hist[1].Avg() > 1000)
        triggered = true;
    else if(ambient_hist[4].Avg() > 1000 || ambient_hist[5].Avg() > 1000)
        triggered = false;
    if(!triggered)
        return;
*/
    Robot::Avoidance();
    return;
    //default
    if(current_state == UNDOCKING)
        Robot::Avoidance();
    else
    {
        speed[0] = 30;
        speed[1] = 30;

        uint32_t temp=0;

        for(int i=0;i<NUM_IRS;i++)
        {
            if(reflective_hist[i].Avg() > 0)
            {
                temp =  reflective_hist[i].Avg();
                speed[0] +=(para.avoid_weightleft[i] * (temp>>3));
                speed[1] += (para.avoid_weightright[i] * (temp >> 3));
            }
        }
    }

}

void RobotSCOUT::Exploring()
{
    Avoidance();
}


void RobotSCOUT::Resting()
{
    resting_count++;
    if(resting_count > 100)
    {
        current_state = FORAGING;
        last_state = RESTING;
        resting_count = 0;
    }
}

void RobotSCOUT::Foraging()
{
    speed[0]=0;
    speed[1]=0;
    foraging_count++;

    SetRGBLED(2, GREEN,GREEN,GREEN,GREEN);

    static int turn_speed = 35;
    if(foraging_count >= para.foraging_time + 20)
    {
        turn_speed = IRandom(1,100) > 50 ? -35: 35;
        foraging_count = 0;

        for(uint8_t i=0; i< NUM_DOCKS; i++)
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
    }


    if(bumped & 0x33)
    {
        Avoidance();
    }
    else
    {
        speed[0] = 20;
        speed[1] = 20;


        if(foraging_count >= para.foraging_time)
        {
            speed[0] = turn_speed;
            speed[1] = -turn_speed;
        }
                
    }

    if(powersource_found)
    {
        current_state = LOCATEENERGY;
        last_state = FORAGING;
    }
    else if(beacon_signals_detected)
    {
        if(organism_found && assembly_info.type2 ==type)
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
        else if(assembly_info == OrganismSequence::Symbol(0)) 
        {
            for(uint8_t i=0;i<NUM_DOCKS;i++)
            {
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
                if(timestamp % 12 == 3 * i && (beacon_signals_detected & (0x3 << (2*i)))!=0)
                    BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING_REQ,0); 
            }

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

    waiting_count++;

    if(waiting_count >= (uint32_t)para.waiting_time)
    {
        waiting_count = 0;//DEFAULT_WAITING_COUNT;
        foraging_count = 0;//DEFAULT_FORAGING_COUNT;

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

void RobotSCOUT::Assembly()
{
    speed[0]=0;
    speed[1]=0;

    assembly_count++;

    SetRGBLED(2, RED,RED,RED,RED);

    if(assembly_count >= (uint32_t)para.assembly_time)
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
        printf("beacon_signals_detected: %d\n", beacon_signals_detected);
        for(uint8_t i=0;i<NUM_DOCKS;i++)
        {
            SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
            if(timestamp % 12 == 3 * i && (beacon_signals_detected & (0x3 << (2*i)))!=0)
                BroadcastIRMessage(i, IR_MSG_TYPE_RECRUITING_REQ,0); 
        }
    }

    if(bumped & 0x33)
    {
        Avoidance();
    }
    else
    {
        speed[0] = 20;
        speed[1] = 20;
    }

}

void RobotSCOUT::LocateEnergy()
{
    direction = FORWARD;

    speed[0] = 30; //right wheel
    speed[1] = 30; //left wheel
    float p_coeff = 0.1;
    float d_coeff = 0.1;
    
    SetRGBLED(2, WHITE,WHITE,GREEN,GREEN);

    if(organism_found && assembly_info.type2 ==type && beacon_signals_detected)
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
        last_state = LOCATEENERGY;

        locatebeacon_count = 0;

    }
    else if(fabs(timestamp_blob_info_updated  - timestamp) < 2)
    {

        speed[0] -= blob_info[0].blobs[0].offset.x  * p_coeff + blob_info[0].blobs[0].offset_deriv.x * d_coeff;
        speed[1] += blob_info[0].blobs[0].offset.x  * p_coeff + blob_info[0].blobs[0].offset_deriv.x * d_coeff;

        int ch=0,index=0;

        printf("%d: channel %d index %d size: (%d %d) offset: (%d %d) speed: (%d %d)\n",timestamp, ch, index,blob_info[ch].blobs[index].size.x, blob_info[ch].blobs[index].size.y,blob_info[ch].blobs[index].offset.x, blob_info[ch].blobs[index].offset.y, speed[0], speed[1] );

        if(blob_info[0].blobs[0].size.x > para.blob_threshold[1])
        {
            current_state = SEEDING;
            last_state = LOCATEENERGY;
        }

    }
    else if(timestamp - timestamp_blob_info_updated > 30)
    {
        current_state = FORAGING;
        last_state = LOCATEENERGY;
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
        //set the direction in case it is changed in Avoidance
        direction = assembly_info.side2 == FRONT ? FORWARD : BACKWARD;

        speed[0] = para.locatebeacon_forward_speed[0];
        speed[1] = para.locatebeacon_forward_speed[1];

        if(id0==0)
        {
            if((beacon_signals_detected & 0x3) != 0 && (beacon_signals_detected & 0xCC)==0)
                turning = 0;
            //no signals on side 1, turn right 
            else if((beacon_signals_detected & 0xC) ==0)
                turning = -1;
            //no signals on side 3, turn left 
            else if((beacon_signals_detected & 0xC0) ==0)
                turning = 1;
            else 
                turning = 0;
                printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tturning: %d\n", beacon[0], beacon[1], beacon[2], beacon[3],beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, turning);
        }
        else
        {

            if((beacon_signals_detected & 0x30) != 0 && (beacon_signals_detected & 0xCC)==0)
                turning = 0;
            //no signals on side 1, turn right
            else if((beacon_signals_detected & 0xC) ==0)
                turning = 1;
            //no signals on side 3, turn left
            else if((beacon_signals_detected & 0xC0) ==0)
                turning = -1;
            else 
                turning = 0;
            printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\tturning: %d\n", beacon[0], beacon[1], beacon[2], beacon[3],beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0, turning);
        }

        if(turning ==-1)
        {
            speed[0] = -direction * 30;
            speed[1] = direction * 30;
        }
        else if(turning == 1)
        {
            speed[0] = direction *  30;
            speed[1] = -direction *  30;
        }
        else
        {
            //using hist will filter out the noise from IRComm
            if(beacon_signals_detected_hist.Sum(id0) > 2 || beacon_signals_detected_hist.Sum(id1) > 2)
            {
                int shift_factor = 1;
                for(int i=0;i<NUM_IRS;i++)
                {
                    speed[0] += (beacon[i] * para.locatebeacon_weightleft[i]) >>shift_factor;
                    speed[1] += (beacon[i] * para.locatebeacon_weightright[i]) >>shift_factor;
                }
            }
            else
            {
                speed[0] = 20;
                speed[1] = 20;
            }

            printf("%d: speed %d %d\n", timestamp, speed[0], speed[1]);
        }
    }

    //checking
    if((beacon_signals_detected_hist.Sum(id0) >= 6 || beacon_signals_detected_hist.Sum(id1) >=6) && ethernet_status_hist.Sum(assembly_info.side2) > 4) 
    {
        SetIRLED(assembly_info.side2, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
        docking_count = 0;
        docking_failed_reverse_count = 0;

        docking_blocked = false;

        current_state = ALIGNMENT;
        last_state = LOCATEBEACON;

        SetRGBLED(0,0,0,0,0);
    }
    else if((beacon_signals_detected == (1<<id0| 1<<id1)) && beacon[id0] >= 30 && beacon[id1] >= 30)  
    {
        current_state = ALIGNMENT;
        last_state = LOCATEBEACON;

        docking_blocked_hist.Reset();
        //using reflective signals if not set
        for(int i=0; i< NUM_DOCKS;i++)
            SetIRLED(i, IRLEDOFF, LED1, IRPULSE0 | IRPULSE1);

    } 
    else if (beacon_signals_detected ==0 )
    {
        if(locatebeacon_count >= (uint32_t)para.locatebeacon_time)
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
        else
            printf("%d: no signals\n", timestamp);

    }

}

//TODO: cleanup the code
void RobotSCOUT::Alignment()
{
    speed[0] = para.aligning_forward_speed[0];
    speed[1] = para.aligning_forward_speed[1];

    int id0 = docking_approaching_sensor_id[0];
    int id1 = docking_approaching_sensor_id[1];

    int reflective_diff = abs(reflective_hist[id0].Avg() - reflective_hist[id1].Avg());
    int reflective_max = std::max(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
    int reflective_min = std::min(reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
    int beacon_diff = abs(beacon[id0] - beacon[id1]);
    int beacon_max = std::max(beacon[id0], beacon[id1]);
    int beacon_min = std::min(beacon[id0], beacon[id1]);

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
                printf("%d:Assembly_info doesn't match, move away! direction: %d\n", timestamp, direction);
                for(int i=0;i<NUM_DOCKS;i++)
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
                SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, 0);
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
            int blocked_threshold = 200;
            if(assembly_info.type1 == ROBOT_SCOUT)
                blocked_threshold = 400;
            else if(assembly_info.type1 == ROBOT_AW)
                blocked_threshold = 150;
            else
                blocked_threshold = 400;

            //keep ticking if getting very close to something
            if(reflective_max > blocked_threshold && reflective_min > 0)
            {
                blocking_count++;
                printf("%d ticking: %d %d %d\n", timestamp, blocking_count, reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
            }

            //added for scout --> aw docking 
            if((reflective_hist[id0].Avg() > 100 || reflective_hist[id1].Avg() > 100 ) && (reflective_hist[id1 + 1].Avg()  > 100 || reflective_hist[id1+2].Avg() > 100))
            {
                blocking_count += 4 * reflective_hist[id1 + 1].Avg() / 60;
                blocking_count += 4 * reflective_hist[id1 + 2].Avg() / 60;
                printf("my left side is bumping to something, %d %d\n", reflective_hist[id1 + 1].Avg(), reflective_hist[id1 + 2].Avg());
            }

            if(reflective_diff > 300 && reflective_diff > reflective_max * 0.7 && reflective_min > 20)
            {
                //if((reflective_diff > 1000 && reflective_diff > reflective_max * 0.7) ||blocking_count > 30)
                printf("reflective is significant different, %d %d, blocked\n", reflective_hist[id0].Avg(), reflective_hist[id1].Avg());
                blocking_count += 4;
            }
            else if(reflective_min > 300 && beacon_diff > beacon_max * 0.7 && beacon_min > 20)
            {
                printf("robot is close, but beacon signals is significant different, %d %d, blocked\n", beacon[id0], beacon[id1]);
                blocking_count += 4;
            }

            //sometimes, both reflective give negative values
            if(reflective_diff > 700 && reflective_max < 0)
            {
                printf("something unusal happens, give up and retry\n");
                blocking_count +=10;
            }

            //sometimes, scout blocked to the corner of another one
            if((beacon_signals_detected == (1<<id0) || beacon_signals_detected == (1<<id1)) && beacon_diff > 40)
            {
                printf("blocked to one corner of another robot, give up and retry\n");
                blocking_count +=10;
            }


            /*
            //TODO: the readings from hall sensor seems not reliable, sometime it fail to update
            //stalled
            printf("encoders: %d %d\n", encoders.left, encoders.right);
            if(reflective_max > 100 && reflective_max < 1000&& (stalled_hist.Sum(0) > 2 || stalled_hist.Sum(1) > 2))
            {
                printf("robot get stalled\n");
                blocking_count +=10;
            }
            */


            //3 second is allowed until fully docked, otherwise, treated as blocked.
            if(blocking_count > 40)
            {
                printf("blocked for 4 seconds, treated as blocked\n");
                docking_blocked = true;
            }


            if(docking_blocked || beacon_signals_detected == 0)
            {
                docking_blocked = false;
                docking_blocked_hist.Reset();
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
                int shift_factor = 4;

                if(assembly_info.type1 == ROBOT_SCOUT)
                {
                    shift_factor = 5;
                }
                else if(assembly_info.type1 == ROBOT_KIT)
                {
                    shift_factor = 4;
                    if(beacon_max < 30 && reflective_max < 200)
                        shift_factor = 3;
                }
                else if(assembly_info.type1 == ROBOT_AW)
                {
                    if(abs(beacon[id0] - beacon[id1]) < 50)
                        shift_factor = 3;
                }

                for(int i=0;i<NUM_IRS;i++)
                {
                    speed[0] += (beacon[i] * para.aligning_weightleft[i]) >> shift_factor;
                    speed[1] += (beacon[i] * para.aligning_weightright[i]) >> shift_factor;
                }

                //move less agressively when bumping to something;
                int divisor = 200;
                if(assembly_info.type1 == ROBOT_SCOUT)
                    divisor = 3000;
                else if(assembly_info.type1 == ROBOT_AW)
                    divisor = 200;
                else
                    divisor = 1000;

                printf("%d: reflective %d %d %d %d %d %d %d %d, speed %d %d\n", timestamp, reflective_hist[0].Avg(), reflective_hist[1].Avg(), reflective_hist[2].Avg(),reflective_hist[3].Avg(),reflective_hist[4].Avg(),reflective_hist[5].Avg(),reflective_hist[6].Avg(),reflective_hist[7].Avg(),speed[0], speed[1]);
                //take the reflect into accout
                if(assembly_info.type1 == ROBOT_SCOUT)
                {
                    if(reflective_max > 30 && beacon_max > 30)
                    {
                        speed[0] += -1 * (reflective_hist[id0].Avg() - reflective_hist[id1].Avg())/30; 
                        speed[1] += 1 *(reflective_hist[id0].Avg() - reflective_hist[id1].Avg())/30; 

                        //cap
                        if(speed[0] < 20)
                            speed[0]=20;
                        if(speed[0] > 50)
                            speed[0]=50;
                        if(speed[1] < 20)
                            speed[1]=20;
                        if(speed[1] > 50)
                            speed[1]=50;
                    }
                    printf("new speed: %d %d\n", speed[0], speed[1]);

                }
                else if(assembly_info.type1 == ROBOT_KIT)
                {
                    if(reflective_max > 30 && reflective_min < 1000 && beacon_max > 30)
                    {
                        speed[0] += -2 * (reflective_hist[id0].Avg() - reflective_hist[id1].Avg())/30; 
                        speed[1] += 2 *(reflective_hist[id0].Avg() - reflective_hist[id1].Avg())/30; 

                        //cap
                        if(speed[0] < 20)
                            speed[0]=20;
                        if(speed[0] > 50)
                            speed[0]=50;
                        if(speed[1] < 20)
                            speed[1]=20;
                        if(speed[1] > 50)
                            speed[1]=50;
                    }
                    printf("new speed: %d %d\n", speed[0], speed[1]);

                }


                if(reflective_min > 0)
                {
                 //   speed[0] -= 6 * (reflective_max / divisor);
                 //   speed[1] -= 6 * (reflective_max / divisor);
                }

                //added
                if(reflective_hist[id1 + 1].Avg()  > 100 || reflective_hist[id1+2].Avg() > 100)
                {
                   // speed[0] -= 0;
                   // speed[1] -= 0;
                }

                printf("beacon: %d %d %d %d %d %d %d %d (%#x %#x %#x)\n", beacon[0], beacon[1], beacon[2], beacon[3],beacon[4], beacon[5], beacon[6], beacon[7],beacon_signals_detected, beacon_signals_detected & 0xC, beacon_signals_detected & 0xC0);
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
        if(recover_count < (uint32_t) para.aligning_reverse_time + docking_trials * 3)
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

                //TODO: test reverse behaviour with new robots
                const int weight_left[8] = {-3,3, 5,3,-3,3,5,3};
                const int weight_right[8] = {3,-3,-5,-3,3,-3,-5,-3};

                int shift_factor = 6;

                if(recover_count < 15) //only do a small turn if necessary
                {
                    if(assembly_info.type1 != ROBOT_AW)
                        shift_factor = 8;
                    printf("reflective: (");
                    for(int i=id0;i<=id1+2;i++)
                    {
                        speed[0] += weight_left[i] * (reflective_hist[i].Avg() >> shift_factor);
                        speed[1] += weight_right[i] * (reflective_hist[i].Avg() >> shift_factor);
                        printf("%d ", reflective_hist[i].Avg());
                    }
                    printf(") speed: (%d %d)\n", speed[0], speed[1]);
                }
                else
                {
                    shift_factor = std::max(beacon[id0], beacon[id1]) / 10;
                    if(shift_factor >5)
                        shift_factor = 5;
                    //try not straight them, need more test
                    speed[0] += -1 * (beacon[id0] - beacon[id1]) >> shift_factor;
                    speed[1] += 1 * (beacon[id0] - beacon[id1]) >> shift_factor;
                    printf("beacon: (%d %d) speed: (%d %d)\n", beacon[id0], beacon[id1],speed[0], speed[1]);

                }
            }
        }
        else if( recover_count == (uint32_t)para.aligning_reverse_time + docking_trials * 5 )
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

void RobotSCOUT::Docking()
{
    speed[0] = para.docking_forward_speed[0];
    speed[1] = para.docking_forward_speed[1];  

    docking_count++;

    if(ethernet_status_hist.Sum(assembly_info.side2) > 4) 
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
                    (msg_docking_signal_req_received & (1<<i)) || (msg_assembly_info_req_received & (1<<i)))
            {
                msg_docking_signal_req_received &= ~(1<<i);

                msg_assembly_info_req_expected |= 1<<i;
                recruitment_stage[i]=STAGE1;
                SetIRLED(i, IRLEDDOCKING, LED1, 0); 
                printf("%d -- Recruitment: channel %d  switch to Stage%d\n\n", timestamp,i, recruitment_stage[i]);
                printf("robots_in_range %d %d, msg_docking_signals_req %#x\n", robots_in_range_detected_hist.Sum(2*i), robots_in_range_detected_hist.Sum(2*i+1), msg_docking_signal_req_received);
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

            if( recruitment_count[i]++ > (uint32_t)para.recruiting_beacon_signals_time )
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

void RobotSCOUT::Undocking()
{
    Robot::Undocking();
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
                ((ScoutBot*)irobot)->OpenDocking(ScoutBot::Side(para.debug.para[9]));

            }
            else if( timestamp == (uint32_t)para.locking_motor_opening_time + 2 || (timestamp > 12 && locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
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
            if(timestamp  == 2)
            {
                SetRGBLED(2,RED,RED,RED,RED);
            }
            else if(timestamp  >= 10 && timestamp < para.debug.para[9])
            { 
                SetRGBLED(2,0,0,0,0);
                speed[0] =para.debug.para[4];
                speed[1] =  para.debug.para[5];
                speed[2] =para.debug.para[6];
                printf("Move motors at speed (%d %d %d)\n", para.debug.para[4], para.debug.para[5], para.debug.para[6]);
            }
            else if(timestamp  == para.debug.para[9])
            { 
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 0;
                printf("Move motors at speed (%d %d %d)\n", para.debug.para[4], para.debug.para[5], para.debug.para[6]);
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
                    printf("open docking unit %d\n", para.debug.para[9]);
                }
                else if(timestamp ==70 || (timestamp > 40 &&locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
                {
                    ((ScoutBot*)irobot)->MoveDocking(ScoutBot::Side(para.debug.para[9]), 0);
                    printf("stop docking unit %d\n", para.debug.para[9]);
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
                    printf("close docking unit %d\n", para.debug.para[9]);
                }
                else if(timestamp ==70 || (timestamp > 40 &&locking_motor_isense_hist.Sum(para.debug.para[9]) >=2 ))
                {
                    ((ScoutBot*)irobot)->MoveDocking(ScoutBot::Side(para.debug.para[9]), 0);
                    printf("stop docking unit %d\n", para.debug.para[9]);
                }
                uint8_t rev = ((ScoutBot*)irobot)->GetDScrewRevolutions(ScoutBot::Side(para.debug.para[9]));
                uint8_t ise = ((ScoutBot*)irobot)->GetDScrewISense(ScoutBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );
            }
            break;
        case 25:
            {
                if(timestamp ==2)
                {
                    irobot->enableDockingSense(para.debug.para[9], true);
                    SetDockingMotor(ScoutBot::Side(para.debug.para[9]), CLOSE);
                }
                else if(timestamp == 100)
                {
                    irobot->enableDockingSense(para.debug.para[9], true);
                    SetDockingMotor(ScoutBot::Side(para.debug.para[9]), OPEN);
                }
                
                uint8_t rev = ((ScoutBot*)irobot)->GetDScrewRevolutions(ScoutBot::Side(para.debug.para[9]));
                uint8_t ise = ((ScoutBot*)irobot)->GetDScrewISense(ScoutBot::Side(para.debug.para[9]));
                printf("%d rev: %d\tisense:%d\n", timestamp, rev, ise );

            }
            break;

        case 26:
            if(timestamp ==2)
            {
                OrganismSequence tree_a, tree_b;
                tree_a.reBuild("KFKBKFAF0000KBAB00000000KBKF0000KRKF0000");
                tree_b.reBuild("KFAF0000KBAB0000");
                std::vector<uint8_t> IPs;
                std::vector<uint8_t> root_IPs;
                std::vector<uint8_t> IPs2;
                IPs.push_back(11);
                IPs.push_back(12);
                IPs.push_back(11);
                IPs.push_back(13);
                root_IPs.push_back(10);
                root_IPs.push_back(11);
                tree_a.setBranchIPs(FRONT, IPs);
                std::cout<<tree_a<<std::endl;

                tree_a.setBranchIPs(BACK, IPs);
                std::cout<<tree_a<<std::endl;
                
                tree_a.setBranchIPs(RIGHT, IPs);
                std::cout<<tree_a<<std::endl;

                tree_a.setBranchIPs(LEFT, IPs);
                std::cout<<tree_a<<std::endl;

                tree_a.setBranchRootIPs(FRONT, IPs);
                std::cout<<tree_a<<std::endl;
                tree_a.setBranchRootIPs(BACK, IPs);
                std::cout<<tree_a<<std::endl;
                tree_a.setBranchRootIPs(RIGHT, IPs);
                std::cout<<tree_a<<std::endl;
                tree_a.setBranchRootIPs(LEFT, IPs);
                std::cout<<tree_a<<std::endl;
                tree_b.setBranchRootIPs(FRONT, root_IPs);
                std::cout<<tree_b<<std::endl;
                tree_b.setBranchRootIPs(BACK, root_IPs);
                std::cout<<tree_b<<std::endl;
                tree_b.getAllIPs(IPs2);
                std::cout<<tree_b<<std::endl;
                tree_a.setBranchIPs(FRONT, IPs2);
                std::cout<<tree_a<<std::endl;

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

                        for(int i=0;i<para.og_seq_list.size();i++)
                            std::cout<<i<<" : "<<para.og_seq_list[i]<<std::endl;
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
        case 32:
            if(timestamp ==2)
            {
                for(int i=0;i<organism_actions.size();i++)
                {
                    action_sequence &oas = organism_actions[i];
                    printf("action sequence: %d\n", oas.sequence_index);
                    printf("       cmd_type: %d\n", oas.cmd_type);
                    printf("       duration: %d\n", oas.duration);
                    for(int j=0;j< oas.robots_in_action.size();j++)
                        printf("           robots_in_action: %d [%d %d %d]\n", oas.robots_in_action[j].index,oas.robots_in_action[j].cmd_data[0],oas.robots_in_action[j].cmd_data[1],oas.robots_in_action[j].cmd_data[2]);

                }
            }
            break;
        default:
            break;
    }

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

