// 
//
// Author: Wenguo Liu
// Date: 02/2012
//
//

#include "robot.hh"
#include "utils/support.hh"


Robot::Robot()
{
    name = strdup("Robot");
    timestamp = 0;
    timestamp_propagated_msg_received=0;
    type = ROBOT_KIT;

    bumped=0;
    assembly_info_checked=false;
    num_robots_inorganism=1;
    organism_found=false;
    powersource_found=false;
    seed=false;
    organism_formed=false;

    hinge_motor_status = LOWED;
    RGBLED_flashing = 0;

    RegisterBehaviour(&Robot::Calibrating, CALIBRATING);
    RegisterBehaviour(&Robot::Exploring, EXPLORING);
    RegisterBehaviour(&Robot::Resting, RESTING);
    RegisterBehaviour(&Robot::Seeding, SEEDING);
    RegisterBehaviour(&Robot::Foraging, FORAGING);
    RegisterBehaviour(&Robot::Assembly, ASSEMBLY);
    RegisterBehaviour(&Robot::Waiting, WAITING);
    RegisterBehaviour(&Robot::LocateEnergy,LOCATEENERGY);
    RegisterBehaviour(&Robot::LocateBeacon, LOCATEBEACON);
    RegisterBehaviour(&Robot::Alignment, ALIGNMENT);
    RegisterBehaviour(&Robot::Recover, RECOVER);
    RegisterBehaviour(&Robot::Docking, DOCKING);
    RegisterBehaviour(&Robot::Locking, LOCKING);
    RegisterBehaviour(&Robot::InOrganism, INORGANISM);
    RegisterBehaviour(&Robot::Disassembly, DISASSEMBLY);
    RegisterBehaviour(&Robot::Undocking, UNDOCKING);
    RegisterBehaviour(&Robot::Recruitment, RECRUITMENT);
    RegisterBehaviour(&Robot::MacroLocomotion, MACROLOCOMOTION);
    RegisterBehaviour(&Robot::Debugging, DEBUGGING);



    for (int i = 0; i < NUM_IRS; i++)
    {
        reflective[i]=0;
        proximity[i]=0;
        reflective_calibrated[i]=0;
        ambient_calibrated[i]=4000;
        // reflective_avg[i]=0;
        beacon[i]=0;
    }

    for (int i = 0; i < NUM_DOCKS; i++)
    {
        comm_status[i] = 0;
        docked[i] = false;
        docking_done[i] = false;
        unlocking_required[i] = false;
        recruitment_signal_interval_count[i]=0;//DEFAULT_RECRUITMENT_COUNT;
        recruitment_count[i]=0;
        neighbours[i]=NULL;
        IRLED_status[i] = 0;
        RGBLED_status[i] = 1;
        recruitment_stage[i] = STAGE0;
        docking_motor_operating_count[i]=0;
        docking_motors_status[i] = OPENED;
    }


    hinge_motor_operating_count=0;
    seeding_count=0;
    foraging_blind_count=0;
    waiting_count = DEFAULT_WAITING_COUNT;
    foraging_count = DEFAULT_FORAGING_COUNT;
    docking_count = 0;
    recover_count = 0;
    inorganism_count = 0;
    macrolocomotion_count = 0;

    beacon_signals_detected=0;
    robot_in_range_replied=0;
    robot_in_range_detected=0;
    expelling_signals_detected=0;
    
    msg_locked_received = 0;
    msg_locked_expected = 0;
    msg_unlocked_received = 0;
    msg_unlocked_expected = 0;
    msg_unlockme_received = 0;
    msg_unlockme_expected = 0;
    msg_lockme_received = 0;
    msg_lockme_expected = 0;
    msg_disassembly_received = 0;
    msg_reshaping_received=0;
    msg_guideme_received = 0;
    msg_organism_seq_received = false;
    msg_organism_seq_expected = false;

    //clear just in case 
    mytree.Clear();
    og = NULL;

    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&txqueue_mutex, NULL);

    robots_in_range_detected_hist.Resize(16);
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;
    direction = FORWARD;

    LED0 = 0x1;
    LED1 = 0x2;
    LED2 = 0x4;
    IR_PULSE0 = 0x1;
    IR_PULSE1 = 0x2;
    IR_PULSE2 = 0x4;
}

Robot::~Robot()
{
    printf("Desctruction Robot\n");
    if(name)
        free(name);
}

bool Robot::Init(const char * optionfile)
{
    if(!LoadParameters(optionfile))
    {
        return false;
    }

    current_state = CALIBRATING;//fsm_state_t(para.init_state);
    last_state = CALIBRATING;//fsm_state_t(para.init_state);

    InitLog();

    InitHardware();

    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    int ret_tx = pthread_create(&ircomm_tx_thread, NULL, IRCommTxThread, this);
    int ret_rx = pthread_create(&ircomm_rx_thread, NULL, IRCommRxThread, this);
    if (ret_tx != 0 || ret_rx!=0)
    {
        printf("Error: Cannot create IRComm Thread.\n");
        return false;
    }


    //turn off all RGB led
    //turn off all IR pulse
    for(int i=0;i<NUM_DOCKS;i++)
    {
        SetRGBLED(i, 0, 0, 0, 0);
        SetIRLED(i, IRLEDOFF, LED1, IR_PULSE0|IR_PULSE1);
    }

    return true;
}

bool Robot::InitLog()
{
    //format log file name;
    std::string time_string;
    ::time_t time_now = time(NULL);
    struct tm * timeinfo;
    timeinfo = localtime (&time_now);
    time_string = datetime_to_string(*timeinfo, "%Y%m%d%H%M%S");
    if(para.logtofile)
    {
        std::ostringstream oss;
        oss << "./log/";
        mkdir(oss.str().c_str(), 0777);
        oss << (char *)name <<"_"<< time_string<< ".log";
        logFile.open(oss.str().c_str());
    }    

    return true;
}

void Robot::Stop()
{
    SetSpeed(0,0,0);
    Reset();
}

void Robot::RegisterBehaviour(robot_callback_t fnp, fsm_state_t state)
{
    behaviours[state] = fnp;
}

void Robot::Update(const uint32_t& ts)
{
    //update Sensors
    UpdateSensors();

    //update status variable
    beacon_signals_detected = 0;
    for(int i=0;i<NUM_IRS;i++)
    {
        if(beacon[i] > BEACON_SIGNAL_DETECTED_THRESHOLD && beacon[i] > proximity[i])
            beacon_signals_detected |= 1<<i;
        else
            beacon_signals_detected &=~(1<<i);

        if(proximity[i]> PROXIMITY_SIGNAL_DETECTED_THRESHOLD  && proximity[i] > beacon[i])
            robot_in_range_detected |=1<<i;
        else
            robot_in_range_detected &=~(1<<i);

        if(reflective[i] > BUMPED_THRESHOLD ||( proximity[i]>BUMPED_THRESHOLD && proximity[i] > beacon[i]))
            bumped |= 1<<i;
    }
    beacon_signals_detected_hist.Push2(beacon_signals_detected);
    robots_in_range_detected_hist.Push2(robot_in_range_detected);

    static fsm_state_t temp_state;
    temp_state = current_state;

    //behaviour
    behaviours[current_state](this);

    if(temp_state!=current_state)
        PrintStatus();

    PrintBeacon();
    PrintProximity();
    PrintReflective();
    PrintAmbient();
    //PrintStatus();

    //update actuators
    UpdateActuators();

    //flasing rgb LED
    for(int i=0;i<NUM_DOCKS;i++)
    {
        if(RGBLED_flashing & (1<<i))
        {
            SetRGBLED(i, 0, 0, 0, 0);
            RGBLED_flashing &= ~(1<<i);
        }

        if(CheckIRLEDStatus(i, LED1>>1) == (0x4 | IRLEDPROXIMITY) 
                || CheckIRLEDStatus(i, LED0>>1) == (0x4 | IRLEDPROXIMITY))
        {
            {
                if((timestamp / 2) %2==0)
                    SetRGBLED(i, 0, 0, 0, GREEN);
                else
                    SetRGBLED(i, 0,0,GREEN,0);
            }
        }

        if(CheckIRLEDStatus(i, LED1>>1) == (0x4 | IRLEDDOCKING) 
                || CheckIRLEDStatus(i, LED0>>1) == (0x4 | IRLEDDOCKING))
        {
            {
                if((timestamp / 5) %2==0)
                    SetRGBLED(i, 0, 0, BLUE, 0);
                else
                    SetRGBLED(i, 0, 0, 0, BLUE);
            }
        }
    }

    timestamp = ts;

    //if(para.logtofile)
    //    Log();

}

void Robot::Calibrating()
{
    leftspeed = 0;
    rightspeed = 0;
    sidespeed = 0;

    static int32_t temp1[8]={0,0,0,0,0,0,0,0};
    static int32_t temp2[8]={0,0,0,0,0,0,0,0};
    static int32_t count = 0;
    count++;
    for(int i=0;i<NUM_IRS;i++)
    {
        temp1[i] += reflective[i];
        temp2[i] += ambient[i];
    }

    if (timestamp > 30)
    {
        printf("Calibrating done (%d): ", count);
        for(int i=0;i<NUM_IRS;i++)
        {
            reflective_calibrated[i]=temp1[i]/count;
            ambient_calibrated[i]=temp2[i]/count;
            printf("%d\t", reflective_calibrated[i]);
        }
        printf("\n");
        robots_in_range_detected_hist.Reset();
        beacon_signals_detected_hist.Reset();
        current_state = (fsm_state_t) para.init_state;
        last_state = CALIBRATING;
    }
}


uint32_t Robot::CheckIRLEDStatus(int channel, int led)
{
    uint32_t ret = 0;
    for(int i=0;i<3;i++)
    {
        if(led == i)
        {
            ret =(IRLED_status[channel] >> (3 * i) ) & 0x7;
            break;
        }
    }
    return ret;
}

void Robot::CheckHingeMotor()
{
    //lifting
    if(hinge_motor_status == LIFTING)
    {
        if(hinge_motor_operating_count++ >= para.hinge_motor_lifting_time)
        {
            SetHingeMotor(STOP);
            CPrintf1(SCR_RED, "Stop hinge motor, after lifting -- %d\n", hinge_motor_operating_count);
        }

    }
    //lowing
    else if(hinge_motor_status == LOWING)
    {
        if(hinge_motor_operating_count++ >= para.hinge_motor_lowing_time)
        {
            SetHingeMotor(STOP);
            CPrintf1(SCR_RED,"Stop hinge motor, after lowing -- %d\n", hinge_motor_operating_count);
        }
    }
    else
    {
        hinge_motor_operating_count = 0;
    }

}

void Robot::CheckDockingMotor()
{
    //0b00--open, 0b01--opening, 0b10--closed, 0b11--closing
    for(int i=0;i<NUM_DOCKS;i++)
    {
        //closing
        if(docking_motors_status[i] == CLOSING)
        {
            if(docking_motor_operating_count[i]++ >= para.docking_motor_closing_time)
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED, "Stop docking motor, after closing -- %d", docking_motor_operating_count[i]);
            }

        }
        //opeing
        else if(docking_motors_status[i] == OPENING)
        {
            if(docking_motor_operating_count[i]++ >= para.docking_motor_opening_time)
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED,"Stop docking motor, after opening -- %d", docking_motor_operating_count[i]);
            }
        }
        else
        {
            docking_motor_operating_count[i] = 0;
        }
    }
}


void Robot::SetOgSequence(const OrganismSequence& og_seq)
{
    mytree = og_seq;
}

const OrganismSequence& Robot::GetOgSequence()
{
    return mytree;
}

bool Robot::isNeighboured(Robot * r)
{
    for (int i = RIGHT; i <= LEFT; i++)
    {
        if(neighbours[i] && neighbours[i] == r)
            return true;
    }

    return false;
}


/*
   void Robot::SendMessage(int channel, Message * msg)
   {
//NULL point
if(!msg)
return;

//no robot attached on that channel
if(!neighbours[channel])
{
delete msg;
return;
}

for(int i=0; i<SIDE_COUNT; i++)
{
if(neighbours[channel]->neighbours[i] == this)
{
//release the old message if appliable
if(neighbours[channel]->neighbour_message[i])
delete neighbours[channel]->neighbour_message[i];

neighbours[channel]->neighbour_message[i]=msg;
//            std::cout<<timestamp<<" : "<<*msg<<std::endl;
break;
}
}
}
*/


std::string Robot::ClockString()
{
    const uint32_t msec_per_hour   = 36000U; //100msec
    const uint32_t msec_per_minute = 600U;
    const uint32_t msec_per_second = 10U;

    uint32_t hours   = timestamp / msec_per_hour;
    uint32_t minutes = (timestamp % msec_per_hour) / msec_per_minute;
    uint32_t seconds = (timestamp % msec_per_minute) / msec_per_second;
    uint32_t msec    = (timestamp % msec_per_second) ;

    std::string str;
    char buf[256];

    if ( hours > 0 )
    {
        snprintf( buf, 255, "%uh:", hours );
        str += buf;
    }

    snprintf( buf, 255, " %um %02us %03umsec", minutes, seconds, 100* msec);
    str += buf;

    return str;
}



//below defined functions for debugging
void Robot::PrintProximity()
{
    if(!para.print_proximity)
        return;

    printf("%d: proxim\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", proximity[i]);
    printf("\n");
}

void Robot::PrintBeacon()
{
    if(!para.print_beacon)
        return;

    printf("%d: beacon\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", beacon[i]);
    printf("\n");
}
void Robot::PrintReflective()
{
    if(!para.print_reflective)
        return;

    printf("%d: reflec\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", reflective_hist[i].Avg());
    printf("\n");
}

void Robot::PrintAmbient()
{
    if(!para.print_ambient)
        return;

    printf("%d: ambient\t", timestamp);
    for(uint8_t i=0;i<NUM_IRS;i++)
        printf("%d\t", ambient_hist[i].Avg());
    printf("\n");
}

void Robot::PrintRGB()
{
    printf("%d: rgb\n", timestamp);
    for(uint8_t i=0;i<NUM_DOCKS;i++)
    {  
        rgb l = color[i];
        printf("%d : %d\t%d\t%d\t%d\n",i,l.red, l.green, l.blue, l.clear);
    }
}

void Robot::PrintStatus()
{
    if(!para.print_status)
        return;

    std::cout << timestamp << ": " << name << " in state " << state_names[current_state]
        << " [" << state_names[last_state] << "] recover count: " << recover_count
        << " speed (" << leftspeed << " , " << rightspeed << " )"  << std::endl;
}


