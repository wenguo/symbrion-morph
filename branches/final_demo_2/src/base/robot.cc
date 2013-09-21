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
    timestamp_hinge_motor_cmd_received =0;
    timestamp_locomotion_motors_cmd_received =0;
    timestamp_user_input_received = 0;
    timestamp_blob_info_updated = 0;
    type = ROBOT_KIT;

    LED0 = 0x1;
    LED1 = 0x2;
    LED2 = 0x4;
 
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
    RegisterBehaviour(&Robot::Raising, RAISING);
    RegisterBehaviour(&Robot::Lowering, LOWERING);
    RegisterBehaviour(&Robot::Reshaping, RESHAPING);
    RegisterBehaviour(&Robot::MacroLocomotion, MACROLOCOMOTION);
    RegisterBehaviour(&Robot::Climbing, CLIMBING);
    RegisterBehaviour(&Robot::Debugging, DEBUGGING);
    RegisterBehaviour(&Robot::Daemon, DAEMON);
    // for self-repair
    RegisterBehaviour(&Robot::Failed, FAILED);
    RegisterBehaviour(&Robot::Support, SUPPORT);
    RegisterBehaviour(&Robot::LeadRepair, LEADREPAIR);
    RegisterBehaviour(&Robot::Repair, REPAIR);

    pthread_mutex_init(&ir_rx_mutex, NULL);
    pthread_mutex_init(&ir_txqueue_mutex, NULL);
    pthread_mutex_init(&eth_rx_mutex, NULL);
    pthread_mutex_init(&eth_txqueue_mutex, NULL);
    pthread_mutex_init(&IPC_data_mutex, NULL);

    ResetAssembly();

    master_IPC.Name("master");
    commander_IPC.Name("commander");

    demo_count = 0;
    daemon_mode = false;
    request_in_processing = 0;
}

// Reset variables used during assembly
void Robot::ResetAssembly(bool reset_ipc)
{
    printf("%d: Reset Assembly\n", timestamp);
    bumped=0;
    assembly_info_checked=false;
    num_robots_inorganism=1;
    organism_found=false;
    powersource_found=false;
    seed=false;
    organism_formed=false;
    module_failed=false;
    heading = 0;
    wait_side = FRONT;
    parent_side = SIDE_COUNT;
    repair_stage = STAGE0;
    repair_start = 0;
    repair_duration = 50;
    move_start = 0;
    move_duration = 50;
    broadcast_start = 0;
    broadcast_duration = 200;
    broadcast_period = 50;

    hinge_motor_status = LOWED;
    RGBLED_flashing = 0;

    for (int i = 0; i < NUM_IRS; i++)
    {
        reflective[i]=0;
        proximity[i]=0;
        // reflective_calibrated[i]=0;
        // ambient_calibrated[i]=4000;
        // reflective_avg[i]=0;
        beacon[i]=0;
    }

    for (int i = 0; i < NUM_DOCKS; i++)
    {
        comm_status[i] = 0;
        docked[i] = 0;
        docking_done[i] = false;
        unlocking_required[i] = false;
        recruitment_signal_interval_count[i]=DEFAULT_RECRUITMENT_COUNT; //what is this for?
        recruitment_count[i]=0;
        neighbours[i]=NULL;
        IRLED_status[i] = 0;
        RGBLED_status[i] = 1;
        recruitment_stage[i] = STAGE0;
        locking_motor_operating_count[i]=0;
        locking_motors_status[i] = OPENED;
        new_id[i] = SIDE_COUNT;
        new_score[i] = 0;
        guiding_signals_count[i]=0;

        //SetRGBLED(i, 0, 0, 0, 0);
        //SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1);
    }


    hinge_motor_operating_count=0;
    seeding_count=0;
    foraging_blind_count=0;
    waiting_count = 0;
    foraging_count =0;
    docking_count = 0;
    docking_failed_reverse_count = 0;
    recover_count = 0;
    inorganism_count = 0;
    macrolocomotion_count = 0;
    undocking_count = 0;
    raising_count = 0;
    lowering_count = 0;
    locatebeacon_count = 0;
    climbing_count = 0;
    reshaping_count = 0;
    disassembly_count = 0;
    resting_count = 0;

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
    msg_disassembly_received = false;
    msg_reshaping_received=0;
    msg_reshaping_expected=0;
    msg_reshaping_score_received=0;
    msg_raising_received = 0;
    msg_raising_start_received = false;
    msg_raising_stop_received = false;
    msg_lowering_received = false;
    msg_climbing_start_received = false;
    msg_climbing_stop_received = false;
    msg_reshaping_start_received = false;
    msg_reshaping_done_received = false;
    msg_guideme_received = 0;
    msg_docking_signal_req_received = 0;
    msg_organism_seq_received = false;
    msg_organism_seq_expected = false;
    msg_ip_addr_received = 0;
    msg_ip_addr_expected = 0;
    msg_assembly_info_received = 0;
    msg_assembly_info_expected = 0;
    msg_assembly_info_req_received = 0;
    msg_assembly_info_req_expected = 0;

    // for self-repair
    msg_failed_received = 0;
    msg_subog_seq_received = 0;
    msg_subog_seq_expected = 0;
    msg_score_seq_received = 0;
    msg_score_seq_expected = 0;
    msg_score_received = 0;

    msg_retreat_received = false;
    msg_stop_received = false;

    pruning_required = 0;

    docking_failed = false;
    docking_trials=0;
    docking_blocked = false;

    //clear just in case 
    target.Clear();
    mytree.Clear();
    subog.Clear();
    og = NULL;

    // for self-repair
    subog_id = SIDE_COUNT;
    subog_str[0] = 0;
    best_score = 0;
    best_id = SIDE_COUNT;
    own_score = 0;


    robots_in_range_detected_hist.Resize(15);
    ethernet_status_hist.Resize(15);
    locking_motor_isense_hist.Resize(8);

    direction = FORWARD;
    memset(speed, 0, 3 * sizeof(int));

    docking_approaching_sensor_id[0] = 0; 
    docking_approaching_sensor_id[1] = 1; 

    docking_blocked = false;
    docking_region_detected = false;
    aligning_region_detected = false;
    blocking_count = 0;

    assembly_info = 0;

    IP_collection_done = false;


    memset(hinge_command, 0, sizeof(hinge_command));
    memset(locomotion_command, 0, sizeof(locomotion_command));
    memset(og_front_aux_reflective_sensors, 0, sizeof(og_front_aux_reflective_sensors));

    current_action_sequence_index = 0;
    front_aw_ip = 0;
    user_input = 0;

    commander_IPC.Stop();
    master_IPC.Stop();
    commander_acks.clear();
    broken_eth_connections = 0;
    IPC_health = true;

    for (int i = 0; i < NUM_DOCKS; i++)
        neighbours_IP[i] = 0;

    reshaping_waiting_for_undock = 0xF;
    reshaping_unlock_sent = 0;
    reshaping_seed = false;
    reshaping_processed = 0;

    disassembly_waiting_for_undock = 0xF;
}

Robot::~Robot()
{
    printf("Desctruction Robot\n");
    if(name)
        free(name);
}

bool Robot::Init(const char * optionfile)
{

    printf("Robot::Init\n");
    if(!LoadParameters(optionfile))
    {
        return false;
    }

    current_state = fsm_state_t(para.init_state);
    last_state = fsm_state_t(para.init_state);


    // For self-repair - make sure every robot knows the target
    if(!para.og_seq_list.empty()) target = para.og_seq_list[0];

    SPIVerbose = QUIET;

    InitLog();

    InitHardware();

    InitVision();

    my_IP = Ethernet::GetLocalIP();
    commander_IP = my_IP;
    commander_port = COMMANDER_PORT_BASE + COMMANDER_PORT;

    id = (my_IP.i32 >> 24) & 0xFF;

    printf("Create threads\n");
    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    int ret_tx = pthread_create(&ircomm_tx_thread, NULL, IRCommTxThread, this);
    int ret_rx = pthread_create(&ircomm_rx_thread, NULL, IRCommRxThread, this);
    if (ret_tx != 0 || ret_rx!=0)
    {
        printf("Error: Cannot create IRComm Thread.\n");
        return false;
    }

    ret_tx = pthread_create(&ethcomm_tx_thread, NULL, EthCommTxThread, this);
    ret_rx = pthread_create(&ethcomm_rx_thread, NULL, EthCommRxThread, this);
    if (ret_tx != 0 || ret_rx!=0)
    {
        printf("Error: Cannot create EthComm Thread.\n");
        return false;
    }


    //turn off all RGB led
    //turn off all IR pulse
    // Set motor opening thresholds
    for(int i=0;i<NUM_DOCKS;i++)
    {
        SetRGBLED(i, 0, 0, 0, 0);
        SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1|IRPULSE2);

        locking_motor_opening_threshold[i] = para.locking_motor_opening_time;
    }

    robots_in_range_detected_hist.Reset();
    beacon_signals_detected_hist.Reset();

    commander_IPC.SetCallback(Process_Organism_command, this);
    master_IPC.SetCallback(Relay_Organism_command, this);

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
    if(para.logtofile > 1)
    {
        std::ostringstream oss;
        oss << "/flash/morph/log/";
        mkdir(oss.str().c_str(), 0777);
        oss << (char *)name <<"_"<< time_string<< ".log";

        //in case the file exists
        std::ifstream tmpfile(oss.str().c_str());
        if (tmpfile)
            oss << "_";

        logFile.open(oss.str().c_str());

        std::ostringstream oss1;
        oss1 << "/flash/morph/log/";
        oss1 << (char *)name <<"_"<< time_string<< ".state";
        logstateFile.open(oss1.str().c_str());

    }    
    else if(para.logtofile == 1)
    {
        std::ostringstream oss, oss1;
        oss << "/flash/morph/log/";
        mkdir(oss.str().c_str(), 0777);
        //oss << (char *)name <<"_"<< time_string<< ".state";
        oss << (char *)name <<".state";
        //in case the file exists
        std::ifstream tmpfile;//(oss.str().c_str());
        tmpfile.open(oss.str().c_str());
        while (tmpfile.is_open())
        {
            tmpfile.close();
            oss << "_" ;
            tmpfile.open(oss.str().c_str());
        }
        tmpfile.close();
        logstateFile.open(oss.str().c_str());
    }


    return true;
}

void Robot::Stop()
{
    master_IPC.Stop();
    commander_IPC.Stop();
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

    //update simulated failures
    UpdateFailures();

    //check for failures and respond
    CheckForFailures();

    //update status variable
    beacon_signals_detected = 0;
    robot_in_range_detected = 0;
    bumped = 0;
    for(int i=0;i<NUM_IRS;i++)
    {
        if(beacon[i] > para.beacon_threshold[i] && beacon[i] > proximity[i])
            beacon_signals_detected |= 1<<i;

        if(proximity[i]> PROXIMITY_SIGNAL_DETECTED_THRESHOLD  && proximity[i] > beacon[i])
            robot_in_range_detected |=1<<i;

        if(reflective_hist[i].Avg() > para.avoid_threshold[i])
            bumped |= 1<<i;
    }
    beacon_signals_detected_hist.Push2(beacon_signals_detected);
    robots_in_range_detected_hist.Push2(robot_in_range_detected);

    static fsm_state_t temp_state;
    temp_state = current_state;

    //behaviour
    behaviours[current_state](this);

    if(temp_state!=current_state)
    {
        PrintStatus();
    }

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

    // if(para.logtofile)
    //     Log();
    LogState();

    //remove any broken ethernet communication connections
    broken_eth_connections = master_IPC.BrokenConnections();
}

void Robot::Calibrating()
{
    memset(speed, 0, 3 * sizeof(int));

    static int32_t temp1[8]={0,0,0,0,0,0,0,0};
    static int32_t temp2[8]={0,0,0,0,0,0,0,0};
    static int32_t count = 0;
    count++;
    for(int i=0;i<NUM_IRS;i++)
    {
        temp1[i] += reflective[i];
        temp2[i] += ambient[i];
    }

    if (timestamp == 100)
    {
        printf("Calibrating done (%d): ", count);
        for(int i=0;i<NUM_IRS;i++)
        {
            para.reflective_calibrated[i]=temp1[i]/count;
            para.ambient_calibrated[i]=temp2[i]/count;
            printf("%d\t", para.reflective_calibrated[i]);
        }

        printf("\n");
        //    current_state = (fsm_state_t) para.init_state;
        //    last_state = CALIBRATING;

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
                    }
                }
            }
        }

        if(type == ROBOT_KIT)
            optionfile->Save("/flash/morph/kit_option.cfg");
        else if(type == ROBOT_AW)
            optionfile->Save("/flash/morph/aw_option.cfg");
        else if(type == ROBOT_SCOUT)
            optionfile->Save("/flash/morph/scout_option.cfg");


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
        if(locking_motors_status[i] == CLOSING)
        {
            locking_motor_isense_hist.Print2();
            locking_motor_operating_count[i]++ ;   
            if(!para.locking_motor_nonreg[i] && locking_motor_operating_count[i] > 10 && locking_motor_isense_hist.Sum(i) >=4)
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED, "Stop docking motor, as docking motor overloaded-- %d", locking_motor_operating_count[i]);

                // Add small amount to ensure that lock opens fully
                locking_motor_opening_threshold[i] = locking_motor_operating_count[i]+para.locking_motor_opening_offset[i];
            }
            else if(locking_motor_operating_count[i] >= para.locking_motor_closing_time)
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED, "Stop docking motor, after closing -- %d", locking_motor_operating_count[i]);

                // Add small amount to ensure that lock opens fully
                locking_motor_opening_threshold[i] = locking_motor_operating_count[i]+para.locking_motor_opening_offset[i];
            }

        }
        //opeing
        else if(locking_motors_status[i] == OPENING)
        {
            locking_motor_operating_count[i]++ ;   
            locking_motor_isense_hist.Print2();
            if(!para.locking_motor_nonreg[i] && locking_motor_operating_count[i] > 10 && locking_motor_isense_hist.Sum(i) >=4)
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED, "Stop docking motor, as docking motor overloaded-- %d", locking_motor_operating_count[i]);
                locking_motor_opening_threshold[i] = para.locking_motor_opening_time;
            }
            else if(locking_motor_operating_count[i] >= locking_motor_opening_threshold[i] )
            {
                SetDockingMotor(i, STOP);
                CPrintf1(SCR_RED,"Stop docking motor, after opening -- %d", locking_motor_operating_count[i]);
                locking_motor_opening_threshold[i] = para.locking_motor_opening_time;
            }
        }
        else
        {
            locking_motor_operating_count[i] = 0;
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
    for (int i = 0; i <NUM_DOCKS ; i++)
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
        << " speed (" << (int)speed[0] << " , " << (int)speed[1] << ", " << (int)speed[2]<< " )"  << std::endl;
}

void Robot::LogState()
{
    if (logstateFile.is_open())
    {
        logstateFile << timestamp << "\t" << current_state << "\t"<< last_state<<"\t" ;
        logstateFile << "("<<speed[0] <<"\t" << speed[1] <<"\t" << speed[2]<<")\t";
        logstateFile << beacon[docking_approaching_sensor_id[0]] <<"\t" << beacon[docking_approaching_sensor_id[1]]<<"\t";
        logstateFile <<"[";
        //logstateFile << (int)(recruitment_stage[0])<<"\t";
        //logstateFile << (int)(recruitment_stage[1])<<"\t";
        //logstateFile << (int)(recruitment_stage[2])<<"\t";
        //logstateFile << (int)(recruitment_stage[3])<<"]";
        logstateFile << reflective_hist[docking_approaching_sensor_id[0]].Avg()<<"\t";
        logstateFile << reflective_hist[docking_approaching_sensor_id[1]].Avg()<<"\t";
        logstateFile << proximity[docking_approaching_sensor_id[0]]<<"\t";
        logstateFile << proximity[docking_approaching_sensor_id[1]]<<"]";
        logstateFile <<std::endl;
    }
}

void Robot::PrintSubOGString( uint8_t *seq)
{
    printf("%d length of sequence: %d\n",timestamp, (int)seq[0]);

    // Print bitstring
    printf("%d bitstring: ",timestamp);
    for( int i=0; i<(int)seq[0]+1; i++ )
    {
        for( int j=7; j>=0; j-- )
        {
            if( (seq[i] & 1<<j) != 0 )
                printf("1");
            else
                printf("0");
        }
        printf(" ");
    }
    printf("\n");

    printf("%d Sequence: ",timestamp);
    for( int i=1; i<(int)seq[0]+1; i++ )
        std::cout << OrganismSequence::Symbol(seq[i]) << " ";
    std::cout << std::endl;

}

const char * Robot::IPToString(Ethernet::IP ip)
{
    struct in_addr addr = { ip.i32 };
    return inet_ntoa(addr);

}
Ethernet::IP Robot::StringToIP(const char *str)
{
    return Ethernet::ip_t::parse(str);
}
Ethernet::IP Robot::getFullIP(const uint8_t addr)
{
    if(addr == 0)
        return Ethernet::ip_t(0);
    else
        return Ethernet::ip_t(uint32_t(addr)<<24 | 52 << 16 | 168 <<8 | 192);
}

void Robot::UpdateOGIRSensors(uint8_t config[2], int data[8], int sensor_type)
{
    uint32_t robot_ip = getFullIP(config[0]).i32;
    int robot_direction =  robot_pose_in_organism[robot_ip].direction;
    int robot_og_irsensor_index =  robot_pose_in_organism[robot_ip].og_irsensor_index;

    if(sensor_type == IR_AUX_REFLECTIVE_DATA)
    {
        if(robot_direction == 1)
        {
            og_front_aux_reflective_sensors[0] = data[6];
            og_front_aux_reflective_sensors[1] = data[7];
            og_front_aux_reflective_sensors[2] = data[0];
            og_front_aux_reflective_sensors[3] = data[1];
        }
        else
        {
            og_front_aux_reflective_sensors[0] = data[2];
            og_front_aux_reflective_sensors[1] = data[3];
            og_front_aux_reflective_sensors[2] = data[4];
            og_front_aux_reflective_sensors[3] = data[5];
        }

    }
    else
    {
        OG_IRsensor * ir_sensors;
        if(sensor_type == IR_REFLECTIVE_DATA)
            ir_sensors = &og_reflective_sensors;
        else if(sensor_type == IR_AMBIENT_DATA)
            ir_sensors= &og_ambient_sensors;
        else if(sensor_type == IR_PROXIMITY_DATA)
            ir_sensors = &og_proximity_sensors;
        else if(sensor_type == IR_BEACON_DATA)
            ir_sensors = &og_beacon_sensors;
        else
            return;

        //header
        pthread_mutex_lock(&IPC_data_mutex);
        if(robot_pose_in_organism[robot_ip].tail_header == -1)
        {
            //  printf("%d: update sensor data [%d] og from %s, as a header (%d) \n", timestamp, sensor_type, IPToString(getFullIP(config[0])), robot_direction);
            ir_sensors->front[0] = robot_direction == 1 ? data[0] : data[4];//front right 
            ir_sensors->front[1] = robot_direction == 1 ? data[1] : data[5];//front left
        }

        //tail
        if(robot_pose_in_organism[robot_ip].tail_header == 1)
        {
            // printf("%d: update sensor data [%d] og from %s, as a tail (%d)\n", timestamp, sensor_type, IPToString(getFullIP(config[0])), robot_direction);
            ir_sensors->back[0] = robot_direction == 1 ? data[4] : data[0]; ;//back left 
            ir_sensors->back[1] = robot_direction == 1 ? data[5] : data[1] ;//back right 
        }

        //left and right
        if(robot_pose_in_organism[robot_ip].og_irsensor_index != -1)
        {
            // printf("%d: update sensor data [%d] og from %s, push into og_irsensor[%d]\n", timestamp, sensor_type, IPToString(getFullIP(config[0])), robot_og_irsensor_index);
            ir_sensors->left[2 * robot_og_irsensor_index]       = robot_direction == 1? data[2] : data[6];
            ir_sensors->left[2 * robot_og_irsensor_index + 1]   = robot_direction == 1? data[3] : data[7];
            ir_sensors->right[2 * robot_og_irsensor_index]      = robot_direction == 1? data[7] : data[3];
            ir_sensors->right[2 * robot_og_irsensor_index + 1]  = robot_direction == 1? data[6] : data[2];
        }

        pthread_mutex_unlock(&IPC_data_mutex);
    }

}

void Robot::RequestOGIRSensors(uint8_t sensor_type)
{
    std::map<uint32_t, robot_pose>::iterator it;
    for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
    {
        if(it->second.og_irsensor_index >=0 || it->second.tail_header !=0)
            RequestOGIRSensors(it->first, sensor_type);
    }

}

void Robot::RequestOGIRSensors(uint32_t addr, uint8_t sensor_type)
{

    IPCSendMessage(addr, IPC_MSG_IRSENSOR_DATA_REQ, (uint8_t*)&sensor_type, 1);
}


//only works for the snake like organism, connected using only front and back sides
void Robot::InitRobotPoseInOrganism()
{
    //clear the existing data
    robot_pose_in_organism.clear();
    robot_in_organism_index_sorted.clear();
    
    //init the client list and the acks
    int pos_index = 0;
    int dir = 1;
    bool new_branch = true;
    robot_pose pose;
    pthread_mutex_lock(&IPC_data_mutex);
    int index_min = 0;
    int index_max = 0;

    //insert myself
    pose.direction = 1;
    pose.index = 0;
    pose.type = type;
    robot_pose_in_organism[my_IP.i32] = pose;

    //insert all other robots
    for(unsigned int i=0;i<mytree.Encoded_Seq().size();i++)
    {
        if(mytree.Encoded_Seq()[i] != OrganismSequence::Symbol(0))
        {
            if(new_branch)
            {
                new_branch = false;
                if(mytree.Encoded_Seq()[i].side1 == BACK)
                    dir = 1;
                else
                    dir = -1;
            }

            pos_index += dir;
            pose.index = pos_index;
            pose.direction = (mytree.Encoded_Seq()[i].side2 == FRONT ? 1 : -1) * dir;
            pose.type = mytree.Encoded_Seq()[i].type2;
            robot_pose_in_organism[getFullIP(mytree.Encoded_Seq()[i].child_IP).i32] = pose;
          //  printf("%d : %s direction %d\n", mytree.Encoded_Seq()[i].child_IP, IPToString(getFullIP(mytree.Encoded_Seq()[i].child_IP).i32), pose.direction);
        }
        else
        {
            pos_index = 0;
            new_branch = true;
       //     printf("new branches\n");
        }

        if(index_min < pos_index)
            index_min = pos_index;

        if(index_max > pos_index)
            index_max = pos_index;
    }

   // printf("min: %d\nmax: %d\n", index_min, index_max);


    std::map<uint32_t, robot_pose>::iterator it;
    for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
    {
        robot_in_organism_index_sorted[it->second.index] = it->first;
    }

    //fill in og_irsensor_index
    int og_irsensor_index = 0;
    std::map<int, uint32_t>::iterator it1=robot_in_organism_index_sorted.begin();
    //set tail_header
    robot_pose_in_organism[it1->second].tail_header = -1;
    for(it1 = robot_in_organism_index_sorted.begin(); it1 != robot_in_organism_index_sorted.end(); it1++)
    {
        robot_pose &p = robot_pose_in_organism[it1->second];
  //      printf("inside: %s's index: %d pose: %d type: %c\n", IPToString(it1->second), p.index,p.direction, robottype_names[p.type]);
        //set og_irsensor_index if it is AW
        if(robot_pose_in_organism[it1->second].type == ROBOT_AW)
            robot_pose_in_organism[it1->second].og_irsensor_index = og_irsensor_index++;
    }
    //set tail_header
    it1--;
    robot_pose_in_organism[it1->second].tail_header = 1;


    //init the OGIRsensor, to make them the right size for easy access later
    og_reflective_sensors.left.clear();
    og_reflective_sensors.right.clear();
    og_proximity_sensors.left.clear();
    og_proximity_sensors.right.clear();
    og_beacon_sensors.left.clear();
    og_beacon_sensors.right.clear();
    og_ambient_sensors.left.clear();
    og_ambient_sensors.right.clear();
    for(it = robot_pose_in_organism.begin(); it != robot_pose_in_organism.end(); it++)
    {
        if(it->second.type == ROBOT_AW)
        {
            //increase the size of left and right for 2
            og_reflective_sensors.left.push_back(0);
            og_reflective_sensors.left.push_back(0);
            og_reflective_sensors.right.push_back(0);
            og_reflective_sensors.right.push_back(0);
            og_proximity_sensors.left.push_back(0);
            og_proximity_sensors.left.push_back(0);
            og_proximity_sensors.right.push_back(0);
            og_proximity_sensors.right.push_back(0);
            og_beacon_sensors.left.push_back(0);
            og_beacon_sensors.left.push_back(0);
            og_beacon_sensors.right.push_back(0);
            og_beacon_sensors.right.push_back(0);
            og_ambient_sensors.left.push_back(0);
            og_ambient_sensors.left.push_back(0);
            og_ambient_sensors.right.push_back(0);
            og_ambient_sensors.right.push_back(0);

        }
    }

    pthread_mutex_unlock(&IPC_data_mutex);
}

void Robot::PrintOGIRSensor(uint8_t sensor_type)
{
    if(sensor_type ==IR_AUX_REFLECTIVE_DATA)
    {
        printf("IR aux reflective: %d %d %d %d\n", og_front_aux_reflective_sensors[0],og_front_aux_reflective_sensors[1],og_front_aux_reflective_sensors[2],og_front_aux_reflective_sensors[3]);
        return;
    }

    OG_IRsensor * ir_sensors;
    char str[10];
    if(sensor_type == IR_REFLECTIVE_DATA)
    {
        ir_sensors = &og_reflective_sensors;
        sprintf(str, "Reflective");
    }
    else if(sensor_type == IR_AMBIENT_DATA)
    {
        ir_sensors= &og_ambient_sensors;
        sprintf(str, "Ambient");
    }
    else if(sensor_type == IR_PROXIMITY_DATA)
    {
        ir_sensors = &og_proximity_sensors;
        sprintf(str, "Proximity");
    }
    else if(sensor_type == IR_BEACON_DATA)
    {
        ir_sensors = &og_beacon_sensors;
        sprintf(str, "Beacon");
    }
    else
        return;

    printf("IR %s Data\n", str);
    printf("\tFront: %d\t%d\n", ir_sensors->front[0], ir_sensors->front[1]);
    printf("\tLeft: ");
    for(int i=0;i<ir_sensors->left.size();i++)
        printf("%d\t", ir_sensors->left[i]);
    printf("\n");
    printf("\tRight: ");
    for(int i=0;i<ir_sensors->right.size();i++)
        printf("%d\t", ir_sensors->right[i]);
    printf("\n");
    printf("\tBack: %d\t%d\n", ir_sensors->back[0], ir_sensors->back[1]);



}

