#ifndef ROBOT_HH
#define ROBOT_HH
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <locale>
#include <stdexcept>
#include <sys/stat.h>

//#include <IRobot.h>
#include <pthread.h>

#include <comm/IRComm.h>
#include <comm/Ethernet.h>
#include "parameter.hh"
#include "global.hh"
#include "og/organism.hh"
#include "og/organism_sample.hh"
#include "utils/worldfile.hh"
#include "utils/hist.hh"
#include "utils/support.hh"
#include "IRMessage.hh"


class Robot
{
    typedef void(*robot_callback_t)(Robot * robot);

    public:
    Robot();
    virtual ~Robot();
    void Update(const uint32_t& ts);
    void Stop();

    bool Init(const char *optionfile);
    bool InitLog();
    bool LoadParameters(const char * filename);

    void SetOgSequence(const OrganismSequence & og_seq);
    const OrganismSequence& GetOgSequence();

    void PrintProximity();
    void PrintBeacon();
    void PrintReflective();
    void PrintAmbient();
    void PrintRGB();
    void PrintStatus();
    void PrintSubOGString( uint8_t* );
    void LogState();

    uint32_t CheckIRLEDStatus(int channel, int led);
    void CheckDockingMotor();
    void CheckHingeMotor();
    
    void SendEthMessage(int channel, uint8_t type, const uint8_t *data, int size=1, bool ack_required = false);
    void PropagateEthMessage(uint8_t type, uint8_t *data = NULL, uint32_t size = 0, Ethernet::IP = 0);

    //send to specified receiver (defined in docked[i]), ack may be required
    void SendIRMessage(int channel, uint8_t type, bool ack_required = true);
    void SendIRMessage(int channel, uint8_t type, const uint8_t data, bool ack_required = true);
    void SendIRMessage(int channel, uint8_t type, const uint8_t *data, int size=1, bool ack_required = true);
    void SendIRAckMessage(int channel, uint8_t type);
    void SendIRAckMessage(int channel, uint8_t type, uint8_t *data, int size=1);
    void PropagateIRMessage(uint8_t type, uint8_t *data = NULL, uint32_t size = 0, int excluded_channel = -1);
    void PropagateSingleIRMessage(uint8_t type, int channel, uint8_t *data = NULL, uint32_t size = 0 );

    //no specified reciever, ack is not required
    void BroadcastIRMessage(int channel, uint8_t type, bool ack_required = false);
    void BroadcastIRMessage(int channel, uint8_t type, const uint8_t data, bool ack_required = false);
    void BroadcastIRMessage(int channel, uint8_t type, const uint8_t *data, int size=1, bool ack_required = false);

    bool MessageWaitingAck(int channel, uint8_t type);
    bool MessageWaitingAck(uint8_t type);

    // reset variables required for self-assembly
    void ResetAssembly();

    protected:    
    virtual void InitHardware()=0;
    virtual void UpdateSensors()=0;
    virtual void UpdateActuators()=0;
    virtual void Reset()=0;
    // for self-repair
    virtual void UpdateFailures()=0;

    virtual void Calibrating();
    virtual void Exploring()=0;
    virtual void Resting()=0;
    virtual void Seeding()=0;
    virtual void Foraging()=0;
    virtual void Assembly()=0;
    virtual void Waiting()=0;
    virtual void LocateEnergy()=0;
    virtual void LocateBeacon()=0;
    virtual void Alignment()=0;
    virtual void Docking()=0;
    virtual void Locking()=0;
    virtual void Recover()=0;
    virtual void Undocking()=0;
    virtual void InOrganism()=0;
    virtual void Disassembly()=0;
    virtual void Recruitment()=0;
    virtual void Raising()=0;
    virtual void Lowering()=0;
    virtual void Reshaping()=0;
    virtual void MacroLocomotion()=0;
    virtual void Debugging()=0;
    virtual void Log()=0;

    // for self-repair
    bool StartRepair();
    void Failed();
    void Support();
    void LeadRepair();
    void Repair();

    std::string ClockString();

    //  void BroadcastMessage(Message); //broadcast message via wired communication bus
    //  void SendMessage(int i, Message*);

    // for self-repair
    void SendFailureMsg( int i );
    void SendSubOrgStr( int i, uint8_t *seq_str );
    void SendScoreStr( int i, const OrganismSequence &seq, uint8_t score );
    uint8_t calculateSubOrgScore( OrganismSequence &seq1, OrganismSequence &seq2 );
    void PropagateSubOrgScore( uint8_t score, uint8_t id, int ignore_side );
    void PropagateReshapeScore( uint8_t score, int ignore_side );
    void BroadcastScore( int i, uint8_t score, uint8_t id );


    void SendBranchTree(int i, const OrganismSequence& seq);
    bool isNeighboured(Robot *);

    virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led)=0;
    virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0)=0; 
    virtual void SetSpeed(int8_t leftspeed, int8_t rightspeed, int8_t sidespeed)=0;

    virtual bool SetDockingMotor(int channel, int status)=0;
    virtual bool SetHingeMotor(int status)=0;

    virtual int in_docking_region(int x[4])=0;
    virtual int in_locking_region(int x[4])=0;

    private:
    static void Calibrating(Robot *robot){robot->Calibrating();}
    static void Exploring(Robot * robot) {robot->Exploring();}
    static void Resting(Robot * robot) {robot->Resting();}
    static void Seeding(Robot * robot) {robot->Seeding();}
    static void Foraging(Robot * robot){robot->Foraging();}
    static void Assembly(Robot * robot){robot->Assembly();}
    static void Waiting(Robot * robot){robot->Waiting();}
    static void LocateEnergy(Robot * robot){robot->LocateEnergy();}
    static void LocateBeacon(Robot * robot){robot->LocateBeacon();}
    static void Alignment(Robot * robot){robot->Alignment();}
    static void Recover(Robot * robot){robot->Recover();}
    static void Docking(Robot * robot){robot->Docking();}
    static void Locking(Robot * robot){robot->Locking();}
    static void InOrganism(Robot * robot){robot->InOrganism();}
    static void Disassembly(Robot * robot){robot->Disassembly();}
    static void Undocking(Robot * robot){robot->Undocking();}
    static void Recruitment(Robot * robot){robot->Recruitment();}
    static void Raising(Robot * robot){robot->Raising();}
    static void Lowering(Robot * robot){robot->Lowering();}
    static void Reshaping(Robot * robot){robot->Reshaping();}
    static void MacroLocomotion(Robot * robot){robot->MacroLocomotion();}
    static void Debugging(Robot * robot){robot->Debugging();}
    // for self-repair
    static void Failed(Robot * robot){robot->Failed();}
    static void Support(Robot * robot){robot->Support();}
    static void LeadRepair(Robot * robot){robot->LeadRepair();}
    static void Repair(Robot * robot){robot->Repair();}

    void RegisterBehaviour(robot_callback_t fnp, fsm_state_t state);
    robot_callback_t behaviours[STATE_COUNT];

    static void *IRCommTxThread(void* robot);
    static void *IRCommRxThread(void* robot);
    static void *EthCommTxThread(void* robot);
    static void *EthCommRxThread(void* robot);

    void SendIRMessage(const IRMessage& msg);
    void SendEthMessage(const EthMessage& msg);
    void ProcessIRMessage(std::auto_ptr<Message>);
    void ProcessEthMessage(std::auto_ptr<Message>);

    protected:

    uint32_t timestamp;
    uint32_t timestamp_propagated_msg_received;
    uint32_t id;
    robot_type type;
    char *name;

    // for self-repair
    uint8_t subog_str[MAX_IR_MESSAGE_SIZE];
    uint8_t subog_id;
    uint8_t best_score;
    uint8_t best_id;
    uint8_t own_score;
    uint8_t new_id[NUM_DOCKS];
    uint8_t new_score[NUM_DOCKS];

    fsm_state_t current_state;
    fsm_state_t last_state;
    robot_mode_t current_mode;

    //raw sensor data
    int32_t ambient[NUM_IRS];
    //int32_t ambient_calibrated[NUM_IRS];
    int32_t reflective[NUM_IRS];   //proximity sensors, using 350us pulse
    //int32_t reflective_calibrated[NUM_IRS];
    int32_t proximity[NUM_IRS];   //proximity sensors, using 64Hz signals
    int32_t beacon[NUM_IRS];
    rgb_t   color[NUM_DOCKS];
    float comm_status[NUM_IRCOMMS];

    //windowed sensor array
    Hist reflective_hist[NUM_IRS];
    Hist ambient_hist[NUM_IRS];
    Hist in_docking_region_hist;
    Hist in_locking_region_hist;
    Hist robots_in_range_detected_hist;
    Hist beacon_signals_detected_hist;
    Hist ambient_avg_threshold_hist;
    Hist proximity_hist[NUM_IRS];

    //status
    uint8_t bumped;
    uint8_t RGBLED_flashing;
    uint8_t docked[NUM_DOCKS];
    //bits 0-1: mytype
    //bits 2-3: myside
    //bits 4-5: neighbour's type
    //bits 6-7: neighbour's side
    bool docking_done[NUM_DOCKS];   
    bool docking_failed;
    bool unlocking_required[NUM_DOCKS];
    uint8_t recruitment_stage[NUM_DOCKS];//using an array in case parallel docking is enabled
    uint32_t IRLED_status[NUM_DOCKS]; //each ir led (bits 0 and 1): 0 or 1-- off,
    //2 -- docking beacon, 3 -- proximity; bit 2: 0 --disabled, 1 --enabled
    //led0, bits 0, 1, 2
    //led1, bits 3, 4, 5
    //led2, bits 6, 7, 8
    //IR_Pulse0, bits 9
    //IR_Pulse§, bits 10
    uint32_t RGBLED_status[NUM_DOCKS];
    uint8_t docking_motors_status[NUM_DOCKS]; //each two bits, 0b00 -- open, 0b01 -- opening, 0b10 -- closed, 0b11 -- closing
    uint8_t hinge_motor_status;

    bool organism_found;
    bool powersource_found;
    bool organism_formed;

    // for self-repair
    bool module_failed;
    uint8_t wait_side;
    uint8_t parent_side;
    uint8_t repair_stage;
    uint32_t repair_start;
    uint16_t repair_duration;
    uint32_t move_start;
    uint16_t move_duration;
    uint32_t broadcast_start;
    uint16_t broadcast_duration;
    uint8_t broadcast_period;

    uint32_t recover_count;
    uint32_t recruitment_signal_interval_count[NUM_DOCKS];
    uint32_t recruitment_count[NUM_DOCKS];
    uint32_t docking_count;
    uint32_t inorganism_count;
    uint32_t macrolocomotion_count;
    uint32_t raising_count;
    uint32_t lowering_count;
    uint32_t undocking_count; //step for undocking, i.e. open connectors and wait a few steps
    uint32_t assembly_count;  //how long the robot detected last recruitment signals
    uint32_t waiting_count;
    uint32_t foraging_count;
    uint32_t seeding_count;
    uint32_t foraging_blind_count;
    uint32_t docking_motor_operating_count[NUM_DOCKS];
    uint32_t docking_failed_reverse_count;
    uint32_t hinge_motor_operating_count;
    uint32_t guiding_signals_count[NUM_DOCKS];

    uint8_t docking_trials;

    uint8_t beacon_signals_detected;
    uint8_t expelling_signals_detected;
    uint8_t robot_in_range_detected;
    uint8_t robot_in_range_replied;
    uint8_t msg_reshaping_received;
    uint8_t msg_reshaping_expected;
    uint8_t msg_raising_received;
    uint8_t msg_lowering_received;
    uint8_t msg_disassembly_received;
    uint8_t msg_locked_received;
    uint8_t msg_locked_expected;
    uint8_t msg_unlocked_received;
    uint8_t msg_unlocked_expected;
    uint8_t msg_unlockme_received;
    uint8_t msg_unlockme_expected;
    uint8_t msg_lockme_received;
    uint8_t msg_lockme_expected;
    uint8_t msg_guideme_received;
    uint8_t msg_docking_signal_req_received;
    bool msg_organism_seq_received;
    bool msg_organism_seq_expected;
    // for self-repair
    uint8_t msg_failed_received;
    uint8_t msg_subog_seq_received;
    uint8_t msg_subog_seq_expected;
    uint8_t msg_score_seq_received;
    uint8_t msg_score_seq_expected;
    uint8_t msg_score_received;
    uint8_t msg_ip_addr_received;
    uint8_t msg_ip_addr_expected;



    //all ir message in a queue
    typedef std::vector<IRMessage> IRMessageQueue;
    typedef std::vector<EthMessage> EthMessageQueue;
    IRMessageQueue IR_TXMsgQueue[NUM_DOCKS];
    EthMessageQueue Eth_TXMsgQueue[NUM_DOCKS];

    unsigned char newrobot_attached; //indicating new robot joined the organism, used to propagate the message to the whole organism
    bool assembly_info_checked;

    int32_t leftspeed;
    int32_t rightspeed;
    int32_t sidespeed;
    int32_t direction;

    uint32_t num_robots_inorganism;


    //organism related
    OrganismSequence mytree;
    OrganismSequence subog;
    OrganismSequence target;
    std::vector<OrganismSequence> mybranches;
    Organism * og;
    OrganismSequence::Symbol assembly_info; //information for which types of robot and which side is required by recruiting robots
    bool seed;
    Robot * neighbours[NUM_DOCKS];

    uint8_t docking_approaching_sensor_id[2]; //store the id of ir sensors on the approaching side

    Morph::Worldfile * optionfile;
    Parameter para;
    std::ofstream logFile;
    std::ofstream logstateFile;


    pthread_mutex_t ir_rx_mutex;
    pthread_mutex_t ir_txqueue_mutex;
    pthread_mutex_t eth_rx_mutex;
    pthread_mutex_t eth_txqueue_mutex;
    pthread_t ircomm_rx_thread;
    pthread_t ircomm_tx_thread;
    pthread_t ethcomm_rx_thread;
    pthread_t ethcomm_tx_thread;

    uint8_t board_dev_num[SIDE_COUNT]; //store the right spi device number for robot_side
    uint8_t robot_side_dev_num[SIDE_COUNT]; //store the corresponding robot_side of spi device

    Ethernet::IP my_IP;
    Ethernet::IP neighbours_IP[SIDE_COUNT];
    

    uint8_t LED0;
    uint8_t LED1;
    uint8_t LED2;
    uint8_t IR_PULSE0;
    uint8_t IR_PULSE1;
    uint8_t IR_PULSE2;
};

#endif
