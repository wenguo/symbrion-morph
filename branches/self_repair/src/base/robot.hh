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
#include <map>

//#include <IRobot.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <comm/IRComm.h>
#include <comm/Ethernet.h>
#include <comm/switch.h>
#include "parameter.hh"
#include "global.hh"
#include "og/organism.hh"
#include "og/organism_sample.hh"
#include "utils/worldfile.hh"
#include "utils/hist.hh"
#include "utils/ipc.hh"
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
    void PrintOGIRSensor(uint8_t type);
    void LogState();

    uint32_t CheckIRLEDStatus(int channel, int led);
    void CheckDockingMotor();
    void CheckHingeMotor();

    void SendEthMessage(int channel, uint8_t type, const uint8_t *data, int size=1, bool ack_required = false);
    void SendEthAckMessage(int channel, uint8_t type);
    void SendEthAckMessage(int channel, uint8_t type, uint8_t *data, int size=1);
    void PropagateEthMessage(uint8_t type, uint8_t *data = NULL, uint32_t size = 0, Ethernet::IP = 0);

    //send to specified receiver (defined in docked[i]), ack may be required
    void SendIRMessage(int channel, uint8_t type, uint8_t ack_required);
    void SendIRMessage(int channel, uint8_t type, const uint8_t data, uint8_t ack_required);
    void SendIRMessage(int channel, uint8_t type, const uint8_t *data, int size, uint8_t ack_required);
    void SendIRAckMessage(int channel, uint8_t type);
    void SendIRAckMessage(int channel, uint8_t type, uint8_t *data, int size=1);
    void PropagateIRMessage(uint8_t type, uint8_t *data = NULL, uint32_t size = 0, int excluded_channel = -1);
    void PropagateSingleIRMessage(uint8_t type, int channel, uint8_t *data = NULL, uint32_t size = 0 );

    //no specified reciever, ack is not required
    void BroadcastIRMessage(int channel, uint8_t type, uint8_t ack_required);
    void BroadcastIRMessage(int channel, uint8_t type, const uint8_t data, uint8_t ack_required);
    void BroadcastIRMessage(int channel, uint8_t type, const uint8_t *data, int size, uint8_t ack_required);

    bool MessageWaitingAck(int channel, uint8_t type);
    bool MessageWaitingAck(uint8_t type);

    // reset variables required for self-assembly
    void ResetAssembly();

    protected:    
    virtual void InitHardware()=0;
    virtual void UpdateSensors()=0;
    virtual void UpdateActuators()=0;
    virtual void Reset()=0;
    virtual void EnablePowerSharing(int side, bool on)=0;
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
    virtual void InOrganism();
    virtual void Disassembly()=0;
    virtual void Recruitment()=0;
    virtual void Raising();
    virtual void Lowering();
    virtual void Reshaping()=0;
    virtual void MacroLocomotion();
    virtual void Climbing();
    virtual void Debugging()=0;
    virtual void Log()=0;


    virtual int32_t get_aux_reflective(uint8_t i)=0;

    // for self-repair
    void CheckForFailures();
    bool StartRepair();
    void Failed();
    void Support();
    void LeadRepair();
    void Repair();

    uint8_t getNextNeighbour(int);
    uint8_t getNextMobileNeighbour( int last );
    uint8_t getNeighbourHeading( int8_t );
    bool isNeighbourConnected(int i);
    void changeState(fsm_state_t);
    void moveTowardsHeading(int);
    uint8_t recruitmentProgress();
    uint8_t getEthChannel(Ethernet::IP);

    void RemoveFromQueue(int channel, uint8_t type, uint8_t subtype = MSG_TYPE_UNKNOWN);
    void RemoveFromAllQueues(uint8_t type, uint8_t subtype = MSG_TYPE_UNKNOWN);

    std::string ClockString();
    static const char * IPToString(Ethernet::IP ip);
    static Ethernet::IP StringToIP(const char *);
    static Ethernet::IP getFullIP(const uint8_t addr);

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
    virtual void SetSpeed(int leftspeed, int rightspeed, int sidespeed)=0;

    virtual bool SetDockingMotor(int channel, int status)=0;
    virtual bool SetHingeMotor(int status)=0;
    virtual bool MoveHingeMotor(int command[4])=0;
    virtual bool RotateDockingUnit(int channel, int8_t angle)=0;

    void IPCSendMessage(uint32_t dst,  uint8_t type, const uint8_t *data, int size);
    void IPCSendMessage(uint8_t type, const uint8_t *data, int size);
    
    //for organism control
    void RequestOGIRSensors(uint8_t sensor_type);
    void RequestOGIRSensors(uint32_t addr, uint8_t sensor_type);
    void InitRobotPoseInOrganism();
    
    //for remote debugging
    void RemoteDebugging(char *data);
    bool ParseCMD(char *buff);

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
    static void Climbing(Robot * robot){robot->Climbing();}
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

    //for organism control
    static void Relay_Organism_command(const LolMessage*msg, void * connection, void *user_ptr);
    static void Process_Organism_command(const LolMessage*msg, void * connection, void *user_ptr);
    void UpdateOGIRSensors(uint8_t config[2], int data[NUM_IRS], int sensor_type);//config: 0 -- position in og, 1-- orientation related to seed

    protected:

    uint32_t timestamp;
    uint32_t timestamp_propagated_msg_received;
    uint32_t timestamp_hinge_motor_cmd_received; //the last time motor command received from the commander
    uint32_t timestamp_locomotion_motors_cmd_received; //the last time motor command received from the commander
    uint8_t id;
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
    int32_t locking_motor_isense[NUM_DOCKS];

    //windowed sensor array
    Hist reflective_hist[NUM_IRS];
    Hist ambient_hist[NUM_IRS];
    Hist in_docking_region_hist;
    Hist in_locking_region_hist;
    Hist robots_in_range_detected_hist;
    Hist beacon_signals_detected_hist;
    Hist ambient_avg_threshold_hist;
    Hist proximity_hist[NUM_IRS];
    Hist ethernet_status_hist;
    Hist locking_motor_isense_hist;
    Hist docking_blocked_hist;

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
    bool docking_blocked;
    bool docking_region_detected;
    bool aligning_region_detected;
    bool unlocking_required[NUM_DOCKS];
    uint8_t recruitment_stage[NUM_DOCKS];//using an array in case parallel docking is enabled
    uint32_t IRLED_status[NUM_DOCKS]; //each ir led (bits 0 and 1): 0 or 1-- off,
    //2 -- docking beacon, 3 -- proximity; bit 2: 0 --disabled, 1 --enabled
    //led0, bits 0, 1, 2
    //led1, bits 3, 4, 5
    //led2, bits 6, 7, 8
    //IR_Pulse0, bits 9
    //IR_Pulse1, bits 10
    uint32_t RGBLED_status[NUM_DOCKS];
    uint8_t locking_motors_status[NUM_DOCKS]; //each two bits, 0b00 -- open, 0b01 -- opening, 0b10 -- closed, 0b11 -- closing
    uint8_t hinge_motor_status;

    bool organism_found;
    bool powersource_found;
    bool organism_formed;

    // for self-repair
    bool module_failed;
    uint8_t heading;
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
    uint32_t locking_motor_operating_count[NUM_DOCKS];
    uint32_t locking_motor_opening_threshold[NUM_DOCKS];
    uint32_t docking_failed_reverse_count;
    uint32_t hinge_motor_operating_count;
    uint32_t guiding_signals_count[NUM_DOCKS];
    uint32_t blocking_count;
    uint32_t locatebeacon_count;
    uint32_t climbing_count;

    uint8_t docking_trials;

    uint8_t beacon_signals_detected;
    uint8_t expelling_signals_detected;
    uint8_t robot_in_range_detected;
    uint8_t robot_in_range_replied;
    uint8_t msg_reshaping_received;
    uint8_t msg_reshaping_expected;
    uint8_t msg_raising_received;
    bool msg_raising_start_received;
    bool msg_raising_stop_received;
    bool msg_lowering_received;
    bool msg_disassembly_received;
    bool msg_climbing_start_received;
    bool msg_climbing_stop_received;
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
    uint8_t msg_assembly_info_received;
    uint8_t msg_assembly_info_expected;
    uint8_t msg_assembly_info_req_received;
    uint8_t msg_assembly_info_req_expected;
    uint8_t msg_recruiting_req_received;
    // for self-repair
    uint8_t msg_failed_received;
    uint8_t msg_subog_seq_received;
    uint8_t msg_subog_seq_expected;
    uint8_t msg_score_seq_received;
    uint8_t msg_score_seq_expected;
    uint8_t msg_score_received;
    uint8_t msg_ip_addr_received;
    uint8_t msg_ip_addr_expected;
    bool msg_retreat_received;
    bool msg_stop_received;
    uint8_t pruning_required;



    //all ir message in a queue
    typedef std::vector<IRMessage> IRMessageQueue;
    typedef std::vector<EthMessage> EthMessageQueue;
    IRMessageQueue IR_TXMsgQueue[NUM_DOCKS];
    EthMessageQueue Eth_TXMsgQueue[NUM_DOCKS];

    unsigned char newrobot_attached; //indicating new robot joined the organism, used to propagate the message to the whole organism
    bool assembly_info_checked;

    int32_t direction;
    int speed[3];

    uint32_t num_robots_inorganism;


    //organism related
    OrganismSequence mytree;
    OrganismSequence subog;
    OrganismSequence target; //the complete organism
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
    bool IP_collection_done;
    
    uint8_t LED0;
    uint8_t LED1;
    uint8_t LED2;

    //for macrolomotion control
    IPC::IPC  master_IPC;
    Ethernet::IP master_IP;
    int          master_port;
    IPC::IPC  commander_IPC;
    Ethernet::IP commander_IP;
    int          commander_port;
    std::map<uint32_t, int> commander_acks; //store all ips in the organism, except itself, and the acks from them.
    pthread_mutex_t IPC_data_mutex;
    int broken_eth_connections;
    bool IPC_health;

    class robot_pose
    {
        public: 
            robot_pose()
            {
                index = 0;
                direction = 1;
                og_irsensor_index = -1;
                type = ROBOT_NONE;
                tail_header = 0;
            }
            robot_pose & operator= (const robot_pose& rhs)
            {
                index = rhs.index;
                direction = rhs.direction;
                type = rhs.type;
                og_irsensor_index = rhs.og_irsensor_index;
                tail_header = rhs.tail_header;
                return *this;
            }
            int index; //relative postion to the seed, 
            int og_irsensor_index; //index used for og_irsenosr vector, only valid for AW robot, as Scout and KIT will be in the middle
            int direction;
            int type;
            int tail_header; // -1 -- header, 1 -- tail
    };

    class action_sequence
    {
        public: 
            action_sequence()
            {
                sequence_index  = 0;
                counter = 0;
                duration = 0;
                cmd_type = CMD_PUSH_DRAG;
            }

            typedef struct
            {
                int index;
                int cmd_data[3];
            } robot_in_action_t;
            enum cmd_type_t {CMD_PUSH_DRAG = 0, CMD_LIFT_ONE = 1};
            int sequence_index;
            int cmd_type;
            uint32_t duration;
            uint32_t counter; //timer to count how many step have already been spent on this action
            std::vector<robot_in_action_t> robots_in_action;
    };

    std::map<uint32_t, robot_pose> robot_pose_in_organism;
    std::map<int, uint32_t>robot_in_organism_index_sorted; //helper variable for fast access to robot_pose_in_organism

    int hinge_command[4];
    int locomotion_command[4];//direction, speed[0], speed[1], speed[2]

    //sensors 
    typedef struct
    {
        int front[2];
        int back[2];
        std::vector<int> left;
        std::vector<int> right;
    } OG_IRsensor;


    OG_IRsensor og_reflective_sensors;
    OG_IRsensor og_ambient_sensors;
    OG_IRsensor og_proximity_sensors;
    OG_IRsensor og_beacon_sensors;
    int32_t og_front_aux_reflective_sensors[4]; //the 'front' 4 aux sensors located in the first AW robots, front right to left 
    

    std::vector<action_sequence> organism_actions;
    int current_action_sequence_index; //index for the action sequence of organism
    uint32_t front_aw_ip; //ip of the front AW when executing the push-drag behaviour

    int user_input;
    uint32_t timestamp_user_input_received;

};

#endif

