#ifndef ROBOT_HH
#define ROBOT_HH

#include <IRobot.h>
#include <comm/IRComm.h>
#include "hist.hh"

#define NUM_IRS 8
#define NUM_DOCKS 4

class Robot{

    public:
        enum role_t {
            ROLE_IDLE=0,
            ROLE_RECRUITING=1,
            ROLE_DOCKING=2
        };
        enum recruiting_status_t {
            RECRUITING_UNKNOWN=0,
            RECRUITING_DONE,
            STAGE1, 
            STAGE2, 
            STAGE3, 
            STAGE4 
        };
        enum docking_stats_t {
            DOCKING_UNKNOWN = 0,
            DOCKING_DONE,
            LOCATEBEACON, 
            ALIGNING, 
            LOCKING 
        };
        union Symbol{
            uint8_t data;
            struct{
                uint8_t type1:2; //bits: 0 - 1 recruiter_type
                uint8_t side1:2; //bits: 2 - 3 recruiter_side
                uint8_t type2:2; //bits: 4 - 5 recruitee_type
                uint8_t side2:2; //bits: 6 - 7 recruitee_side
            };
        };
        enum irmsg_type_t {
            //broadcast, no ack required
            IR_MSG_TYPE_UNKNOWN =0,
            IR_MSG_TYPE_RECRUITING, //followed by assembly_info
            IR_MSG_TYPE_EXPELLING,  //no data 
            IR_MSG_TYPE_GUIDEME, //no data
            IR_MSG_TYPE_DOCKING_SIGNALS_REQ,

            //broadcast, ack required
            IR_MSG_TYPE_LOCKED,
            IR_MSG_TYPE_LOCKME,

            IR_MSG_TYPE_ASSEMBLY_INFO,
            IR_MSG_TYPE_ASSEMBLY_INFO_REQ,

            IR_MSG_TYPE_IP_ADDR,
            IR_MSG_TYPE_IP_ADDR_REQ,

            IR_MSG_TYPE_ACK,              //followed by acknowledged message type
            IR_MSG_TYPE_COUNT
        };

    public:
        static bool Initialise(RobotBase *robot);
        static Robot *Instance() {return instance;};
        bool Recruiting(Symbol s);
        bool Docking(Symbol s);
        uint8_t GetRecruitingStatus(uint8_t side);
        uint8_t GetDockingStatus();
        void Reset();

    protected:
        Robot();
        ~Robot(){};


        void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        void SetRGBLED(int channel, uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br);

        virtual void Recruiting()=0;
        virtual void UpdateSensors()=0;
        virtual void UpdateActuators()=0;

        virtual void LocateBeacon()=0;
        virtual void Aligning()=0;
        virtual void Locking()=0;

        uint8_t recruiting_type[NUM_DOCKS];
        uint8_t recruiting_status[NUM_DOCKS];
        uint8_t docking_type;
        uint8_t docking_status;

        RobotBase * irobot;

        uint32_t guiding_signals_count[NUM_DOCKS];

        uint8_t beacon_signals_detected;
        uint8_t expelling_signals_detected;
        uint8_t robot_in_range_detected;
        uint8_t msg_locked_received;
        uint8_t msg_locked_expected;
        uint8_t msg_lockme_received;
        uint8_t msg_lockme_expected;
        uint8_t msg_guideme_received;
        uint8_t msg_docking_signal_req_received;

        int32_t ambient[NUM_IRS];
        int32_t reflective[NUM_IRS];   //proximity sensors, using 350us pulse
        int32_t proximity[NUM_IRS];   //proximity sensors, using 64Hz signals
        int32_t beacon[NUM_IRS];
        Hist reflective_hist[NUM_IRS];
        Hist ambient_hist[NUM_IRS];
        Hist in_docking_region_hist;
        Hist in_locking_region_hist;
        Hist robots_in_range_detected_hist;
        Hist beacon_signals_detected_hist;
        Hist ambient_avg_threshold_hist;
        Hist proximity_hist[NUM_IRS];
        Hist ethernet_status_hist;

        int8_t speed[3];
        uint32_t timestamp;

    private:
        bool Update();
        void Docking();
        static Robot * instance;
        static void * MainThread(void *ptr);
        static void ProcessIRMessage(Message *msg);

        uint8_t role;
        pthread_t main_thread;
        pthread_mutex_t mutex;
        bool main_thread_running;
};

class RobotSCOUT: public Robot{
    enum {LEFTWHEEL=0, RIGHTWHEEL=1};
    public:
        RobotSCOUT():Robot(){};
        ~RobotSCOUT(){};

        virtual void Recruiting();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

        virtual void LocateBeacon();
        virtual void Aligning();
        virtual void Locking();

};

class RobotKIT: public Robot{
    public:
        RobotKIT():Robot(){};
        ~RobotKIT(){};

        virtual void Recruiting();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

        virtual void LocateBeacon();
        virtual void Aligning();
        virtual void Locking();
};

class RobotAW: public Robot{
    public:
        RobotAW():Robot(){};
        ~RobotAW(){};

        virtual void Recruiting();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

        virtual void LocateBeacon();
        virtual void Aligning();
        virtual void Locking();
};

#endif