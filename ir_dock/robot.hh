#ifndef ROBOT_HH
#define ROBOT_HH

#include <IRobot.h>

class Robot{

    public:
        enum role_t {ROLE_UNKNOWN=0, ROLE_RECRUITING=1, ROLE_DOCKING=2};
        enum recruiting_status_t {RECRUITING_UNKNOWN=0, STAGE1, STAGE2, STAGE3, STAGE4, RECRUITING_DONE};
        enum docking_stats_t {DOCKING_UNKNOWN=0, LOCATEBEACON, ALIGNING, LOCKING, DOCKING_DONE};
        union Symbol{
            uint8_t data;
            struct{
                uint8_t type1:2; //bits: 0 - 1 recruiter_type
                uint8_t side1:2; //bits: 2 - 3 recruiter_side
                uint8_t type2:2; //bits: 4 - 5 recruitee_type
                uint8_t side2:2; //bits: 6 - 7 recruitee_side
            };
        };

    public:
        static bool Initialise(RobotBase *robot);
        static Robot *Instance() {return instance;};
        bool Recruiting(Symbol s);
        bool Docking(Symbol s);
        uint8_t GetRecruitingStatus(uint8_t side);
        uint8_t GetDockingStatus();
        void ResetIRDocking();

    protected:
        Robot();
        ~Robot(){};
        virtual void Recruiting()=0;
        virtual void Docking()=0;
        virtual void UpdateSensors()=0;
        virtual void UpdateActuators()=0;

        pthread_mutex_t mutex;
        uint8_t recruiting_type[4];
        uint8_t recruiting_status[4];
        uint8_t docking_type;
        uint8_t docking_status;

        RobotBase * irobot;
        uint8_t role;
        pthread_t main_thread;
        bool main_thread_running;
    private:
        void Update();
        static Robot * instance;
        static void * MainThread(void *ptr);
};

class RobotSCOUT: public Robot{
    public:
        RobotSCOUT():Robot(){};
        ~RobotSCOUT(){};

        virtual void Recruiting();
        virtual void Docking();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

};

class RobotKIT: public Robot{
    public:
        RobotKIT():Robot(){};
        ~RobotKIT(){};

        virtual void Recruiting();
        virtual void Docking();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

};

class RobotAW: public Robot{
    public:
        RobotAW():Robot(){};
        ~RobotAW(){};

        virtual void Recruiting();
        virtual void Docking();
        virtual void UpdateSensors();
        virtual void UpdateActuators();

};

#endif
