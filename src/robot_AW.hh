#ifndef ROBOT_AW_HH
#define ROBOT_AW_HH

#include <robot.hh>
#include <IRobot.h>
#include <comm/IRComm.h>

class RobotAW:public Robot
{
    public:
        RobotAW(ActiveWheel * r);
        virtual ~RobotAW();

        void PrintAuxReflective();
        void PrintAuxAmbient();

    protected:
        virtual void InitHardware();
        virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0); 
        virtual bool SetDockingMotor(int channel, int status);
        virtual bool SetHingeMotor(int status);
        virtual void SetSpeed(int leftspeed, int rightspeed, int sidespeed);
        virtual void Reset();

        virtual void UpdateSensors();
        virtual void UpdateActuators();
        // for self-repair
        virtual void UpdateFailures();

        virtual void Calibrating();
        virtual void Exploring();
        virtual void Resting();
        virtual void Seeding();
        virtual void Foraging();
        virtual void Assembly();
        virtual void Waiting();
        virtual void LocateEnergy();
        virtual void LocateBeacon();
        virtual void Alignment();
        virtual void Recover();
        virtual void Docking();
        virtual void Locking();
        virtual void InOrganism();
        virtual void Disassembly();
        virtual void Undocking();
        virtual void Recruitment();
        virtual void Raising();
        virtual void Lowering();
        virtual void Reshaping();
        virtual void MacroLocomotion();
        virtual void Debugging();

        virtual void Log();

        float hinge_start_pos;
        uint8_t hinge_speed;

    private:
        void Avoidance();
        ActiveWheel * irobot;
        int32_t aux_ambient[8];
        int32_t aux_reflective[8];   //proximity sensors, using 350us pulse
        int32_t aux_proximity[8];   //proximity sensors, using 64Hz signals
        int32_t aux_beacon[8];

        Hist aux_reflective_hist[NUM_IRS];
        Hist aux_ambient_hist[NUM_IRS];

        uint8_t aux_bumped;
        uint8_t org_bumped;
        bool free_move;
};

#endif
