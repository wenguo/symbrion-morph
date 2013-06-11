#ifndef ROBOT_AW_HH
#define ROBOT_AW_HH

#include <robot.hh>
#include <IRobot.h>

class robotAW:public Robot
{
    public:
        robotAW(ActiveWheel * r);
        virtual ~robotAW();

        void PrintAuxReflective();
        void PrintAuxAmbient();

    protected:
        virtual void InitHardware();
        virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0); 
        virtual bool SetDockingMotor(int channel, int status);
        virtual bool SetHingeMotor(int status);
        virtual bool MoveHingeMotor(int command[4]);
        virtual bool RotateDockingUnit(int channel, int8_t angle);
        virtual void SetSpeed(int leftspeed, int rightspeed, int sidespeed);
        virtual void Reset();
        virtual void EnablePowerSharing(int side, bool on);

        virtual int32_t get_aux_reflective(uint8_t i){return aux_reflective[i & 0x7];};

        virtual void UpdateSensors();
        virtual void UpdateActuators();
        // for self-repair
        virtual void UpdateFailures();

        virtual void Calibrating();
        virtual void Exploring();
        virtual void Resting();
        virtual void Foraging();
        virtual void Assembly();
        virtual void Waiting();
        virtual void LocateEnergy();
        virtual void LocateBeacon();
        virtual void Alignment();
        virtual void Recover();
        virtual void Docking();
        virtual void Undocking();
        virtual void Locking();
        virtual void Recruitment();
        virtual void Debugging();
        virtual void Avoidance();

        virtual void Log();

        float hinge_start_pos;
        uint8_t hinge_speed;

    private:
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
