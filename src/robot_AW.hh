#ifndef ROBOT_AW_HH
#define ROBOT_AW_HH

#include <robot.hh>
#include <IRobot.h>
#include <comm/IRComm.h>

class RobotAW:public Robot, public ActiveWheel 
{
    public:
        RobotAW();
        virtual ~RobotAW();

    protected:
        virtual void InitHardware();
        virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0); 
        virtual bool SetDockingMotor(int channel, int status);
        virtual bool SetHingeMotor(int status);
        virtual void SetSpeed(int8_t leftspeed, int8_t rightspeed, int8_t sidespeed);
        virtual void Reset();

        virtual int in_docking_region(int x[4]);
        virtual int in_locking_region(int x[4]);

        virtual void UpdateSensors();
        virtual void UpdateActuators();
        // for self-repair
        virtual void UpdateFailures();

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
        virtual void Transforming();
        virtual void Reshaping();
        virtual void MacroLocomotion();
        virtual void Debugging();

        virtual void Log();
    private:
        void Avoidance();

};

#endif
