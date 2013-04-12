#ifndef ROBOT_KIT_HH
#define ROBOT_KIT_HH

#include <robot.hh>
#include <IRobot.h>
#include <comm/IRComm.h>

class RobotKIT:public Robot
{
    public:
        RobotKIT(KaBot * r);
        virtual ~RobotKIT();

    protected:
        virtual void InitHardware();
        virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0); 
        virtual bool SetDockingMotor(int channel, int status);
        virtual bool SetHingeMotor(int status);
        virtual bool MoveHingeMotor(int command[4]);
        virtual void SetSpeed(int leftspeed, int rightspeed, int sidespeed);
        virtual void Reset();

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
        virtual void Raising();
        virtual void Lowering();
        virtual void Reshaping();
        virtual void MacroLocomotion();
        virtual void Climbing();
        virtual void Debugging();

        virtual void Log();
    private:
        void Avoidance();
        KaBot * irobot;
};
#endif
