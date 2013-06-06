#ifndef ROBOT_SCOUT_HH
#define ROBOT_SCOUT_HH

#include <robot.hh>
#include <IRobot.h>
#include <comm/IRComm.h>

class RobotSCOUT:public Robot
{
    public:
        RobotSCOUT(ScoutBot * robot);
        virtual ~RobotSCOUT();

    protected:
        virtual void InitHardware();
        virtual void SetIRLED(int channel, IRLEDMode mode, uint8_t led, uint8_t pulse_led);
        virtual void SetRGBLED(int channel, uint8_t tl=LED_BLUE, uint8_t tr=0, uint8_t bl=0, uint8_t br=0); 
        virtual bool SetDockingMotor(int channel, int status);
        virtual bool SetHingeMotor(int status);
        virtual bool MoveHingeMotor(int command[4]);
        virtual bool RotateDockingUnit(int channel, int8_t angle) {};
        virtual void SetSpeed(int leftspeed, int rightspeed, int sidespeed);
        virtual void Reset();
        virtual void EnablePowerSharing(int side, bool on);

        virtual int32_t get_aux_reflective(uint8_t i){return 0;}

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
        virtual void Undocking();
        virtual void Locking();
        virtual void Recruitment();
        virtual void Debugging();
        virtual void Avoidance();

        virtual void Log();
    private:
        ScoutBot * irobot;

        hallSensorScout encoders, last_encoders;
        Hist stalled_hist;

};
#endif
