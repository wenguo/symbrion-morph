#include "global.hh"

const char* state_names[STATE_COUNT]={
    "Calibrating",
    "Exploring",
    "Resting",
    "Seeding",
    "Foraging",
    "Assembly",
    "Waiting",
    "LocateEnergy",
    "LocateBeacon",
    "Alignment",
    "Recover",
    "Docking",
    "Locking",
    "Disassembly",
    "Undocking",
    "InOrganism",
    "Recruitment",
    "Raising",
    "Lowering",
    "Reshaping",
    "MacroLocomotion",
    "Failed",
    "Support",
    "LeadRepair",
    "Repair",
    "BroadcastScore",
    "Debugging",
    "Climbing",
};

const char * message_names[MSG_TYPE_COUNT]={
    "UnKnown",
    "Propagated",
    "Disassembling",
    "NewRobot_Joined",
    "Organism_Formed",
    "Raising",
    "Raising Start",
    "Raising Stop",
    "Lowering",
    "Reshaping",
    "Failed",
    "Sub_OG_String",
    "Sub_OG_Score_String",
    "Sub_OG_Score",
    "Retreat",
    "Stop",
    "IP_Addr_Collection",
    "ACK",
    "Remote Debug message",
    "Recruiting",
    "Recruiting_REQ",
    "Expelling",
    "Powersource_Found",
    "GuideMe",
    "DockingSignal_REQ",
    "ObjectType",
    "Locked",
    "UnLocked",
    "LockMe",
    "UnLockMe",
    "Organism_Sequence",
    "Assembly_Info",
    "Assembly_Info_REQ",
    "IP_Addr",
    "IP_Addr_REQ",
    "IPC_Hinge_3D_Motion_REQ",
    "IPC_Locomotion_2D_REQ",
    "IPC_IRSensor_data_REQ",
    "IPC_Raising_start",
    "IPC_Raising_stop",
    "IPC_Lowering_start",
    "IPC_Lowering_stop",
    "IPC_ACK"
};

const char * remote_cmd_names[REMOTE_CMD_COUNT] = {
    "NONE",
    "locking motor cmd",
    "2d locomotion cmd",
    "3d hinge cmd"
};
