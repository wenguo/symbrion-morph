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
    "Debugging"
};

const char * irmessage_names[IR_MSG_TYPE_COUNT]={
    "UnKnown",
    "Recruiting",
    "Expelling",
    "Powersource_Found",
    "GuideMe",
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
    "Propagated",
    "Disassembling",
    "NewRobot_Joined",
    "Organism_Formed",
    "Raising",
    "Lowering",
    "Reshaping",
    "Failed",
    "Sub_OG_String",
    "Sub_OG_Score_String",
    "Sub_OG_Score",
    "ACK"
};




