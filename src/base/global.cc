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
    "ACK"
};


const char * ethmessage_names[ETH_MSG_TYPE_COUNT]={
    "Unknown",

    "Propagated",
    "Disassembly",
    "NewRobot_Joined",
    "Organism_Formed",
    "Raising",
    "Raising Start",
    "Raising Stop",
    "Lowering",
    "Reshaping",

    // for self-repair
    "Failed",
    "Sub_OG_String",
    "Sub_OG_Score_String",
    "Sub_OG_Score",
    "Retreat",
    "Stop",
    "ACK"
};



