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
    "MacroLocomotion",
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
    "Propagated",
    "Disassembling",
    "NewRobot_Joined",
    "Organism_Formed",
    "ACK"
};




