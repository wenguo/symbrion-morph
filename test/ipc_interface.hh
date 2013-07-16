/*
 *      Created on: 07/2013
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef IPC_INTERFACE_HH
#define IPC_INTERFACE_HH

enum daemon_msg_type_t
{
    DAEMON_MSG_UNKNOWN = 0,
    DAEMON_MSG_RECRUITING_REQ = 0X1,
    DAEMON_MSG_SEEDING_REQ,
    DAEMON_MSG_DOCKING_REQ,
    DAEMON_MSG_NEIGHBOUR_IP_REQ,
    DAEMON_MSG_NEIGHBOUR_IP,
    DAEMON_MSG_SEED_IP_REQ,
    DAEMON_MSG_SEED_IP,
    DAEMON_MSG_ALLROBOTS_IP_REQ,
    DAEMON_MSG_ALLROBOTS_IP,
    DAEMON_MSG_PROGRESS_REQ,
    DAEMON_MSG_PROGRESS,
    DAEMON_MSG_FORCE_QUIT_REQ,

    DAEMON_MSG_ACK,

    DAEMON_MSG_COUNT
};

const char *daemon_message_names[DAEMON_MSG_COUNT] = {
    "Unknown",
    "Recruiting",
    "Seeding", 
    "Docking",
    "Neighbour's IP REQ",
    "Neighbour's IP",
    "Seed's IP REQ",
    "Seed's IP",
    "AllRobot's IP REQ",
    "AllRobot's IP",
    "Progress REQ",
    "Progress",
    "Force Quit"
};


#endif
