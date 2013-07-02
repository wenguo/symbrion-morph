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
    DAEMON_MSG_RECRUITING = 0X1,
    DAEMON_MSG_DOCKING,
    DAEMON_MSG_NEIGHBOUR_IP_REQ,
    DAEMON_MSG_NEIGHBOUR_IP,
    DAEMON_MSG_PROGRESS_REQ,
    DAEMON_MSG_PROGRESS,
    DAEMON_MSG_FORCE_QUIT,

    DAEMON_MSG_ACK,

    DAEMON_MSG_COUNT
};

static char *daemon_message_names[DAEMON_MSG_COUNT] = {
    "Unknown",
    "Recruiting",
    "Docking",
    "Neighbour's IP REQ",
    "Neighbour's IP",
    "Progress REQ",
    "Progress",
    "Force Quit"
};

#endif
