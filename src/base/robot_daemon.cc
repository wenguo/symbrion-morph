#include "robot.hh"

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

const char *daemon_message_names[DAEMON_MSG_COUNT] = {
    "Unknown",
    "Recruiting",
    "Docking",
    "Neighbour's IP REQ",
    "Neighbour's IP",
    "Progress REQ",
    "Progress",
    "Force Quit"
};

void Robot::Daemon()
{
    if(!daemon_mode)
    {
        daemon_mode = true;
        daemon_IPC.SetCallback(Process_Daemon_command, this);
        daemon_IPC.Name("Daemon");
        daemon_IPC.Start("localhost", 40000, true);

    }

    if(!isPaused())
    {
        //pause SPI 
        Pause(true);
    }
}


void Robot::Process_Daemon_command(const LolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    bool valid = true;

    switch(msg->command)
    {       
        case DAEMON_MSG_RECRUITING:
            // msg->data
            // [0] -- recruiting side
            // [1] -- recruited robot type
            // [2] -- recruited robot side
            {
                if((robot->request_in_processing & 1<<msg->data[0]) != 0)
                {
                    uint8_t buf = 0;
                    ipc->SendData(DAEMON_MSG_ACK, &buf, 1);
                }
                else
                {
                    //resume SPI communication
                    robot->Pause(false);
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        robot->SetRGBLED(i, 0, 0, 0, 0);
                    }

                    robot->request_in_processing |= 1<<msg->data[0];
                    OrganismSequence::Symbol sym;
                    sym.type1 = robot->type;
                    sym.side1 = msg->data[0];
                    sym.type2 = msg->data[1];
                    sym.side2 = msg->data[2];
                    uint8_t tree_data[2];
                    tree_data[0] = sym.data;
                    tree_data[1] = 0;
                    robot->mytree.reBuild(tree_data, 2);

                    robot->current_state = SEEDING;
                    robot->last_state = DAEMON;
                }
            }
            break;
        case DAEMON_MSG_DOCKING:
            //no following data
            {
                if(robot->request_in_processing)
                {
                    uint8_t buf = 0;
                    ipc->SendData(DAEMON_MSG_ACK, &buf, 1);
                }
                else
                {
                    //resume SPI communication
                    robot->Pause(false);
                    robot->request_in_processing = 1;

                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        robot->SetIRLED(i, IRLEDOFF, robot->LED0|robot->LED1|robot->LED2, IRPULSE0|IRPULSE1);
                        robot->SetRGBLED(i, 0, 0, 0, 0);
                    }

                    robot->assembly_count = 0;

                    robot->current_state = ASSEMBLY;
                    robot->last_state = DAEMON;
                }
            }
            break;
        case DAEMON_MSG_NEIGHBOUR_IP_REQ:
            //msg->data[0] -- the side
            {
                uint8_t ip_data[5];
                ip_data[0] = msg->data[0];
                memcpy(ip_data + 1, &(robot->neighbours_IP[msg->data[0]].i32), 4);
                ipc->SendData(DAEMON_MSG_NEIGHBOUR_IP, (uint8_t*)&ip_data, sizeof(ip_data)); 
            }
            break;
        case DAEMON_MSG_PROGRESS_REQ:
            //no following data
            {
                uint8_t done = robot->isPaused() ? 1 : 0; //it will automatically return to state Daemon and pause the SPI once dockign is done
                ipc->SendData(DAEMON_MSG_PROGRESS, &done, 1); 
            }
            break;
        case DAEMON_MSG_FORCE_QUIT:
            {
                robot->request_in_processing = 0;
                robot->ResetAssembly();
                robot->last_state = robot->current_state;
                robot->current_state = DAEMON;
            }
            break;
        default:
            valid = false;
            break;
    }

//    printf("%d: Daemon received [%s]\n", robot->timestamp, daemon_message_names[msg->command]);

}
