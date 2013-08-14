#include "robot.hh"

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
    "Recruiting REQ",
    "Seeding REQ", 
    "Docking REQ",
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


void Robot::Process_Daemon_command(const ELolMessage*msg, void* connection, void *user_ptr)
{
    if(!msg || !connection || !user_ptr)
        return;

    Robot *robot = (Robot*)user_ptr;
    IPC::Connection * conn=(IPC::Connection*) connection;
    IPC::IPC * ipc = (IPC::IPC*) conn->ipc;

    bool valid = true;

    printf("%d: Daemon received [%s]\n", robot->timestamp, daemon_message_names[msg->command]);

    switch(msg->command)
    {       
        case DAEMON_MSG_SEEDING_REQ:
            //msg->data
            //[0] -- size of sequence
            //[1] - [x] data;
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
                    for(int i=0;i<NUM_DOCKS;i++)
                    {
                        robot->SetRGBLED(i, 0, 0, 0, 0);
                    }

                    printf("%d: Daemon received seeding request with data (%d) [",robot->timestamp, msg->data[0]);
                    for(int i=0;i<msg->data[0];i++)
                        printf(" %d ", msg->data[i+1]);
                    printf("\n");

                    rt_status ret = robot->mytree.reBuild(msg->data + 1, msg->data[0]);


                    if(ret.status != RT_OK)
                    {
                        printf("%d: Daemon failed to rebuild the shape from the data received\n",robot->timestamp);
                    }
                    else
                    {
                        std::cout<<"target shape is: "<<robot->mytree<<std::endl;
                        robot->request_in_processing = 1;
                        robot->current_state = SEEDING;
                        robot->last_state = DAEMON;
                    }
                }
            }
            break;
        case DAEMON_MSG_RECRUITING_REQ:
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

                    OrganismSequence::Symbol sym;
                    sym.type1 = robot->type;
                    sym.side1 = msg->data[0];
                    sym.type2 = msg->data[1];
                    sym.side2 = msg->data[2];
                    uint8_t tree_data[2];
                    tree_data[0] = sym.data;
                    tree_data[1] = 0;
                    rt_status ret = robot->mytree.reBuild(tree_data, 2);

                    if(ret.status != RT_OK)
                    {
                        printf("%d: Daemon failed to rebuild the shape from the data received: %d %d %d\n",robot->timestamp, msg->data[0], msg->data[1], msg->data[2]);
                        robot->mytree.Clear();
                    }
                    else
                    {
                        std::cout<<"target shape is: "<<robot->mytree<<std::endl;
                        robot->request_in_processing |= 1<<msg->data[0];
                        robot->current_state = SEEDING;
                        robot->last_state = DAEMON;
                    }
                }
            }
            break;
        case DAEMON_MSG_DOCKING_REQ:
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
        case DAEMON_MSG_SEED_IP_REQ:
            {
                ipc->SendData(DAEMON_MSG_SEED_IP, (uint8_t*)&(robot->commander_IP.i32), sizeof(uint32_t)); 
            }
            break;
        case DAEMON_MSG_ALLROBOTS_IP_REQ:
            {
                uint8_t buf[robot->robot_in_organism_index_sorted.size() * sizeof(uint32_t) + 1];
                memset(buf, 0, sizeof(buf));

                uint32_t ip_array[robot->robot_in_organism_index_sorted.size()];
                
                if(robot->seed && robot->organism_formed && robot->last_state == INORGANISM)
                {
                    buf[0] = robot->robot_in_organism_index_sorted.size(); //the size of all ips
                    int index = 0;
                    std::map<int,uint32_t>::iterator it = robot->robot_in_organism_index_sorted.begin();
                    for(it = robot->robot_in_organism_index_sorted.begin(); it != robot->robot_in_organism_index_sorted.end(); it++)
                    {
                        ip_array[index++] = it->second;
                    }
                    memcpy(buf+1, (uint8_t*)ip_array, robot->robot_in_organism_index_sorted.size() * sizeof(uint32_t));

                }

                ipc->SendData(DAEMON_MSG_ALLROBOTS_IP, buf, sizeof(buf));
            }
            break;
        case DAEMON_MSG_PROGRESS_REQ:
            //no following data
            {
                uint8_t ret[2];
                ret[0] = robot->isPaused() ? 1 : 0; //it will automatically return to state Daemon and pause the SPI once dockign is done
                ret[1] = robot->current_state;

                ipc->SendData(DAEMON_MSG_PROGRESS, ret, 2); 
            }
            break;
        case DAEMON_MSG_FORCE_QUIT_REQ:
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


}