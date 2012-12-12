#include "robot.hh"

Robot* Robot::instance = NULL;

Robot::Robot()
{
    role = ROLE_IDLE;
    pthread_mutex_init(&mutex, NULL);
}
bool Robot::Initialise(RobotBase *robot)
{
    if(!robot)
        return false;

    if(robot->GetRobotType() == RobotBase::UNKNOWN)
        return false;
    else if(robot->GetRobotType() == RobotBase::KABOT)
        instance = new RobotKIT();
    else if(robot->GetRobotType() == RobotBase::ACTIVEWHEEL)
        instance = new RobotAW();
    else if(robot->GetRobotType() == RobotBase::SCOUTBOT)
        instance = new RobotSCOUT();

    instance->irobot = robot;

    instance->Reset();

    SPIVerbose = QUIET;

    return true;
}

void Robot::Reset()
{
    printf("Reset\n");
    pthread_mutex_lock(&mutex);
    role = ROLE_IDLE;
    for(int i=0;i<NUM_DOCKS;i++)
    {
        recruiting_status[i] = RECRUITING_UNKNOWN;
        recruiting_type[i] = 0;
    }
    docking_status = DOCKING_UNKNOWN;
    docking_type = 0;
    main_thread_running = false;
    pthread_mutex_unlock(&mutex);
}

bool Robot::Recruiting(Symbol s)
{
    if(role == ROLE_DOCKING)
    {
        printf("Robot is in Docking mode, please wait!\n");
        return false;
    }

    //create new thread if not present
    if(!main_thread_running)
    {
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        int ret = pthread_create(&main_thread, &attributes, MainThread, instance);

        if(ret!=0)
        {
            printf("Can not create a thread for recruting control\n");
            return false;
        }
    }

    pthread_mutex_lock(&mutex);
    role = ROLE_RECRUITING;
    if(recruiting_status[s.side1] == RECRUITING_UNKNOWN)
    {
        recruiting_type[s.side1] = s.data;
        recruiting_status[s.side1] = STAGE1;
    }
    else
    {
        printf("This side is already docked or in recruiting\n");
    }
    pthread_mutex_unlock(&mutex);
    printf("\tSet Role to be Recruiter: %#x\n", role);
    return true;
}



bool Robot::Docking(Symbol s)
{
    if(role != ROLE_IDLE) 
    {
        printf("Robot is in %s mode, please wait!\n", role == ROLE_RECRUITING?"Recruiting":"Docking");
        return false;
    }

    if(!main_thread_running)
    {
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        int ret = pthread_create(&main_thread, &attributes, MainThread, instance);

        if(ret!=0)
        {
            printf("Can not create a thread for recruting control\n");
            return false;
        }
    }

    pthread_mutex_lock(&mutex);
    role = ROLE_DOCKING;
    if(docking_status == DOCKING_UNKNOWN)
    {
        docking_status = LOCATEBEACON; 
        docking_type = s.data;
    }
    else
    {
        printf("This robot is already docked\n");
    }
    pthread_mutex_unlock(&mutex);
    printf("\tSet Role to be Recruitee: %#x\n", role);

    return true;
}

uint8_t Robot::GetRecruitingStatus(uint8_t side)
{
    uint8_t ret;
    pthread_mutex_lock(&mutex);
    ret = recruiting_status[side];
    pthread_mutex_unlock(&mutex);
    return ret;
}

uint8_t Robot::GetDockingStatus()
{
    uint8_t ret;
    pthread_mutex_lock(&mutex);
    ret = docking_status;
    pthread_mutex_unlock(&mutex);
    return ret;
}

void Robot::Docking()
{
    switch(docking_status)
    {
        case LOCATEBEACON:
            LocateBeacon();
            break;
        case ALIGNING:
            Aligning();
            break;
        case LOCKING:
            Locking();
            break;
        default:
            break;
    }
}


bool Robot::Update()
{
    UpdateSensors();

    bool ret = true;
    if(role == ROLE_RECRUITING)
    {
        Recruiting();
        //check if all recruiting side done
        for(int i=0;i<NUM_DOCKS;i++)
        {
            if(recruiting_status[i] > RECRUITING_DONE)
            {
                ret = false;
                break;
            }
        }

    }
    else if(role == ROLE_DOCKING)
    {
        Docking();
        if(docking_status != DOCKING_DONE)
            ret = false;
    }
    else
        printf("Unknown role %#x\n", role);

    UpdateActuators();

    return ret;
}

void* Robot::MainThread(void *ptr)
{
    printf("Start Main Thread\n");
    Robot *robot = (Robot*)ptr;
    robot->main_thread_running = true;
    bool done = false;

    IRComm::Initialize();
    IRComm::SetMessageCallback(ProcessIRMessage);

    while(!done)
    {
        pthread_mutex_lock(&robot->mutex);
        done = robot->Update();
        pthread_mutex_unlock(&robot->mutex);

        //sleep for 100ms
        usleep(100000);
    }

    printf("Exit Main Thread\n");
    robot->role = ROLE_IDLE;
    robot->main_thread_running = false;
    return NULL;
}


void Robot::ProcessIRMessage(Message *msg)
{
    printf("recieved message\n");

    //remove that messae from memo
    IRComm::ReadMessage();
}
