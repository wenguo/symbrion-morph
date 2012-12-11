#include "robot.hh"

Robot* Robot::instance = NULL;

Robot::Robot()
{
    role = ROLE_UNKNOWN;
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
    instance->main_thread_running = false;

    return true;
}

bool Robot::Recruiting(Symbol s)
{
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
    pthread_mutex_unlock(&mutex);
    printf("\tSet Role to be Recruiter: %#x\n", role);
    return true;
}

bool Robot::Docking(Symbol s)
{
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


void Robot::Update()
{
    UpdateSensors();

    if(role == ROLE_RECRUITING)
        Recruiting();
    else if(role == ROLE_DOCKING)
        Docking();
    else
        printf("Unknown role %#x\n", role);

    UpdateActuators();

}

void* Robot::MainThread(void *ptr)
{
    printf("Start Main Thread\n");
    Robot *robot = (Robot*)ptr;
    while(1)
    {
        pthread_mutex_lock(&robot->mutex);
        robot->Update();
        pthread_mutex_unlock(&robot->mutex);
        usleep(1000000);
    }

    robot->main_thread_running = false;
    return NULL;
}


