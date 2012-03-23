#include "IRobot.h"
#include "comm/IRComm.h"
#include <pthread.h>
#include <signal.h>
#include <cstdio>
#include <string.h>

// ----------------------------------------------------------
// ---- Simple threaded wait for the user to hit any key ----
// ----------------------------------------------------------
// To use that, simply call startStdinRead(), and either trylock(&stdin_read),
// or lock(&stdin_read) to recognize a user input
pthread_t input_thread;
pthread_mutex_t stdin_read;

void* input_thread_fct(void* p) {
	pthread_mutex_lock(&stdin_read);
	getc(stdin);
	pthread_mutex_unlock(&stdin_read);
	return NULL;
}

inline void startStdinRead(void) {
	pthread_create(&input_thread, NULL, input_thread_fct, NULL);
	usleep(10000);
}

// ---------------------------------------------------
// ---- Interrupt (ctrl+c in the console) handler ----
// ---------------------------------------------------
void interrupt_signal_handler(int signal) {
	if (signal == SIGINT) {
		RobotBase::MSPReset();
		exit(0);
	}
}

// -----------------------
// ---- Main function ----
// -----------------------
int main(int argc, char** args) {
	pthread_mutex_init(&stdin_read,NULL);

	RobotBase::RobotType robot_type = RobotBase::Initialize();

	IRComm::Initialize();
	
	// Capture a SIGTERM signal (i.e. MSP reset before quitting)
	struct sigaction a;
	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);
	
	// Enable power for the motors
	switch (robot_type) {
		case RobotBase::ACTIVEWHEEL:
			((ActiveWheel*) RobotBase::Instance())->EnableMotors(true);
			break;
		case RobotBase::KABOT:
			((KaBot*) RobotBase::Instance())->EnableMotors(true);
			break;
		case RobotBase::SCOUTBOT:
			((ScoutBot*) RobotBase::Instance())->EnableMotors(true);
			break;
		default:
			break;
	}

       RobotBase::Instance()->SetIRLED(SPI_D, 0x7);
	
	// Main loop
	while (true) {
		// <Your code>
		// To 
                char str[10];
               

                static int count=0;
                
                int side = count++%4;
                str[1]= side *10 + 1;
                str[2]= side *10 + 2;
                str[3]= side *10 + 3;
                str[4]= side *10 + 4;
                str[5]= side *10 + 5;

                
                //if((count/4)%2==0)
                    str[0]=6;
               // else
               //     str[0]=7;

                    /*
		IRComm::SendMessage(side, str, str[0]);
                printf("%d -- side %d: Broadcast Message", count, count%4);
                for(int i=0;i<str[0];i++)
                    printf("%#x\t", str[i]);
                printf("\n");
*/
                
                if(IRComm::HasMessage())
                {
                    std::auto_ptr<Message>  msg = IRComm::ReadMessage();
                    int size = msg.get()->GetDataLength();
                    char * data = (char *)msg.get()->GetData();
                    printf("%d received message: ", count);
                    for(int i=0;i<size;i++)
                        printf("%c(%#x)\t",data[i],data[i]);
                    printf("\n");
                }
		// Sensor data are sent every 20ms by the MSP.
		// Moreover, because of the motor regulation motor speed should
		// not be updated too often.
		usleep(200000);
	}
	
	return 0;
}
