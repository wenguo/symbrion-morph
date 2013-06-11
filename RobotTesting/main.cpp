/**
 * Author: Benjamin Girault <giraulbn@ipvs.uni-stuttgart.de>
 *
 * To get a colored output, use minicom with the option "-c on"
 */

#include "IRobot.h"
#include <iostream>
#include <cstdio>
#include <string>
#include <pthread.h>

RobotBase::RobotType rotyp;
RobotBase *robot;

const std::string normal = "\033[00m";
const std::string black = "\033[01;30m";
const std::string red = "\033[01;31m";
const std::string green = "\033[01;32m";
const std::string yellow = "\033[01;33m";
const std::string blue = "\033[01;34m";
const std::string purple = "\033[01;35m";
const std::string lightblue = "\033[01;36m";
const std::string bold = "\033[01;37m";

pthread_t input_thread;
pthread_mutex_t cin_read;

void* input_thread_fct(void* p) {
	pthread_mutex_lock(&cin_read);
	std::cin.get();
	pthread_mutex_unlock(&cin_read);
	return NULL;
}

inline void startCinRead(void) {
	pthread_create(&input_thread, NULL, input_thread_fct, NULL);
	usleep(10000);
}

// ------------------------------------------
// ---------------- RGB LEDs ----------------
// ------------------------------------------

void testRGBLEDs(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "RGB LED:" << std::endl;
	std::cout << black;
	std::cout << "Colors should change every 0.5s." << std::endl;
	std::cout << "(Press enter to stop)" << std::endl;
	std::cout << normal;

	startCinRead();
	int cur_color = 0;
	const uint8_t colors[8] = {0, 3, 12, 48, 15, 51, 60, 63};
	do {
		robot->SetLED(s,
					  colors[cur_color],
					  colors[cur_color],
					  colors[cur_color],
					  colors[cur_color]);
		cur_color = (cur_color + 1) % 8;
		usleep(500000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
}

// -------------------------------------
// ---------------- SPI ----------------
// -------------------------------------

bool testSPI(SPIDeviceNum s) {
	return RobotBase::IsBoardRunning(s);
}

// ---------------------------------------------
// ---------------- IR Distance ----------------
// ---------------------------------------------

void testIRDstBase(SPIDeviceNum s, int ir_num) {
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);

	std::cout << yellow;
	std::cout << "Ambient\t\tRefective" << std::endl;;
	std::cout << normal;

	startCinRead();
	do {
		IRValues v = robot->GetIRValues(s);
		std::cout << v.sensor[ir_num].ambient;
		std::cout << "\t\t";
		std::cout << v.sensor[ir_num].reflective;
		std::cout << std::endl;
		usleep(100000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	std::cout << std::endl;
}

void testSingleIRDst(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "Infrared Distance:" << std::endl;
	std::cout << black;
	std::cout << "\"Reflective\" should increase with an approaching obstacle." << std::endl;
	std::cout << "\"Ambient\" should be lesser than 4000." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	std::cout << normal;
	testIRDstBase(s, 0);
}

void testCoupleIRDst(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "Infrared Distance Left:" << std::endl;
	std::cout << black;
	std::cout << "\"Reflective\" should increase with an approaching obstacle." << std::endl;
	std::cout << "\"Ambient\" should be lesser than 4000." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	std::cout << normal;
	testIRDstBase(s, 0);
	std::cout << blue;
	std::cout << "Infrared Distance Right:" << std::endl;
	std::cout << black;
	std::cout << "\"Reflective\" should increase with an approaching obstacle." << std::endl;
	std::cout << "\"Ambient\" should be lesser than 4000." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	std::cout << normal;
	testIRDstBase(s, 1);
}

void testIRDst(SPIDeviceNum s) {
	robot->enableIR(s, true);
	robot->SetIRLED(s, 0);
	robot->SetIRPulse(s, 3);
	robot->SetIRMode(s, IRLEDREFLECTIVE);
	IRValues v = robot->GetIRValues(s);
	if (rotyp == RobotBase::ACTIVEWHEEL && s % 2 == 1)
		testSingleIRDst(s);
	else
		testCoupleIRDst(s);
}

// ----------------------------------------
// ---------------- IR LED ----------------
// ----------------------------------------

void testIRLEDBase(SPIDeviceNum s) {
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);

	std::cout << yellow;
	std::cout << "Proximity Left\tProximity Right" << std::endl;
	std::cout << normal;

	startCinRead();
	do {
		IRValues v = robot->GetIRValues(s);
		std::cout << v.sensor[0].proximity;
		std::cout << "\t\t\t";
		std::cout << v.sensor[1].proximity;
		std::cout << std::endl;
		usleep(100000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	std::cout << std::endl;
}

void testIRLED(SPIDeviceNum s) {
	robot->enableIR(s, true);
	for (int i = 0; i < 3; i++) {
		std::cout << blue;
		switch (i) {
			case 0:
				if (rotyp == RobotBase::ACTIVEWHEEL) {
					std::cout << "Infrared LED Left:" << std::endl;
				} else if (rotyp == RobotBase::KABOT) {
					std::cout << "Infrared LED Left:" << std::endl;
				} else if (rotyp == RobotBase::SCOUTBOT) {
					std::cout << "Infrared LED Left:" << std::endl;
				}
				break;
			case 1:
				if (rotyp == RobotBase::ACTIVEWHEEL) {
					std::cout << "Infrared LED Right:" << std::endl;
				} else if (rotyp == RobotBase::KABOT) {
					std::cout << "Infrared LED Top:" << std::endl;
				} else if (rotyp == RobotBase::SCOUTBOT) {
					std::cout << "Infrared LED Top:" << std::endl;
				}
				break;
			case 2:
				if (rotyp == RobotBase::ACTIVEWHEEL) {
					std::cout << "Infrared LED Top:" << std::endl;
				} else if (rotyp == RobotBase::KABOT) {
					std::cout << "Infrared LED Right:" << std::endl;
				} else if (rotyp == RobotBase::SCOUTBOT) {
					std::cout << "Infrared LED Right:" << std::endl;
				}
				break;
			default:
				return;
		}
		std::cout << black;
		std::cout << "\"Proximity\" of working sensors should increase with an approaching obstacle." << std::endl;
		std::cout << "(Press enter to start, and again to stop)" << std::endl;
		std::cout << normal;
		robot->SetIRLED(s, 1 << i);
		robot->SetIRPulse(s, 3);
		robot->SetIRMode(s, IRLEDPROXIMITY);
		testIRLEDBase(s);
	}
}

// -----------------------------------------------
// ---------------- Ambient Light ----------------
// -----------------------------------------------

void testAmbientLight(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "Ambient Light:" << std::endl;
	std::cout << black;
	std::cout << "The value should decrease when the board is covered." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	robot->enableAmbientLight(s, true);
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);

	std::cout << yellow;
	std::cout << "Ambient Light" << std::endl;
	std::cout << normal;

	startCinRead();
	do {
		std::cout << robot->GetAmbientLight(s) << std::endl;
		usleep(100000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	std::cout << std::endl;
}

// -----------------------------------------------
// ---------------- Accelerometer ----------------
// -----------------------------------------------

void testAccelerometer(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "Accelerometer:" << std::endl;
	std::cout << black;
	std::cout << "The acceleration is in milli g." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	robot->enableAccelerometer(s, true);
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);

	std::cout << yellow;
	std::cout << "a_x\ta_y\ta_z" << std::endl;
	std::cout << normal;

	acceleration_t a;
	startCinRead();
	do {
		a = robot->GetAcceleration(s);
		std::cout << a.x << "\t" << a.y << "\t" << a.z << std::endl;
		usleep(100000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	std::cout << std::endl;
}

// --------------------------------------------
// ---------------- RGB Sensor ----------------
// --------------------------------------------

void testRGBSensor(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "RGB Sensor:" << std::endl;
	std::cout << black;
	std::cout << "The RGB Values should reflect the color of the LEDs." << std::endl;
	std::cout << "Place a white sheet of paper in front of the sensor" << std::endl;
	std::cout << "for better results." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);

	std::cout << yellow;
	std::cout << "red\tgreen\tblue\tambient" << std::endl;
	std::cout << normal;

	rgb_t r;
	int time = 0;
	int cur_color = 0;
	const uint8_t colors[8] = {
		0,
		LED_RED,
		LED_GREEN,
		LED_BLUE,
		LED_RED | LED_GREEN,
		LED_RED | LED_BLUE,
		LED_GREEN | LED_BLUE,
		LED_RED | LED_GREEN | LED_BLUE
	};
	startCinRead();
	do {
		r = robot->GetRGB(s);
		std::cout << r.red << "\t" << r.green << "\t" << r.blue << "\t" << r.clear << std::endl;
		if (time % 20 == 0) {
			time = 0;
			robot->SetLED(s,
						  colors[cur_color],
						  colors[cur_color],
						  colors[cur_color],
						  colors[cur_color]);
			if (colors[cur_color] & LED_RED)
				std::cout << red << "RED" << " ";
			if (colors[cur_color] & LED_GREEN)
				std::cout << green << "GREEN" << " ";
			if (colors[cur_color] & LED_BLUE)
				std::cout << blue << "BLUE" << " ";
			std::cout << normal << std::endl;
			cur_color = (cur_color + 1) % 8;
		}
		time++;
		usleep(100000);
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	std::cout << std::endl;
}

// --------------------------------------------
// ---------------- Microphone ----------------
// --------------------------------------------

void testMicrophone(SPIDeviceNum s) {
	std::cout << blue;
	std::cout << "Microphone" << std::endl;
	std::cout << black;
	std::cout << "Here is a dump of the Microphone data." << std::endl;
	std::cout << "Make a loud enough sound near the board and watch." << std::endl;
	std::cout << "The test function should recognize that sound." << std::endl;
	std::cout << "(Press enter to start, and again to stop)" << std::endl;
	startCinRead();
	pthread_mutex_lock(&cin_read);
	pthread_mutex_unlock(&cin_read);
	
	for (int i = 0; i < 4; i++) {
		robot->enableAccelerometer(i, false);
		robot->enableAmbientLight(i, false);
		robot->enableDockingSense(i, false);
		robot->enableIR(i, false);
	}
	robot->enableMicrophone(s, true, 1);
	usleep(500000);
	std::cout << normal;
	uint16_t buffer[128];
	startCinRead();
	do {
		int length = robot->GetMicrophoneBuffer(s, buffer, 128);
		for (int i = 0; i < length; i++) {
			std::cout << buffer[i] << ' ';
		}
		std::cout << std::endl;
		bool sound = false;
		for (int i = 0; i < length && !sound; i++) {
			// We only test for 0 values, because we don't know for sure
			// on how many bits the values are (8 bits? 12bits?)
			if (buffer[i] == 0) {
				sound = true;
			}
		}
		if (sound) {
			std::cout << "Sound!" << std::endl;
			sleep(1);
		}
	} while (pthread_mutex_trylock(&cin_read));
	pthread_mutex_unlock(&cin_read);
	robot->enableMicrophone(s, false, 1);
	std::cout << std::endl;
}

// --------------------------------------
// ---------------- Main ----------------
// --------------------------------------

int main(int argc, char** argv) {
	pthread_mutex_init(&cin_read,NULL);
	SPIVerbose = QUIET;
	rotyp = RobotBase::Initialize("RobotTesting");
	RobotBase::SetPrintEnabled(0, false);
	RobotBase::SetPrintEnabled(1, false);
	RobotBase::SetPrintEnabled(2, false);
	RobotBase::SetPrintEnabled(3, false);

	robot = RobotBase::Instance();
	
	if (argc < 2 || argv[1][0] == '0') {
		switch (rotyp) {
			case RobotBase::ACTIVEWHEEL:
				// Hinge Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Hinge Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ActiveWheel::RIGHT)) {
					testRGBLEDs(ActiveWheel::RIGHT);
					testRGBSensor(ActiveWheel::RIGHT);
					testIRDst(ActiveWheel::RIGHT);
					testIRLED(ActiveWheel::RIGHT);
					testAmbientLight(ActiveWheel::RIGHT);
					testAccelerometer(ActiveWheel::RIGHT);
					testMicrophone(ActiveWheel::RIGHT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::KABOT:
				// Board A
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Board A:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(KaBot::FRONT)) {
					testRGBLEDs(KaBot::FRONT);
					//testRGBSensor(KaBot::FRONT);
					testIRDst(KaBot::FRONT);
					testIRLED(KaBot::FRONT);
					testAmbientLight(KaBot::FRONT);
					testAccelerometer(KaBot::FRONT);
					testMicrophone(KaBot::FRONT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::SCOUTBOT:
				// Front Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Front Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ScoutBot::FRONT)) {
					testRGBLEDs(ScoutBot::FRONT);
					//testRGBSensor(ScoutBot::FRONT);
					testIRDst(ScoutBot::FRONT);
					testIRLED(ScoutBot::FRONT);
					testAmbientLight(ScoutBot::FRONT);
					testAccelerometer(ScoutBot::FRONT);
					testMicrophone(ScoutBot::FRONT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			default:
				break;
		}
	}

	if (argc < 2 || argv[1][0] == '1') {
		switch (rotyp) {
			case RobotBase::ACTIVEWHEEL:
				// Front Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Front Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ActiveWheel::FRONT)) {
					testRGBLEDs(ActiveWheel::FRONT);
					testIRDst(ActiveWheel::FRONT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::KABOT:
				// Board B
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Board B:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(KaBot::LEFT)) {
					testRGBLEDs(KaBot::LEFT);
					//testRGBSensor(KaBot::LEFT);
					testIRDst(KaBot::LEFT);
					testIRLED(KaBot::LEFT);
					testAmbientLight(KaBot::LEFT);
					testMicrophone(KaBot::LEFT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::SCOUTBOT:
                                				// Front Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Left Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ScoutBot::LEFT)) {
					testRGBLEDs(ScoutBot::LEFT);
					//testRGBSensor(ScoutBot::FRONT);
					testIRDst(ScoutBot::LEFT);
					testIRLED(ScoutBot::LEFT);
					testAmbientLight(ScoutBot::LEFT);
					testAccelerometer(ScoutBot::LEFT);
					testMicrophone(ScoutBot::LEFT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			default:
				break;
		}
	}

	if (argc < 2 || argv[1][0] == '2') {
		switch (rotyp) {
			case RobotBase::ACTIVEWHEEL:
				// Side Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Side Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ActiveWheel::LEFT)) {
					testRGBLEDs(ActiveWheel::LEFT);
					testRGBSensor(ActiveWheel::LEFT);
					testIRDst(ActiveWheel::LEFT);
					testIRLED(ActiveWheel::LEFT);
					testAmbientLight(ActiveWheel::LEFT);
					testMicrophone(ActiveWheel::LEFT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::KABOT:
				// Board C
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Board C:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(KaBot::REAR)) {
					testRGBLEDs(KaBot::REAR);
					//testRGBSensor(KaBot::REAR);
					testIRDst(KaBot::REAR);
					testIRLED(KaBot::REAR);
					testAmbientLight(KaBot::REAR);
					testAccelerometer(KaBot::REAR);
					testMicrophone(KaBot::REAR);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::SCOUTBOT:
				// Arm Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Arm Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;
				
				if (testSPI(ScoutBot::REAR)) {
					testRGBLEDs(ScoutBot::REAR);
					//testRGBSensor(ScoutBot::REAR);
					testIRDst(ScoutBot::REAR);
					testIRLED(ScoutBot::REAR);
					testAmbientLight(ScoutBot::REAR);
					testMicrophone(ScoutBot::REAR);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			default:
				break;
		}
	}

	if (argc < 2 || argv[1][0] == '3') {
		switch (rotyp) {
			case RobotBase::ACTIVEWHEEL:
				// Power Management Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Power Management Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;

				if (testSPI(ActiveWheel::REAR)) {
					testRGBLEDs(ActiveWheel::REAR);
					testIRDst(ActiveWheel::REAR);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::KABOT:
				// Board D
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Board D:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;
				
				if (testSPI(KaBot::RIGHT)) {
					testRGBLEDs(KaBot::RIGHT);
					//testRGBSensor(KaBot::RIGHT);
					testIRDst(KaBot::RIGHT);
					testIRLED(KaBot::RIGHT);
					testAmbientLight(KaBot::RIGHT);
					testMicrophone(KaBot::RIGHT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			case RobotBase::SCOUTBOT:
				// Right Board
				std::cout << red;
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Right Board:" << std::endl;
				std::cout << normal;
				std::cout << std::endl;
				
				if (testSPI(ScoutBot::RIGHT)) {
					testRGBLEDs(ScoutBot::RIGHT);
					//testRGBSensor(ScoutBot::RIGHT);
					testIRDst(ScoutBot::RIGHT);
					testIRLED(ScoutBot::RIGHT);
					testAmbientLight(ScoutBot::RIGHT);
					testMicrophone(ScoutBot::RIGHT);
				} else {
					std::cout << purple;
					std::cout << "NO SPI" << std::endl;
					std::cout << normal;
				}
				break;
			default:
				break;
		}
	}
	
	return 0;
}
