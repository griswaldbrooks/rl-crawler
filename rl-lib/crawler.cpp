// Library for interfacing with the crawler robot using Arduino libraries



// Include Crawler Library
#include <crawler.h>

#define J1PIN 		6
#define J2PIN 		5
#define WAIT_TIME	15	// Time in ms for servo delay

Arm::Arm(){
	joint1.attach(J1PIN);
	joint1.attach(J1PIN);
}

void Arm::setJointAngle(int joint, int angle){
	// Set the joints
	switch(joint){
		case 1: joint1.write(angle);
	}
}
int Arm::getJointAngle(int joint);