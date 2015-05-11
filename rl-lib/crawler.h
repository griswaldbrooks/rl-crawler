// Library for interfacing with the crawler robot using Arduino libraries

#ifndef CRAWLER_H
#define CRAWLER_H

// Include Arduino Libray
#include <Arduino.h>
#include <Servo.h>

class Arm{
public:
	Arm();
	void setJointAngle(int joint, int angle);
	int getJointAngle(int joint);
private:
	Servo joint1, joint2;
	int angle1, angle2;
};

#endif