// Library for interfacing with the crawler robot using Arduino libraries


// Include Arduino Libray
#include <Arduino.h>
// Include Crawler Library
#include <crawler.h>

// Joint servo Arduino pins for the crawler.
#define J1PIN 		6
#define J2PIN 		5
// Joint calibration parameters.
/* 
* Joint 1
* Desired angle | Commanded angle
* 0 	| 	105
* -90 	|	18
*
* Joint 2
* Desired angle | Commanded angle
* 0 	| 	82
* 90 	|	168
*/
#define CAL_JOINT_M1		(0.967)
#define CAL_JOINT_M2		(0.956)
#define CAL_JOINT_B1		105
#define CAL_JOINT_B2		82

// Lidar analog pin
#define LIDAR_ANALOG_PIN 	A0
// Constant to convert from ADC value to voltage
#define ANALOG_CONVERSION_FACTOR (5.0/1023.0)
// Lidar calibration parameters
/*
* Distance Measured| Voltage In
*  1 m 	| 0.09V
*  2 m 	| 0.18V
*  3 m 	| 0.27V
*  4 m 	| 0.36V
*  5 m 	| 0.45V
*  6 m 	| 0.54V
*  7 m 	| 0.63V
*  8 m 	| 0.72V
*/
#define CAL_LIDAR_M1	(11.11111)
#define CAL_LIDAR_B1	0
// Sampling Time of Lidar in milliseconds
#define LIDAR_DT	30

///*** ArmConfig Class ***///

/*! Constructor for ArmConfig
* \param[in] angle1 	Angle for Joint 1.
* \param[in] angle2 	Angle for Joint 2.
*/
ArmConfig::ArmConfig(float angle1, float angle2){
	angle1 = angle1;
	angle2 = angle2;
}

bool ArmConfig::doComparison(Configuration * c){
	return c->Compare(this);
}

bool ArmConfig::Compare(Configuration * c){
	return false;
}

bool ArmConfig::Compare(ArmConfig * c){
	if(c->angle1 != this->angle1){
		Serial.println("angle1 false.");
		return false;
	}
	else if(c->angle2 != this->angle2){
		Serial.println("angle2 false.");
		return false;
	}
	Serial.println("ArmConfig true.");
	return true;
}

///*** ActionConfig Class ***///

/*! Constructor for ActionConfig
* \param[in] a1 		Action for Joint 1.
* \param[in] a2 		Action for Joint 2.
*/
ActionConfig::ActionConfig(float a1, float a2){
	a1 = a1;
	a2 = a2;
}

// Accepts Configurations to compare with.
bool ActionConfig::doComparison(Configuration * c){
	return c->Compare(this);
}

// Comparision for abstract configuration comparisons.
bool ActionConfig::Compare(Configuration * c){
	return false;
}

// Comparision for action configurations.
bool ActionConfig::Compare(ActionConfig * c){
	if(c->a1 != this->a1){
		return false;
	}
	else if(c->a2 != this->a2){
		return false;
	}
	return true;
}

///*** Arm Class ***///

Arm::Arm():_arm(0,0){
	// Initialize servo angles.
	// _arm.angle1 = _arm.angle2 = 0;
	// Initialize the calibration parameters.
	_m1 = CAL_JOINT_M1;
	_m2 = CAL_JOINT_M2;
	_b1 = CAL_JOINT_B1;
	_b2 = CAL_JOINT_B2;
}

void Arm::setJointAngle(int joint, float angle){
	// Set the joints.
	switch(joint){
		case 1: 
			_arm.angle1 = angle;
			break;
		case 2: 
			_arm.angle2 = angle;
			break;
		default:
			break;
	}
}

// Sets both joint angles for the arm class. 

void Arm::setJointConfig(float angle1, float angle2){
	_arm.angle1 = angle1;
	_arm.angle2 = angle2;
}

float Arm::getJointAngle(int joint){
	// Get the angles.
	switch(joint){
		case 1: 
			return _arm.angle1;
		case 2: 
			return _arm.angle2;
		default:
			return 0;
	}
}

float Arm::getJointAngleCalibrated(int joint){
	// Get the angles.
	float calAngle;
	switch(joint){
		case 1: 
			calAngle = _m1*_arm.angle1 + _b1;
			break;
		case 2: 
			calAngle = _m2*_arm.angle2 + _b2;
			break;
		default:
			calAngle = 0;
			break;
	}

	// Enforce joint limits.
	if(calAngle > 180) calAngle = 180;
	if(calAngle < 0) calAngle = 0;

	return calAngle;
}

int Arm::getJointPin(int joint){
	// Get the pins.
	switch(joint){
		case 1: 
			return J1PIN;
		case 2: 
			return J2PIN;
		default:
			return 0;
	}
}

///*** Lidar Class ***///

/*! \brief Default constructor */
Lidar::Lidar(){
	// Set calibration parameters.
	_m1 = CAL_LIDAR_M1;
	_b1 = CAL_LIDAR_B1;
	// Initialize readings.
	_voltage = _distance = 0;
}

/*! \brief Grabs a distance reading from the Lidar. */
void Lidar::readSensor(){
	_voltage = analogRead(LIDAR_ANALOG_PIN)*ANALOG_CONVERSION_FACTOR;
	_distance = _m1*_voltage + _b1;
}

/*! \brief Grabs N distance readings from the Lidar and saves their average. */
void Lidar::readSensorNTimes(int n){
	float voltageRead, voltageSum = 0;
	int counter;

	for(counter = 1; counter != n; counter++){
		// Grab the data.
		voltageRead = analogRead(LIDAR_ANALOG_PIN)*ANALOG_CONVERSION_FACTOR;
		// Sume the readings.
		voltageSum += voltageRead;
		// Wait until the next reading is ready.
		delay(LIDAR_DT + 10);
	}
	_voltage = voltageSum/n;
	_distance = _m1*_voltage + _b1;
}

/*! \brief Gets the voltage read from the most recent Lidar read. 
* \return 				The value the Lidar returned in volts.
*/
float Lidar::getVoltage(){
	return _voltage;
}

/*! \brief Gets the distance read from the most recent Lidar read. 
*
* \return 				The distance read from the Lidar in meters.
*/
float Lidar::getDistance(){
	return _distance;
}

///*** Crawler Factory Class ***///

/*! Default constructor for factory.
*/
CrawlerFactory::CrawlerFactory():SSFactory(){
}

/*! \brief Get the next configurationj state in the configuration space. */
Configuration* nextState(Configuration * state, Configuration * action){

}

