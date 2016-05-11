// Library for interfacing with the crawler robot using Arduino libraries

#ifndef CRAWLER_H
#define CRAWLER_H

// Used to make crawler configuration objects.
#include <Arduino.h>
#include <rl-lib_state.h>

/*! \brief Arm configuration class to hold joint variables used with the State class. */

class ArmConfig : public Configuration{
public:

	float angle1, angle2; /**< Angles for each of the joints. */

	/*! Constructor for ArmConfig
	* \param[in] angle1 	Angle for Joint 1.
	* \param[in] angle2 	Angle for Joint 2.
	*/
	ArmConfig(float angle1, float angle2);

	/*! \brief Do the comparison on the configuration.
	*
	* 	Will return false as the abstract type cannot be compared to the derived type.
	*
	* \param[in] c 		The Configuration to compare.
	* \return 			If the configuration passed in is equal to this one.
	*/
	bool doComparison(Configuration * c);
	bool Compare(Configuration * c);
	bool Compare(ArmConfig * c);

};

/*! \brief Action configuration class to hold change in joint variables used with the Action class. */

class ActionConfig : public Configuration{
public:

	float a1, a2; /**< Action for each joint. */

	/*! Constructor for ActionConfig
	* \param[in] a1 	 	Action for Joint 1.
	* \param[in] a2 		Action for Joint 2.
	*/
	ActionConfig(float a1, float a2);

	/*! \brief Accept a configuration to do a comparision on.
	* \param[in] c 		The configuration to compare with this one.
	* \return 			If the comparison was sucessful.
	*/
	bool doComparison(Configuration * c);

	/*! \brief Do the comparison on the configuration.
	* \param[in] c 		The configuration to compare.
	* \return 			If the configuration passed in is equal to this one.
	*/
	bool Compare(Configuration * c);

	/*! \brief Do the comparison on the action configuration.
	* \param[in] c 		The ActionConfig to compare.
	* \return 			If the configuration passed in is equal to this one.
	*/
	bool Compare(ActionConfig * c);
	
};

/*! \brief Arm class for assigning angles to the crawler arm. */
/**
* This class is for assigning arm values but itself does not command
* the hardware. The calibrated values must be taken from it and given to
* the servo driver.
*/
class Arm{
public:
	/*! \brief Default constructor */
	Arm();

	/*! \brief Sets joint angles for the arm class. 
	* \param[in] joint 		Which joint in the chain to be commanded.
	* \param[in] angle 		The angle to which the joint will be commanded.
	*/
	void setJointAngle(int joint, float angle);


	/*! \brief Sets both joint angles for the arm class. 
	* \param[in] angle1		Angle of the first joint, J1.
	* \param[in] angle2		Angle of the second joint, J2.
	*/
	void setJointConfig(float angle1, float angle2);

	/*! \brief Gets joint angles for the arm class. 
	* \param[in] joint 		The joint to get the angle for.
	* \return 				The angle for the given joint in degrees.
	*/
	float getJointAngle(int joint);

	/*! \brief Gets joint angles for the arm class to be sent to the servos. 
	*
	* Takes into account any calibration parameters
	* needed for correct angle mapping.
	*
	* \param[in] joint 		The joint to get the angle for.
	* \return 				The angle for the given joint in degrees.
	*/
	float getJointAngleCalibrated(int joint);

	/*! \brief Gets the Arduino pin assignment for that joint.
	* \param[in] joint 		The joint to get the pin number for.
	* \return 				The pin number for that joint.
	*/
	int getJointPin(int joint);
private:
	// Joint Angles in degrees
	ArmConfig _arm; /**< The angles for each one of the joints. */
	// Calibration parameters
	float _m1, _m2, _b1, _b2; /**< The slope and offset calibration parameters for each of the joints. */
};

/*! \brief Lidar class for reading distance values from Lidar. */
/**
* This class is for reading the SF10/A Laser Altimeter for getting distance 
* measurements to an object.
*/
class Lidar{
public:
	/*! \brief Default constructor */
	Lidar();

	/*! \brief Grabs a distance reading from the Lidar. */
	void readSensor();

	/*! \brief Grabs N distance readings from the Lidar and saves their average.
	* \param[in] n 		The number of readings to take.
	*/
	void readSensorNTimes(int n);

	/*! \brief Gets the voltage read from the most recent Lidar read. 
	* \return 				The value the Lidar returned in volts.
	*/
	float getVoltage();

	/*! \brief Gets the distance read from the most recent Lidar read. 
	*
	* \return 				The distance read from the Lidar in meters.
	*/
	float getDistance();

private:
	// Data read from the Lidar.
	float _voltage; 	/**< The voltage value read from the Lidar.*/
	float _distance; /**< The distance converted from the voltage value.*/
	// Calibration parameters
	float _m1, _b1; /**< The slope and offset calibration parameters used to convert volts into meters. */
};

/*! \brief Factory class for constructing the state space for the crawler class. */
/**
	This class takes in a StateSpace pointer, and sets up the space for the crawler
	application. It arranges the state space into a 5x5 2D grid with the different 
	joint configurations for each state.
*/
class CrawlerFactory : public SSFactory{
public:
	/*! \brief Default constructor for factory.
	*/
	CrawlerFactory();

	/*! \brief Compare two ArmConfigs and see if they are equal.
	*
	*/
	bool CompareArms(ArmConfig * arm1, ArmConfig * arm2);
	
private:
	/*! \brief Get the next configuration state in the configuration space. */
	ArmConfig * nextState(ArmConfig * state, ActionConfig * action);
};


#endif