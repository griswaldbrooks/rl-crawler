// Basic library for doing Value Iteration on Crawler Robot.

#ifndef CRAWLER_BASIC_H
#define CRAWLER_BASIC_H

// Used to make crawler configuration objects.
#include <Arduino.h>

// Class declarations.
class ActionConfig;
class ArmConfig;
class CrawlerState;
class CrawlerAction;

/*! \brief Printer object for taking data from different classes and printing them to the terminal. */
class Printer{
public:
	/*! Default Constructor */
	Printer();

	/*! Print the Crawler State. 
	* \param[in] state 	State to print.
	*/
	void print(CrawlerState * state);

	/*! Print the Crawler Action. 
	* \param[in] action 	Action to print.
	*/
	void print(CrawlerAction * action);
	
	/*! Print the ArmConfig. 
	* \param[in] armconfig 	ArmConfig to print.
	*/
	void print(ArmConfig * armconfig);
	
	/*! Print the ActionConfig. 
	* \param[in] actionconfig 	ActionConfig to print.
	*/
	void print(ActionConfig * actionconfig);
};

/*! \brief Action configuration class to hold change in joint variables used with the Action class. */

class ActionConfig{
public:

	float a1, a2; /**< Action for each joint. */

	/*! Constructor for ActionConfig
	* \param[in] a1 	 	Action for Joint 1.
	* \param[in] a2 		Action for Joint 2.
	*/
	ActionConfig(float a1, float a2);

	/*! \brief Print action using printer. 
	* \param[in] printer 	Printer object that will extract Action data.
	*/
	void sendToPrint(Printer & printer);
};

/*! \brief Arm configuration class to hold joint variables used with the State class. */

class ArmConfig{
public:

	float angle1, angle2; /**< Angles for each of the joints. */

	/*! Constructor for ArmConfig
	* \param[in] angle1 	Angle for Joint 1.
	* \param[in] angle2 	Angle for Joint 2.
	*/
	ArmConfig(float angle1, float angle2);
	
	/*! Copy Constructor for ArmConfig. */
	ArmConfig(const ArmConfig&);

	/*! Assignment operator. */
	ArmConfig& operator=(const ArmConfig&); 

	/*! \brief Print arm using printer. 
	* \param[in] printer 	Printer object that will extract arm data.
	*/
	void sendToPrint(Printer & printer);
};

/*! Equality operator for comparing ActionConfigs 
	* \param[in] lhs		The left hand side of the operator.
	* \param[in] rhs 		The right hand side of the operator.
	*/
bool operator==(ActionConfig & lhs, ActionConfig & rhs);

/*! Equality operator for comparing ArmConfigs 
* \param[in] lhs		The left hand side of the operator.
* \param[in] rhs 		The right hand side of the operator.
*/
bool operator==(ArmConfig & lhs, ArmConfig & rhs);

/*! Sum operator for combining Arms and Actions
* \param[in] lhs		The ArmConfig on the left hand side of the operator.
* \param[in] rhs 		The ActionConfig on the right hand side of the operator.
*/
ArmConfig operator+(ArmConfig & lhs, ActionConfig & rhs);
/*! Sum operator for combining Arms and Actions
* \param[in] lhs		The ActionConfig on the left hand side of the operator.
* \param[in] rhs 		The ArmConfig on the right hand side of the operator.
*/
ArmConfig operator+(ActionConfig & lhs, ArmConfig & rhs);


/*! \brief ActionConfigSet class. */
/**
* This is an aggregate class for containing a set of Action Configuration objects.
*/

class ActionConfigSet{
public:
	/*! \brief Default constructor for the set. */
	ActionConfigSet();

	/*! \brief Constructor for the set with known size 
	* \param[in] n 		The number of configurations in the set.
	*/
	ActionConfigSet(int n);

	/*! \brief Destructor */
	~ActionConfigSet();

	/*! \brief Add Configuration to the set. 
	* \param[in] c	Configuration to be added to the set.
	*/
	void push(ActionConfig * c);

	/*! \brief Remove Configuration from the set. 
	* \return		Configuration object removed from the set.
	*/
	ActionConfig * pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Configurations are in this set?
	* \return 		Returns the number of Configurations in the set.
	*/
	int size();	

	/*! \brief Clear the set. */
	void clear();

	/*! \brief Declare iterator class as friend. */
	friend class ActionConfigSetIter;

private:
	ActionConfig ** _configs; 	/**< Configurations held. */
	int _last; 					/**< Index of the current Configuration being examined. */
	int _capacity; 				/**< Number of elements in the array */
};

/*! \brief Iterator class for ActionConfigSet. */
class ActionConfigSetIter{
public:
	/*! \brief Constructor taking ActionConfigSet associated with iterator.
	* \param[in] as 	Pointer to the ActionSet for this iterator.
	*/
	ActionConfigSetIter(const ActionConfigSet * as);

	/*! \brief Set the iterator to the first element of the set. */
	void first();

	/*! \brief Increment the iterator. */
	void next();

	/*! \brief Check to see if we are at the last element. */
	bool isDone();

	/*! \brief Dereference operator. */
	ActionConfig * operator *();

	/*! \brief Arrow operator. */
	ActionConfig** operator->() const;

private:
	const ActionConfigSet * _as; 	/**< Pointer to the ActionSet for this iterator. */
	int _current; 				/**< Index of the current element. */
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

// Declaration of Crawler State class.
class CrawlerState;

/*! \brief Crawler Action class. */
/**
* This class is for creating crawler actions that can be used with
* different Reinforcement Learning Algorithms.
*/

// class CrawlerAction{
// public:
// 	/*! \brief Default constructor
// 	*/
// 	CrawlerAction();

// 	/*! \brief Constructor for a given configuration.
// 	*/
// 	CrawlerAction(ActionConfig * a);

// 	/*! \brief Updates the reward through an average. 
// 	* \param[in] r 	The reward value to incorporate.
// 	*/
// 	void updateReward(float r);

// 	/*! \brief Sets the reward value directly.
// 	* \param[in] r 	The value to set the reward to.
// 	*/
// 	void setReward(float r);

// 	/*! \brief Returns the current reward.
// 	* \return 		The value of the current reward for that action.
// 	*/
// 	float getReward();

// 	/*! \brief Change the action. 
// 	* \param[in] a 	The action to do in the configuration space.
// 	*/
// 	void setConfig(ActionConfig * a);

// 	/*! \brief Get the action. 
// 	* \return 		The action to do in the configuration space..
// 	*/
// 	ActionConfig * getConfig();

// 	/*! \brief Set next state. 
// 	* \param[in] s 	Pointer to the state reached for doing this action (from the parent state).
// 	*/
// 	void setNextState(CrawlerState * s);

// 	/*! \brief Get next state. 
// 	* \return  	Pointer to the state reached for doing this action (from the parent state).
// 	*/
// 	CrawlerState * getNextState();

// 	/*! \brief Print action using printer. 
// 	* \param[in] printer 	Printer object that will extract Action data.
// 	*/
// 	void sendToPrint(Printer & printer);

// private:
// 	CrawlerState * _state; 			/**< State reached by doing this action. Implicitly the is dependent on the state that holds this action. */
// 	ActionConfig * _action; 	/**< Movement to do in the configuration space. */
// 	float _reward; 				/**< The reward for doing that action. */

// };

class CrawlerAction{
public:
	/*! \brief Default constructor
	*/
	CrawlerAction();

	/*! \brief Constructor for a given configuration.
	*/
	CrawlerAction(ActionConfig * a);

	/*! \brief Updates the reward through an average. 
	* \param[in] r 	The reward value to incorporate.
	*/
	void updateReward(float r);

	/*! \brief Print action using printer. 
	* \param[in] printer 	Printer object that will extract Action data.
	*/
	void sendToPrint(Printer & printer);

// private:
	CrawlerState * nextState; 			/**< State reached by doing this action. Implicitly the is dependent on the state that holds this action. */
	ActionConfig * config; 	/**< Movement to do in the configuration space. */
	float reward; 				/**< The reward for doing that action. */

};

// Declaration of CrawlerActionSet iterator.
class CrawlerActionSetIter;

/*! \brief CrawlerActionSet class. */
/**
* This is an aggregate class for containing a set of CrawlerAction objects.
*/

class CrawlerActionSet{
public:
	/*! \brief Default constructor for the set. */
	CrawlerActionSet();

	/*! \brief Constructor for the set with known size 
	* \param[in] n 		The number of actions in the set.
	*/
	CrawlerActionSet(int n);

	/*! \brief Default destructor for the set. */
	~CrawlerActionSet();

	/*! \brief Add Action to the set. 
	* \param[in] a	Action to be added to the set.
	*/
	void push(CrawlerAction * a);

	/*! \brief Remove Action from the set. 
	* \return		Action object removed from the set.
	*/
	CrawlerAction * pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Actions are in this set?
	* \return 		Returns the number of Actions in the set.
	*/
	int size();	

	/*! \brief Declare iterator class as friend. */
	friend class CrawlerActionSetIter;

	/*! \brief Create iterator for set. */
	CrawlerActionSetIter * createIter() const;

private:
	CrawlerAction ** _actions; 	/**< Actions held. */
	int _last; 			/**< Index of the last Configuration in the set. */
	int _capacity; 		/**< Number of elements in the array */
};

/*! \brief Iterator class for ActionSet. */
class CrawlerActionSetIter{
public:
	/*! \brief Constructor taking ActionSet associated with iterator.
	* \param[in] as 	Pointer to the ActionSet for this iterator.
	*/
	CrawlerActionSetIter(const CrawlerActionSet * as);

	/*! \brief Set the iterator to the first element of the set. */
	void first();

	/*! \brief Increment the iterator. */
	void next();

	/*! \brief Check to see if we are at the last element. */
	bool isDone();

	/*! \brief Dereference operator. */
	CrawlerAction * operator *();

	/*! \brief Arrow operator. */
	CrawlerAction ** operator->() const;

private:
	const CrawlerActionSet * _as; 	/**< Pointer to the ActionSet for this iterator. */
	int _current; 				/**< Index of the current element. */
};


/*! \brief Crawler State class. */
/**
* This class is for creating states for the crawler robot that can be used with
* different Reinforcement Learning Algorithms.
*/

class CrawlerState{
public:
	/*! \brief Default constructor.
	*/
	CrawlerState();

	/*! \brief Constructor for known action set size.
	* \param[in] n 		The number of actions for the state.
	*/
	CrawlerState(int n);

	/*! \brief Constructor with Configuration state and Action set.
	* \param[in] state 		The Configuration for that state.
	* \param[in] actions	The ConfigurationSet describing the actions in that state.
	*/
	CrawlerState(ArmConfig * state, ActionConfigSet * actions);

	/*! \brief Default destructor. */
	~CrawlerState();

	/*! \brief Set the state configuration. 
	* \param[in] config 	Configuration to set the State to.
	*/
	void setConfig(ArmConfig * config);

	/*! \brief Method to get system configuration for this state. 
	* \return 				The Configuration of this state.
	*/
	ArmConfig * getConfig();

	/*! \brief Set the state value.
	* \param[in] value 		Value to set the state to.
	*/
	void setValue(float value);

	/*! \brief Get the state value.
	* \return 				Value of the state.
	*/
	float getValue();

	/*! \brief Get the previous state value.
	* \return 				Previous value of the state.
	*/
	float getPreviousValue();

	/*! \brief Set the policy.
	* \param[in] index 		Index of the policy to follow.
	*/
	void setPolicy(int index);

	/*! \brief Get the policy.
	* \return 				Index of the policy the state follows.
	*/
	int getPolicy();

	/*! \brief Get the previous policy.
	* \return 				Index of the previous policy the state followed.
	*/
	int getPreviousPolicy();

	/*! \brief Get the ActionSet for this State.
	* \return 				Pointer to the ActionSet that belongs to this State.
	*/
	CrawlerActionSet * getActions();

	/*! \brief Print state using printer. 
	* \param[in] printer 	Printer object that will extract State data.
	*/
	void sendToPrint(Printer & printer);

private:
	ArmConfig * _config; 	/**< The physical state of the system. */
	
	float _value, _pvalue; 	/**< The current and previous values of the state. */
	int _policy, _ppolicy; 	/**< The current and previous policy to do in that state, represented by indices for the action set. */

	CrawlerActionSet * _actions;	/**< The different actions that can be done in that state. */	
};

/*! \brief Basic State Space class for the Crawler. */
/** 
* The state space is an aggregate class for containing the states and 
* setting the relationships between states.
*/
// Declaration of the CrawlerStateSpace itereator.
class CrawlerStateSpaceIter;

class CrawlerStateSpace{
public:
	/*! \brief Default constructor
	*/
	CrawlerStateSpace();

	/*! \brief Constructor for known number of states.
	* \param[in] n 		Number of system states.
	*/
	CrawlerStateSpace(int n);

	/*! \brief Destructor. */
	~CrawlerStateSpace();

	/*! \brief Add State to the set. 
	* \param[in] s	Pointer to the State to be added to the set.
	*/
	void push(CrawlerState * s);

	/*! \brief Remove State from the set. 
	* \return		Pointer to the State object removed from the set.
	*/
	CrawlerState * pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Configurations are in this set?
	* \return 		Returns the number of Configurations in the set.
	*/
	int size();	

	/*! \brief Set the Global Action Configuration Space
	* \param[in] gacSet 	The set of all possible action configurations for the space.
	*/
	void setGlobalActionConfigs(ActionConfigSet * gacSet);

	/*! \brief Get the Global Action Configuration Space
	* \return  				The set of all possible action configurations for the space.
	*/
	ActionConfigSet * getGlobalActionConfigs();

	/*! \brief Declare iterator class as friend. */
	friend class CrawlerStateSpaceIter;

	/*! \brief Create iterator for set. */
	CrawlerStateSpaceIter * createIter() const;

private:

	CrawlerState ** _states; 	/**< Pointer to the memory space that hold the value map. */
	int _last; 			/**< Index of the last state in the set. */
	int _capacity; 		/**< Number of elements in the array. */
	ActionConfigSet * _globalActionConfigs; /**< Pointer to the full set of potential action configurations. */
};

/*! \brief Iterator class for CrawlerStateSpace. */
class CrawlerStateSpaceIter{
public:
	/*! \brief Constructor taking CrawlerStateSpace associated with iterator.
	* \param[in] as 	Pointer to the CrawlerStateSpace for this iterator.
	*/
	CrawlerStateSpaceIter(const CrawlerStateSpace * ss);

	/*! \brief Set the iterator to the first element of the set. */
	void first();

	/*! \brief Increment the iterator. */
	void next();

	/*! \brief Check to see if we are at the last element. */
	bool isDone();

	/*! \brief Dereference operator. */
	CrawlerState * operator *();

	/*! \brief Arrow operator. */
	CrawlerState** operator->() const;

private:
	const CrawlerStateSpace * _ss; 	/**< Pointer to the ActionSet for this iterator. */
	int _current; 				/**< Index of the current element. */
};


/*! \brief Factory class for constructing the state space for the application. */

class CrawlerSSFactory{
public:
	/*! \brief Default constructor for factory.
	*/
	CrawlerSSFactory();

	/*! \brief Sets up the CrawlerStateSpace object according to the crawler configuration. */
	void buildStateSpace();

	/*! \brief Produces the configured CrawlerStateSpace object.
	* \return 	Returns the state space configured for the application.
	*/
	CrawlerStateSpace * getStateSpace();

private:
	/*! \brief Check to see if a state configuration is in a set. 
	* \param[in] state 		State Configuration to look for.
	* \param[in] set 		Set to check.
	*/
	bool inSet(ArmConfig * state, CrawlerStateSpace * set);

	/*! \brief Checks to see if a state configuration is valid according to the crawler limits.
	* \param[in] state 		The state to check.
	* \return 				If the state is within limits.
	*/
	bool isValidConfig(ArmConfig * state);

	/*! \brief Get the next configuration state in the configuration space. 
	* \param[in] 	state 	The current state that the action is performed in.
	* \param[in]	action 	The action performed in that state.
	* \return 				The next state in the configuration space.
	*/
	ArmConfig * nextConfig(ArmConfig * state, ActionConfig * action);

	/*! \brief Get the initial configuration state in the configuration space. 
	* \return 				The first state in the system.
	*/
	ArmConfig * initialConfig();

	/*! \brief Get the action set for that state.
	* \param[in] 	state 		The system state.
	* \param[out] 	actions		Pointer to the set of actions that can be performed in that state.
	*/
	void getActionConfigs(ArmConfig * state, ActionConfigSet * actions);

	CrawlerStateSpace * _ss;   /**< State space object being worked on. */

};

#endif

