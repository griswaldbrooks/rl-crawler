// Library for interfacing with the crawler robot using Arduino libraries



// Include Arduino Libray
#include <Arduino.h>
// Include Crawler Library
#include <crawler_basic.h>

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

// Large float used to detect if this is the first time the reward was
// updated.
#define LARGE_FLOAT		(1e9)
// Weighting factor for incorporating rewards. Value is between 0 and 1.
#define REWARD_WEIGHT	(0.5)

#define DEFAULT_STATE_SPACE_SIZE	25
#define DEFAULT_ACTION_SET_SIZE		4
#define DEFAULT_CONFIGURATION_SET_SIZE 	10

// Arm State Space Parameters (degrees)
#define ARM_JOINT_ANGLE_MAX 		45
#define ARM_JOINT_ANGLE_MIN 		-45
#define ARM_JOINT_ANGLE_INC 		20

int freeRam (){
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

///*** Printer Class ***///
/*! Default Constructor */
Printer::Printer(){}

/*! Print the Crawler State. 
* \param[in] state 	State to print.
*/
void Printer::print(CrawlerState * state){
	// Print configuration.
	Serial.println(F(" State Configuraton: "));
	state->getConfig()->sendToPrint(*this);
	
	// Print values.
	Serial.print(F(" Value: "));
	Serial.println(state->getValue());
	Serial.print(F(" Previous Value: "));
	Serial.println(state->getPreviousValue());
	
	// Print policies.
	Serial.print(F(" Policy: "));
	Serial.println(state->getPolicy());
	Serial.print(F(" Previous Policy: "));
	Serial.println(state->getPreviousPolicy());
	
	// Print action set.
	CrawlerActionSetIter as(state->getActions());
	for(as.first(); as.isDone(); as.next()){
		(*as)->sendToPrint(*this);
	}
}

/*! Print the Crawler Action. 
* \param[in] action 	Action to print.
*/
void Printer::print(CrawlerAction * action){
	// Print configuration.
	Serial.println(F(" Action Configuration: "));
	action->config->sendToPrint(*this);

	// Print reward.
	Serial.print(F(" Reward: "));
	Serial.println(action->reward);

	Serial.println(F(" Next State: "));
	// Print next state.
	if(action->nextState){
		action->nextState->getConfig()->sendToPrint(*this);
	}
}

/*! Print the ArmConfig. 
* \param[in] armconfig 	ArmConfig to print.
*/
void Printer::print(ArmConfig * armconfig){
	Serial.print(F(" Angle1: "));
	Serial.println(armconfig->angle1);
	Serial.print(F(" Angle2: "));
	Serial.println(armconfig->angle2);
}

/*! Print the ActionConfig. 
* \param[in] actionconfig 	ActionConfig to print.
*/
void Printer::print(ActionConfig * actionconfig){
	Serial.print(F(" A1: "));
	Serial.println(actionconfig->a1);
	Serial.print(F(" A2: "));
	Serial.println(actionconfig->a2);
}

///*** ArmConfig Class ***///

/*! Constructor for ArmConfig
* \param[in] angle1 	Angle for Joint 1.
* \param[in] angle2 	Angle for Joint 2.
*/
ArmConfig::ArmConfig(float angle1, float angle2)
	:angle1(angle1), angle2(angle2){}

/*! Copy Constructor for ArmConfig
	*/
ArmConfig::ArmConfig(const ArmConfig& arm){
	angle1 = arm.angle1;
	angle2 = arm.angle2;
}

/*! Assignment operator. */
ArmConfig& ArmConfig::operator=(const ArmConfig& rhs){
	// Check for self-assignment.
	if(this != &rhs){
		this->angle1 = rhs.angle1;
		this->angle2 = rhs.angle2;
	}
	return *this;
}

/*! \brief Print arm using printer. 
* \param[in] printer 	Printer object that will extract arm data.
*/
void ArmConfig::sendToPrint(Printer & printer){
	printer.print(this);
}

/*! Equality operator for comparing ArmConfigs 
* \param[in] lhs		The left hand side of the operator.
* \param[in] rhs 		The right hand side of the operator.
*/
bool operator==(ArmConfig & lhs, ArmConfig & rhs){
	if(lhs.angle1 != rhs.angle1){
		return false;
	}
	else if(lhs.angle2 != rhs.angle2){
		return false;
	}
	return true;
}

///*** ActionConfig Class ***///

/*! Constructor for ActionConfig
* \param[in] a1 		Action for Joint 1.
* \param[in] a2 		Action for Joint 2.
*/
ActionConfig::ActionConfig(float a1, float a2)
	:a1(a1),a2(a2){}

/*! Equality operator for comparing ArmConfigs 
* \param[in] lhs		The left hand side of the operator.
* \param[in] rhs 		The right hand side of the operator.
*/
bool operator==(ActionConfig & lhs, ActionConfig & rhs){
	if(lhs.a1 != rhs.a1){
		return false;
	}
	else if(lhs.a2 != rhs.a2){
		return false;
	}
	return true;
}

/*! \brief Print arm using printer. 
* \param[in] printer 	Printer object that will extract arm data.
*/
void ActionConfig::sendToPrint(Printer & printer){
	printer.print(this);
}

/*! Sum operator for combining Arms and Actions
* \param[in] lhs		The ArmConfig on the left hand side of the operator.
* \param[in] rhs 		The ActionConfig on the right hand side of the operator.
*/
ArmConfig operator+(ArmConfig & lhs, ActionConfig & rhs){
	return ArmConfig(lhs.angle1 + rhs.a1, lhs.angle2 + rhs.a2);
}
/*! Sum operator for combining Arms and Actions
* \param[in] lhs		The ActionConfig on the left hand side of the operator.
* \param[in] rhs 		The ArmConfig on the right hand side of the operator.
*/
ArmConfig operator+(ActionConfig & lhs, ArmConfig & rhs){
	return ArmConfig(lhs.a1 + rhs.angle1, lhs.a2 + rhs.angle2);
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
		// Sum the readings.
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


///*** ActionConfigSet ***///

/*! \brief Default constructor for the set. */
ActionConfigSet::ActionConfigSet(){
	_last = -1;
	_capacity = DEFAULT_CONFIGURATION_SET_SIZE;
	_configs = new ActionConfig*[_capacity];
}

/*! \brief Constructor for the set of known size. */
ActionConfigSet::ActionConfigSet(int n){
	_last = -1;
	_capacity = n;
	_configs = new ActionConfig*[_capacity];
}

/*! \brief Destructor */
ActionConfigSet::~ActionConfigSet(){
	delete [] _configs;
}

/*! \brief Add Configuration to the set. 
* \param[in] c	Pointer to the Configuration to be added to the set.
*/
void ActionConfigSet::push(ActionConfig * c){
	// Check to see if we are at capacity
	if(_last != (_capacity - 1)){
		// Increment index to the next available position.
		_last++;
		// Add the element.
		_configs[_last] = c;
	}
}

/*! \brief Remove Configuration from the set. 
* \return		Pointer to the Configuration object removed from the set.
*/
ActionConfig * ActionConfigSet::pop(){
	if(!this->isEmpty()){
		// Return the current config and decrement the index.
		return _configs[_last--];
	}
	else{
		return _configs[0];
	}
}

/*! \brief Are there any objects in the set? 
* \return 		Returns true if there is anything in the set.
*/
bool ActionConfigSet::isEmpty(){
	return _last == -1;
}

/*! \brief How many Configurations are in this set?
* \return 		Returns the number of Configurations in the set.
*/
int ActionConfigSet::size(){
	return _last + 1;
}

/*! \brief Clear the set. */
void ActionConfigSet::clear(){
	_last = -1;
}

///*** Iterator class for ActionConfigSet. ***///

/*! \brief Constructor taking ActionConfigSet associated with iterator.
* \param[in] as 	Pointer to the ActionConfigSet for this iterator.
*/
ActionConfigSetIter::ActionConfigSetIter(const ActionConfigSet * as){
	_as = as;
}

/*! \brief Set the iterator to the first element of the set. */
void ActionConfigSetIter::first(){
	_current = 0;
}

/*! \brief Increment the iterator. */
void ActionConfigSetIter::next(){
	_current++;
}

/*! \brief Check to see if we are at the last element. */
bool ActionConfigSetIter::isDone(){
	// Are we at the index after the last index?
	return _current == (_as->_last + 1);
}

/*! \brief Dereference operator. */
ActionConfig * ActionConfigSetIter::operator *(){
	return _as->_configs[_current];
}

/*! \brief Arrow operator. */
ActionConfig** ActionConfigSetIter::operator->() const{
	return &(_as->_configs[_current]);
}

///*** Crawler Action Class ***///

// Default Constructor for Action Class.
CrawlerAction::CrawlerAction(){
	// Initialize reward with specific large value.
	reward = LARGE_FLOAT;
	// Initialize pointers.
	nextState = NULL;
	config = NULL;
}

/*! \brief Constructor for a given configuration.
*/
CrawlerAction::CrawlerAction(ActionConfig * a){
	// Initialize reward with specific large value.
	reward = LARGE_FLOAT;	
	// Set configuration.
	config = a;
	// Initialize pointers.
	nextState = NULL;
}

// Updates the reward through an average.
void CrawlerAction::updateReward(float r){
	// If this is the first time the reward is being updated, set
	// it to the new reward.
	if(reward == LARGE_FLOAT){
		reward = r;
	}
	else{
		// Do a weighted average of the new and previous reward.
		reward = REWARD_WEIGHT*r + (1 - REWARD_WEIGHT)*reward;
	}
}

/*! \brief Print state using printer. */
void CrawlerAction::sendToPrint(Printer & printer){
	printer.print(this);
}

///*** CrawlerActionSet class. ***///

/*! \brief Default constructor for the set. */
CrawlerActionSet::CrawlerActionSet(){
	_last = -1;
	_capacity = DEFAULT_ACTION_SET_SIZE;
	_actions = new CrawlerAction*[_capacity];
}

/*! \brief Constructor for the set with known size 
* \param[in] n 		The number of actions in the set.
*/
CrawlerActionSet::CrawlerActionSet(int n){
	_last = -1;
	_capacity = n;
	_actions = new CrawlerAction*[_capacity];
}

/*! \brief Default destructor for the set. */
CrawlerActionSet::~CrawlerActionSet(){
	delete [] _actions;
}

/*! \brief Add Action to the set. 
* \param[in] a	Action to be added to the set.
*/
void CrawlerActionSet::push(CrawlerAction * a){
	// Check to see if we are at capacity
	if(_last != (_capacity - 1)){
		// Increment index to the next available position.
		_last++;
		// Add the element.
		_actions[_last] = a;
	}
}

/*! \brief Remove Action from the set. 
* \return		Action object removed from the set.
*/
CrawlerAction * CrawlerActionSet::pop(){
	if(!this->isEmpty()){
		// Return the current config and decrement the index.
		return _actions[_last--];
	}
	else{
		return _actions[0];
	}
}

/*! \brief Are there any objects in the set? 
* \return 		Returns true if there is anything in the set.
*/
bool CrawlerActionSet::isEmpty(){
	return _last == -1;
}

/*! \brief How many Actions are in this set?
* \return 		Returns the number of Actions in the set.
*/
int CrawlerActionSet::size(){
	return _last + 1;
}

/*! \brief Create iterator for set. */
CrawlerActionSetIter * CrawlerActionSet::createIter() const{
	return new CrawlerActionSetIter(this);
}


///*** Iterator class for CrawlerActionSet. ***///

/*! \brief Constructor taking ActionSet associated with iterator.
* \param[in] as 	Pointer to the ActionSet for this iterator.
*/
CrawlerActionSetIter::CrawlerActionSetIter(const CrawlerActionSet * as){
	_as = as;
}

/*! \brief Set the iterator to the first element of the set. */
void CrawlerActionSetIter::first(){
	_current = 0;
}

/*! \brief Increment the iterator. */
void CrawlerActionSetIter::next(){
	_current++;
}

/*! \brief Check to see if we are at the last element. */
bool CrawlerActionSetIter::isDone(){
	// Are we at the index after the last index?
	return _current == (_as->_last + 1);
}

/*! \brief Dereference operator. */
CrawlerAction * CrawlerActionSetIter::operator *(){
	return _as->_actions[_current];
}

/*! \brief Arrow operator. */
CrawlerAction ** CrawlerActionSetIter::operator->() const{
	return &(_as->_actions[_current]);
}


///*** CrawlerState Class ***///

// Default Constructor for State Class.
CrawlerState::CrawlerState(){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set action set size.
	_actions = new CrawlerActionSet(DEFAULT_ACTION_SET_SIZE);
	// Initialize pointers.
	_config = NULL;
}

/*! \brief Constructor for known action set size.
* \param[in] n 		The number of actions for the state.
*/
CrawlerState::CrawlerState(int n){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set action set size.
	_actions = new CrawlerActionSet(n);
	// Initialize pointers.
	_config = NULL;
}

/*! \brief Constructor with Configuration state and Action set.
* \param[in] state 		The Configuration for that state.
* \param[in] actions	The ConfigurationSet describing the actions in that state.
*/
CrawlerState::CrawlerState(ArmConfig * state, ActionConfigSet * actions){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set configuration.
	_config = state;
	Serial.print(F("Before Crawler ActionSet: "));
	Serial.println(freeRam());
	// Instantiate Action Set.
	_actions = new CrawlerActionSet(actions->size());
	Serial.print(F("After Crawler ActionSet: "));
	Serial.println(freeRam());
	// Loop through actions and incorporate them into the set.
	for(; actions->size();){
		// Add it to the set.
		_actions->push(new CrawlerAction(actions->pop()));
	}

}

// Default Desctructor for State Class.
CrawlerState::~CrawlerState(){
	// Free the objects.
	delete _config;
	delete _actions;
}

// Method to set system configuration for this state.
void CrawlerState::setConfig(ArmConfig * config){
	_config = config;
}

// Method to get system configuration for this state.
ArmConfig * CrawlerState::getConfig(){
	return _config;
}

/*! \brief Set the state value.
* \param[in] value 		Value to set the state to.
*/
void CrawlerState::setValue(float value){
	// Remember previous value.
	_pvalue = _value;
	_value = value;
}

/*! \brief Get the state value.
* \return 				Value of the state.
*/
float CrawlerState::getValue(){return _value;}

/*! \brief Get the previous state value.
* \return 				Previous value of the state.
*/
float CrawlerState::getPreviousValue(){return _pvalue;}

/*! \brief Set the policy.
* \param[in] index 		Index of the policy to follow.
*/
void CrawlerState::setPolicy(int index){
	if((index > -1) && (index < _actions->size())){
		_ppolicy = _policy;
		_policy = index;
	}
}

/*! \brief Get the policy.
* \return 				Index of the policy the state follows.
*/
int CrawlerState::getPolicy(){return _policy;}

/*! \brief Get the previous policy.
* \return 				Index of the previous policy the state followed.
*/
int CrawlerState::getPreviousPolicy(){return _ppolicy;}

/*! \brief Get the ActionSet for this State.
* \return 				Pointer to the ActionSet that belongs to this State.
*/
CrawlerActionSet * CrawlerState::getActions(){
	return _actions;
}

/*! \brief Print state using printer. */
void CrawlerState::sendToPrint(Printer & printer){
	printer.print(this);
}

///*** CrawlerStateSpace Class ***///

CrawlerStateSpace::CrawlerStateSpace(){
	// Index of the last state in the space.
	_last = -1;
	_capacity = DEFAULT_STATE_SPACE_SIZE;
	// Set state space vector.
	_states = new CrawlerState*[_capacity];	
}

/*! \brief Constructor for known number of states.
* \param[in] n 		Number of system states.
*/
CrawlerStateSpace::CrawlerStateSpace(int n){
	// Index of the last state in the space.
	_last = -1;
	_capacity = n;
	// Set state space vector.
	_states = new CrawlerState*[_capacity];	
}

/*! \brief Destructor. */
CrawlerStateSpace::~CrawlerStateSpace(){
	delete [] _states;
}

/*! \brief Add State to the space. 
*/
void CrawlerStateSpace::push(CrawlerState * s){
	// Check to see if we are at capacity
	if(_last != (_capacity - 1)){
		// Increment index to the next available position.
		_last++;
		// Add the element.
		_states[_last] = s;
	}
}

/*! \brief Remove State from the set. 
* \return		Pointer to the State object removed from the set.
*/
CrawlerState * CrawlerStateSpace::pop(){
	if(!this->isEmpty()){
		// Return the current config and decrement the index.
		return _states[_last--];
	}
	else{
		return _states[0];
	}
}

/*! \brief Are there any objects in the set? 
* \return 		Returns true if there is anything in the set.
*/
bool CrawlerStateSpace::isEmpty(){
	return _last == -1;
}

/*! \brief Set the Global Action Configuration Space
* \param[in] gacSet 	The set of all possible action configurations for the space.
*/
void CrawlerStateSpace::setGlobalActionConfigs(ActionConfigSet * gacSet){
	_globalActionConfigs = gacSet;
}

/*! \brief Get the Global Action Configuration Space
* \return  				The set of all possible action configurations for the space.
*/
ActionConfigSet * CrawlerStateSpace::getGlobalActionConfigs(){
	return _globalActionConfigs;
}

/*! \brief Returns the size of the space.
* \return 	The number of states in the space.
*/
int CrawlerStateSpace::size(){
	return _last + 1;
}

/*! \brief Create iterator for set. */
CrawlerStateSpaceIter * CrawlerStateSpace::createIter() const{
	return new CrawlerStateSpaceIter(this);
}

///*** Iterator class for CrawlerStateSpace. ***///

/*! \brief Constructor taking StateSpace associated with iterator.
* \param[in] as 	Pointer to the StateSpace for this iterator.
*/
CrawlerStateSpaceIter::CrawlerStateSpaceIter(const CrawlerStateSpace * ss){
	_ss = ss;
}

/*! \brief Set the iterator to the first element of the set. */
void CrawlerStateSpaceIter::first(){
	_current = 0;
}

/*! \brief Increment the iterator. */
void CrawlerStateSpaceIter::next(){
	_current++;
}

/*! \brief Check to see if we are at the last element. */
bool CrawlerStateSpaceIter::isDone(){
	// Are we at the index after the last index?
	return _current == (_ss->_last + 1);
}

/*! \brief Dereference operator. */
CrawlerState * CrawlerStateSpaceIter::operator *(){
	return _ss->_states[_current];
}

/*! \brief Arrow operator. */
CrawlerState** CrawlerStateSpaceIter::operator->() const{
	return &(_ss->_states[_current]);
}


///*** CrawlerStateSpace Factory Class ***///

/*! Default constructor for factory.
*/
CrawlerSSFactory::CrawlerSSFactory(){
	// Add the State Space.
	_ss = new CrawlerStateSpace(25);
	// Create the global action configuration space.
	ActionConfigSet * gacSpace = new ActionConfigSet(4);

	// Add the four configurations.
	gacSpace->push(new ActionConfig(ARM_JOINT_ANGLE_INC, 0));
	gacSpace->push(new ActionConfig(-ARM_JOINT_ANGLE_INC, 0));
	gacSpace->push(new ActionConfig(0, ARM_JOINT_ANGLE_INC));
	gacSpace->push(new ActionConfig(0, -ARM_JOINT_ANGLE_INC));

	// Set the global action configuration space to the state space.
	_ss->setGlobalActionConfigs(gacSpace);
}

/*! \brief Sets up the StateSpace object. */
void CrawlerSSFactory::buildStateSpace(){
	CrawlerStateSpace * candidateStates = new CrawlerStateSpace;
	//** Grab the initial state and action space. **//
	ArmConfig * stateConfig = initialConfig();
	Serial.print(F("Get first stateConfig."));

	ActionConfigSet * actions = new ActionConfigSet(4);
	getActionConfigs(stateConfig, actions);
	// Serial.print("Get _actions: "); Serial.println(actions->size());
	Serial.print(F("ActionConfigSet freeRam: "));
	Serial.println(freeRam());
	//** Add this to the candidate set. **//
	candidateStates->push(new CrawlerState(stateConfig, actions));
	Serial.print(F("push candidateStates freeRam: "));
	Serial.println(freeRam());
	//** Iterate through all of the candidate states. **//
	while(!candidateStates->isEmpty()){
		// Grab the top most state.
		CrawlerState * s = candidateStates->pop();
		Serial.print(F("Get crawler state from candidateStates. "));
		Printer p;
		s->sendToPrint(p);
		Serial.print(F("iter freeRam: "));
		Serial.println(freeRam());

		//** Get all of the next states for this state. **//
		Serial.print("Number of actions: ");
		Serial.println(s->getActions()->size());
		CrawlerActionSetIter as(s->getActions());
		// Serial.println("Get CrawlerActionSetIter.");
		while(!Serial.available());
		Serial.read();
		for(as.first(); !as.isDone(); as.next()){
			// Next state config.
			Serial.print(F("for freeRam: "));
			Serial.println(freeRam());
			delay(100);
			stateConfig = nextConfig(s->getConfig(), (*as)->config);
			Serial.print(F("stateConfig freeRam: "));
			Serial.println(freeRam());
			delay(100);
			
			// Check to see if it is in the candidate list or the state space.
			if(!inSet(stateConfig, candidateStates) && !inSet(stateConfig, _ss)){
				// Serial.println("It was a new state config!");
				// If not, then add to the candidate list.
				getActionConfigs(stateConfig, actions);
				Serial.print(F("actions freeRam: "));
				Serial.println(freeRam());
				delay(100);
				candidateStates->push(new CrawlerState(stateConfig, actions));				
				Serial.print(F("candidate state push freeRam: "));
				Serial.println(freeRam());
				delay(100);
			}
		}

		// Add the expanded state to the state space.
		_ss->push(s);
		Serial.print(F("Push s to _ss size = "));
		Serial.println(_ss->size());
	}
}

/*! Produces the configured StateSpace object.
* \return 	Returns the state space configured for the application.
*/
CrawlerStateSpace * CrawlerSSFactory::getStateSpace(){
	return _ss;
}

bool CrawlerSSFactory::inSet(ArmConfig * state, CrawlerStateSpace * set){
	// Iterate through the set and check for matching configurations.
	CrawlerStateSpaceIter ss(set);
	for(ss.first(); !ss.isDone(); ss.next()){
		if(*(*ss)->getConfig() == *state){ 
			// Found one!
			return true;
		}
	}
	// Didn't find one.
	return false;
}

//Is a configuration valid?
bool CrawlerSSFactory::isValidConfig(ArmConfig * state){
	return  (state->angle1 <= ARM_JOINT_ANGLE_MAX) &&
			(state->angle1 >= ARM_JOINT_ANGLE_MIN) &&
			(state->angle2 <= ARM_JOINT_ANGLE_MAX) &&
			(state->angle2 >= ARM_JOINT_ANGLE_MIN);
}

/*! \brief Get the next configuration state in the configuration space. 
* \param[in] 	state 	The current state that the action is performed in.
* \param[in]	action 	The action performed in that state.
* \return 				The next state in the configuration space.
*/
ArmConfig * CrawlerSSFactory::nextConfig(ArmConfig * state, ActionConfig * action){
	// Create new confugration.
	ArmConfig candArm(*state + *action);
	// Check to see if it's within the correct ranges.
	if(isValidConfig(&candArm)){
		// Then we can return the new arm.
		return new ArmConfig(candArm);
	}

	// Otherwise just return the old arm.
	return state;
}

/*! \brief Get the initial configuration state in the configuration space. 
* \return 				The first state in the system.
*/
ArmConfig * CrawlerSSFactory::initialConfig(){
	// Arm starts initially at the origin.
	return new ArmConfig(0,0);
}

/*! \brief Get the action set for that state.
* \param[in] 	state 	The system state.
* \return 				Pointer to the set of actions that can be performed in that state.
*/
void CrawlerSSFactory::getActionConfigs(ArmConfig * state, ActionConfigSet * actions) {
	// States have four possible choices:
	// Angle1 : Increase or Decrease by ARM_JOINT_ANGLE_INC
	// Angle2 : Increase or Decrease by ARM_JOINT_ANGLE_INC

	// Be sure that the action set is empty.
	actions->clear();

	// Get the Global Action Configuration Set
	ActionConfigSetIter gas(_ss->getGlobalActionConfigs());
	
	// Dummy variable to use for comparisons.
	ArmConfig candArm(0,0);

	// Check all of the configurations.
	for(gas.first(); !gas.isDone(); gas.next()){
		// Create the new candidate state configuration.
		candArm = *state + **gas;
		// Check to see if that is a valid configuration.
		if(isValidConfig(&candArm)){
			// If so, add it.
			actions->push(*gas);
		}	
	}
}