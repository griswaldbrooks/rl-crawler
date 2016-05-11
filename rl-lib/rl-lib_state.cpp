// Library for creating value maps

#include <rl-lib_state.h>

// Large float used to detect if this is the first time the reward was
// updated.
#define LARGE_FLOAT		(1e9)
// Weighting factor for incorporating rewards. Value is between 0 and 1.
#define REWARD_WEIGHT	(0.5)

///*** ConfigurationSet ***///

/*! \brief Default constructor for the set. */
ConfigurationSet::ConfigurationSet(){
	_last = -1;
	_capacity = DEFAULT_CONFIGURATION_SET_SIZE;
	_configs = new Configuration*[_capacity];
}

/*! \brief Constructor for the set of known size. */
ConfigurationSet::ConfigurationSet(int n){
	_last = -1;
	_capacity = n;
	_configs = new Configuration*[_capacity];
}

/*! \brief Add Configuration to the set. 
* \param[in] c	Pointer to the Configuration to be added to the set.
*/
void ConfigurationSet::push(Configuration * c){
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
Configuration * ConfigurationSet::pop(){
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
bool ConfigurationSet::isEmpty(){
	return _last == -1;
}

/*! \brief How many Configurations are in this set?
* \return 		Returns the number of Configurations in the set.
*/
int ConfigurationSet::size(){
	return _last + 1;
}

///*** Action Class ***///

// Default Constructor for Action Class.
Action::Action(){
	// Initialize reward with specific large value.
	_reward = LARGE_FLOAT;
}

/*! \brief Constructor for a given configuration.
*/
Action::Action(Configuration * a){
	// Initialize reward with specific large value.
	_reward = LARGE_FLOAT;	
	// Set configuration.
	_action = a;
}

// Updates the reward through an average.
void Action::updateReward(float r){
	// If this is the first time the reward is being updated, set
	// it to the new reward.
	if(_reward == LARGE_FLOAT){
		_reward = r;
	}
	else{
		// Do a weighted average of the new and previous reward.
		_reward = REWARD_WEIGHT*r + (1 - REWARD_WEIGHT)*_reward;
	}
}

/*! \brief Sets the reward value directly. */
void Action::setReward(float r){
	_reward = r;
}

/*! \brief Returns the current reward.*/
float Action::getReward(){
	return _reward;
}

/*! \brief Change the action. 
*/
void Action::setConfig(Configuration * a){
	_action = a;
}

/*! \brief Get the action. 
*/
Configuration * Action::getConfig(){
	return _action;
}

/*! \brief Set next state. 
* \param[in] S 	Pointer to the state reached for doing this action (from the parent state).
*/
void Action::setNextState(State * s){
	_state = s;
}

///*** ActionSet class. ***///

/*! \brief Default constructor for the set. */
ActionSet::ActionSet(){
	_last = -1;
	_capacity = DEFAULT_ACTION_SET_SIZE;
	_actions = new Action[_capacity];
}

/*! \brief Constructor for the set with known size 
* \param[in] n 		The number of actions in the set.
*/
ActionSet::ActionSet(int n){
	_last = -1;
	_capacity = n;
	_actions = new Action[_capacity];
}

/*! \brief Default destructor for the set. */
ActionSet::~ActionSet(){
	delete _actions;
}

/*! \brief Add Action to the set. 
* \param[in] a	Action to be added to the set.
*/
void ActionSet::push(Action a){
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
Action ActionSet::pop(){
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
bool ActionSet::isEmpty(){
	return _last == -1;
}

/*! \brief How many Actions are in this set?
* \return 		Returns the number of Actions in the set.
*/
int ActionSet::size(){
	return _last + 1;
}

/*! \brief Create iterator for set. */
ActionSetIter * ActionSet::createIter() const{
	return new ActionSetIter(this);
}


///*** Iterator class for ActionSet. ***///

/*! \brief Constructor taking ActionSet associated with iterator.
* \param[in] as 	Pointer to the ActionSet for this iterator.
*/
ActionSetIter::ActionSetIter(const ActionSet * as){
	_as = as;
}

/*! \brief Set the iterator to the first element of the set. */
void ActionSetIter::first(){
	_current = 0;
}

/*! \brief Increment the iterator. */
void ActionSetIter::next(){
	_current++;
}

/*! \brief Check to see if we are at the last element. */
bool ActionSetIter::isDone(){
	// Are we at the index after the last index?
	return _current == (_as->_last + 1);
}

/*! \brief Dereference operator. */
Action ActionSetIter::operator *(){
	return _as->_actions[_current];
}

/*! \brief Arrow operator. */
Action* ActionSetIter::operator->() const{
	return &(_as->_actions[_current]);
}


///*** State Class ***///

// Default Constructor for State Class.
State::State(){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set action set size.
	_actions = new ActionSet(DEFAULT_ACTION_SET_SIZE);
}

/*! \brief Constructor for known action set size.
* \param[in] n 		The number of actions for the state.
*/
State::State(int n){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set action set size.
	_actions = new ActionSet(n);
}

/*! \brief Constructor with Configuration state and Action set.
* \param[in] state 		The Configuration for that state.
* \param[in] actions	The ConfigurationSet describing the actions in that state.
*/
State::State(Configuration * state, ConfigurationSet * actions){
	// No initial value of that state.
	_value = _pvalue = 0;
	// Initialize to random policy.
	_policy = _ppolicy = 0;
	// Set configuration.
	_config = state;

	// Instantiate Action Set.
	_actions = new ActionSet(actions->size());

	// Loop through actions and incorporate them into the set.
	Action dummyAction;
	for(int i = 0; i < actions->size(); i++){
		// Set the dummy according to the configuration.
		dummyAction.setConfig(actions->pop());
		// Add it to the set.
		_actions->push(dummyAction);
	}

}

// Default Desctructor for State Class.
State::~State(){
	// Free the objects.
}

// Method to set system configuration for this state.
void State::setConfig(Configuration * config){
	_config = config;
}

// Method to get system configuration for this state.
Configuration * State::getConfig(){
	return _config;
}

///*** StateSpace Class ***///

StateSpace::StateSpace(){
	// Index of the last state in the space.
	_last = -1;
	_capacity = DEFAULT_STATE_SPACE_SIZE;
	// Set state space vector.
	_states = new State*[_capacity];	
}

/*! \brief Constructor for known number of states.
* \param[in] n 		Number of system states.
*/
StateSpace::StateSpace(int n){
	// Index of the last state in the space.
	_last = -1;
	_capacity = n;
	// Set state space vector.
	_states = new State*[_capacity];	
}

/*! \brief Add State to the space. 
*/
void StateSpace::push(State * s){
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
State * StateSpace::pop(){
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
bool StateSpace::isEmpty(){
	return _last == -1;
}

/*! \brief Returns the size of the space.
* \return 	The number of states in the space.
*/
int StateSpace::size(){
	return _last + 1;
}

/*! \brief Create iterator for set. */
StateSpaceIter * StateSpace::createIter() const{
	return new StateSpaceIter(this);
}

///*** Iterator class for StateSpace. ***///

/*! \brief Constructor taking StateSpace associated with iterator.
* \param[in] as 	Pointer to the StateSpace for this iterator.
*/
StateSpaceIter::StateSpaceIter(const StateSpace * ss){
	_ss = ss;
}

/*! \brief Set the iterator to the first element of the set. */
void StateSpaceIter::first(){
	_current = 0;
}

/*! \brief Increment the iterator. */
void StateSpaceIter::next(){
	_current++;
}

/*! \brief Check to see if we are at the last element. */
bool StateSpaceIter::isDone(){
	// Are we at the index after the last index?
	return _current == (_ss->_last + 1);
}

/*! \brief Dereference operator. */
State * StateSpaceIter::operator *(){
	return _ss->_states[_current];
}

/*! \brief Arrow operator. */
State** StateSpaceIter::operator->() const{
	return &(_ss->_states[_current]);
}


///*** StateSpace Factory Class ***///

/*! Default constructor for factory.
*/
SSFactory::SSFactory(){
	_ss = new StateSpace;
}

/*! \brief Sets up the StateSpace object. */
// void SSFactory::buildStateSpace(){
// 	//** Grab the initial state and action space. **//
// 	Configuration * stateConfig = initialState();
// 	_actions = getActions(stateConfig);
// 	//** Add this to the candidate set. **//
// 	_candidateStates.push(new State(stateConfig, _actions));

// 	//** Iterate through all of the candidate states. **//
// 	while(!_candidateStates.isEmpty()){
// 		// Grab the top most state.
// 		State * s = _candidateStates.pop();

// 		//** Get all of the next states for this state. **//
// 		ActionSetIter as(s->getActions());
// 		for(as.first(); as.isDone(); as.next()){
// 			// Next state config.
// 			stateConfig = nextState(s->getConfig(), as->getConfig());
			
// 			// Check to see if it is in the candidate list or the state space.
// 			if(!inSet(stateConfig, &_candidateStates) && !inSet(stateConfig, _ss)){
// 				// If not, then add to the candidate list.
// 				_actions = getActions(stateConfig);
// 				_candidateStates.push(new State(stateConfig, _actions));				
// 			}
// 		}

// 		// Add the expanded state to the state space.
// 		_ss->push(s);
// 	}
// }

/*! Produces the configured StateSpace object.
* \return 	Returns the state space configured for the application.
*/
StateSpace * SSFactory::getStateSpace(){
	return _ss;
}