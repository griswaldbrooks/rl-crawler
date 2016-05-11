// Library for creating state objects.

#ifndef RLLIB_STATE_H
#define RLLIB_STATE_H

#define DEFAULT_STATE_SPACE_SIZE	25
#define DEFAULT_ACTION_SET_SIZE		4
#define DEFAULT_CONFIGURATION_SET_SIZE 	10

// Declaration of State class.
class State;

/*! \brief Configuration class. */
/**
* This class is for creating system state or configurations that the 
* system would actually be in. For example, for a arm, it might be a set
* of joint angles. 
*/

class Configuration{
public:
	/*! \brief Accept a configuration to do a comparision on.
	* \param[in] c 		The configuration to compare with this one.
	* \return 			If the comparison was sucessful.
	*/
	// virtual bool doComparison(Configuration * c) = 0;

	/*! \brief Do the comparison on the configuration.
	* \param[in] c 		The configuration to compare.
	* \return 			If the configuration passed in is equal to this one.
	*/
	virtual bool doComparison(Configuration * c) = 0;
	virtual bool Compare(Configuration * c) = 0;
};

/*! \brief ConfigurationSet class. */
/**
* This is an aggregate class for containing a set of Configuration objects.
*/

class ConfigurationSet{
public:
	/*! \brief Default constructor for the set. */
	ConfigurationSet();

	/*! \brief Constructor for the set with known size 
	* \param[in] n 		The number of configurations in the set.
	*/
	ConfigurationSet(int n);

	/*! \brief Add Configuration to the set. 
	* \param[in] c	Configuration to be added to the set.
	*/
	void push(Configuration * c);

	/*! \brief Remove Configuration from the set. 
	* \return		Configuration object removed from the set.
	*/
	Configuration * pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Configurations are in this set?
	* \return 		Returns the number of Configurations in the set.
	*/
	int size();	

private:
	Configuration ** _configs; 	/**< Configurations held. */
	int _last; 					/**< Index of the current Configuration being examined. */
	int _capacity; 				/**< Number of elements in the array */
};

/*! \brief Action class. */
/**
* This class is for creating actions that can be used with
* different Reinforcement Learning Algorithms.
*/

class Action{
public:
	/*! \brief Default constructor
	*/
	Action();

	/*! \brief Constructor for a given configuration.
	*/
	Action(Configuration * a);

	/*! \brief Updates the reward through an average. 
	* \param[in] r 	The reward value to incorporate.
	*/
	void updateReward(float r);

	/*! \brief Sets the reward value directly.
	* \param[in] r 	The value to set the reward to.
	*/
	void setReward(float r);

	/*! \brief Returns the current reward.
	* \return 		The value of the current reward for that action.
	*/
	float getReward();

	/*! \brief Change the action. 
	* \param[in] a 	The action to do in the configuration space.
	*/
	void setConfig(Configuration * a);

	/*! \brief Get the action. 
	* \return 		The action to do in the configuration space..
	*/
	Configuration * getConfig();

	/*! \brief Set next state. 
	* \param[in] s 	Pointer to the state reached for doing this action (from the parent state).
	*/
	void setNextState(State * s);

	/*! \brief Get next state. 
	* \return  	Pointer to the state reached for doing this action (from the parent state).
	*/
	State * getNextState();

private:
	State * _state; 			/**< State reached by doing this action. Implicitly the is dependent on the state that holds this action. */
	Configuration * _action; 	/**< Movement to do in the configuration space. */
	float _reward; 				/**< The reward for doing that action. */

};

// Declaration of ActionSet iterator.
class ActionSetIter;

/*! \brief ActionSet class. */
/**
* This is an aggregate class for containing a set of Action objects.
*/

class ActionSet{
public:
	/*! \brief Default constructor for the set. */
	ActionSet();

	/*! \brief Constructor for the set with known size 
	* \param[in] n 		The number of actions in the set.
	*/
	ActionSet(int n);

	/*! \brief Default destructor for the set. */
	~ActionSet();

	/*! \brief Add Action to the set. 
	* \param[in] a	Action to be added to the set.
	*/
	void push(Action a);

	/*! \brief Remove Action from the set. 
	* \return		Action object removed from the set.
	*/
	Action pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Actions are in this set?
	* \return 		Returns the number of Actions in the set.
	*/
	int size();	

	/*! \brief Declare iterator class as friend. */
	friend class ActionSetIter;

	/*! \brief Create iterator for set. */
	ActionSetIter * createIter() const;

private:
	Action * _actions; 	/**< Actions held. */
	int _last; 			/**< Index of the last Configuration in the set. */
	int _capacity; 		/**< Number of elements in the array */
};

/*! \brief Iterator class for ActionSet. */
class ActionSetIter{
public:
	/*! \brief Constructor taking ActionSet associated with iterator.
	* \param[in] as 	Pointer to the ActionSet for this iterator.
	*/
	ActionSetIter(const ActionSet * as);

	/*! \brief Set the iterator to the first element of the set. */
	void first();

	/*! \brief Increment the iterator. */
	void next();

	/*! \brief Check to see if we are at the last element. */
	bool isDone();

	/*! \brief Dereference operator. */
	Action operator *();

	/*! \brief Arrow operator. */
	Action* operator->() const;

private:
	const ActionSet * _as; 	/**< Pointer to the ActionSet for this iterator. */
	int _current; 				/**< Index of the current element. */
};

/*! \brief State class. */
/**
* This class is for creating states that can be used with
* different Reinforcement Learning Algorithms.
*/

class State{
public:
	/*! \brief Default constructor.
	*/
	State();

	/*! \brief Constructor for known action set size.
	* \param[in] n 		The number of actions for the state.
	*/
	State(int n);

	/*! \brief Constructor with Configuration state and Action set.
	* \param[in] state 		The Configuration for that state.
	* \param[in] actions	The ConfigurationSet describing the actions in that state.
	*/
	State(Configuration * state, ConfigurationSet * actions);

	/*! \brief Default destructor. */
	~State();

	/*! \brief Set the state configuration. 
	* \param[in] config 	Configuration to set the State to.
	*/
	void setConfig(Configuration * config);

	/*! \brief Method to get system configuration for this state. 
	* \return 				The Configuration of this state.
	*/
	Configuration * getConfig();

	/*! \brief Get the ActionSet for this State.
	* \return 				Pointer to the ActionSet that belongs to this State.
	*/
	ActionSet * getActions();

private:
	Configuration * _config; 	/**< The physical state of the system. */
	
	float _value, _pvalue; 	/**< The current and previous values of the state. */
	int _policy, _ppolicy; 	/**< The current and previous policy to do in that state, represented by indices for the action set. */

	ActionSet * _actions;	/**< The different actions that can be done in that state. */	
};

// Declaration of the StateSpace itereator.
class StateSpaceIter;

/*! \brief StateSpace class. */
/**
* This class is for State Spaces that can be used with
* different Reinforcement Learning Algorithms.
*/

class StateSpace{
public:
	/*! \brief Default constructor
	*/
	StateSpace();

	/*! \brief Constructor for known number of states.
	* \param[in] n 		Number of system states.
	*/
	StateSpace(int n);

	/*! \brief Add State to the set. 
	* \param[in] s	Pointer to the State to be added to the set.
	*/
	void push(State * s);

	/*! \brief Remove State from the set. 
	* \return		Pointer to the State object removed from the set.
	*/
	State * pop();

	/*! \brief Are there any objects in the set? 
	* \return 		Returns true if there is anything in the set.
	*/
	bool isEmpty();

	/*! \brief How many Configurations are in this set?
	* \return 		Returns the number of Configurations in the set.
	*/
	int size();	

	/*! \brief Declare iterator class as friend. */
	friend class StateSpaceIter;

	/*! \brief Create iterator for set. */
	StateSpaceIter * createIter() const;

private:
	State ** _states; 	/**< Pointer to the memory space that hold the value map. */
	int _last; 			/**< Index of the last state in the set. */
	int _capacity; 		/**< Number of elements in the array. */
};

/*! \brief Iterator class for StateSpace. */
class StateSpaceIter{
public:
	/*! \brief Constructor taking StateSpace associated with iterator.
	* \param[in] as 	Pointer to the StateSpace for this iterator.
	*/
	StateSpaceIter(const StateSpace * ss);

	/*! \brief Set the iterator to the first element of the set. */
	void first();

	/*! \brief Increment the iterator. */
	void next();

	/*! \brief Check to see if we are at the last element. */
	bool isDone();

	/*! \brief Dereference operator. */
	State * operator *();

	/*! \brief Arrow operator. */
	State** operator->() const;

private:
	const StateSpace * _ss; 	/**< Pointer to the ActionSet for this iterator. */
	int _current; 				/**< Index of the current element. */
};

/*! \brief Factory class for constructing the state space for the application. */
class SSFactory{


private:
	/*! \brief Check to see if a state configuration is in a set. 
	* \param[in] state 		State Configuration to look for.
	* \param[in] set 		Set to check.
	*/
	template<typename ConfigType>
	bool inSet(ConfigType * state, StateSpace * set){
		// Iterate through the set and check for matching configurations.
		StateSpaceIter ss(set);
		for(ss.first(); ss.isDone(); ss.next()){
			// if((*ss)->getConfig() == state){ // <---------- This is wrong. I need to compare values, not pointers!!!!!!!!!!!!
			// 	// Found one!
			// 	return true;
			// }
		}

		// Didn't find one.
		return false;
	}

	/*! \brief Get the next configuration state in the configuration space. 
	* \param[in] 	state 	The current state that the action is performed in.
	* \param[in]	action 	The action performed in that state.
	* \return 				The next state in the configuration space.
	*/
	virtual Configuration * nextState(Configuration * state, Configuration * action) = 0;

	/*! \brief Get the initial configuration state in the configuration space. 
	* \return 				The first state in the system.
	*/
	virtual Configuration * initialState() = 0;

	/*! \brief Get the action set for that state.
	* \param[in] 	state 	The system state.
	* \return 				Pointer to the set of actions that can be performed in that state.
	*/
	virtual ConfigurationSet * getActions(Configuration * state) = 0;

	StateSpace * _ss;   /**< State space object being worked on. */
	StateSpace _candidateStates; /**< States queued during state space exploration. */
	ConfigurationSet * _actions; /**< Place to hold the action set for a given state. */

public:
	/*! \brief Default constructor for factory.
	*/
	SSFactory();

	/*! \brief Sets up the StateSpace object according to the crawler configuration. */
	template<typename ConfigType>
	void buildStateSpace(){
	//** Grab the initial state and action space. **//
	ConfigType * stateConfig = initialState();
	_actions = getActions(stateConfig);
	//** Add this to the candidate set. **//
	_candidateStates.push(new State(stateConfig, _actions));

	//** Iterate through all of the candidate states. **//
	while(!_candidateStates.isEmpty()){
		// Grab the top most state.
		State * s = _candidateStates.pop();

		//** Get all of the next states for this state. **//
		ActionSetIter as(s->getActions());
		for(as.first(); as.isDone(); as.next()){
			// Next state config.
			stateConfig = nextState(s->getConfig(), as->getConfig());
			
			// Check to see if it is in the candidate list or the state space.
			if(!inSet<ConfigType>(stateConfig, &_candidateStates) && !inSet<ConfigType>(stateConfig, _ss)){
				// If not, then add to the candidate list.
				_actions = getActions(stateConfig);
				_candidateStates.push(new State(stateConfig, _actions));				
			}
		}

		// Add the expanded state to the state space.
		_ss->push(s);
	}
}

	/*! \brief Produces the configured StateSpace object.
	* \return 	Returns the state space configured for the application.
	*/
	StateSpace * getStateSpace();
};


// class ConfigurationFactory
// {
// public:
// 	Configuration *makeArmConfiguration()
// 	{
// 		 Configuration * c new ArmConfiguration();
// 		 // insert c and 1 into configurations and types
// 		 return c;
// 	}
// 	Configuration *makeActionConfiguration()
// 		{
// 		 Configuration * c = new ActionConfiguration();
// 		 // insert c and 2 into configurations and types
// 		 return c;
// 	}
// 	bool compare(Configuration *p1, Configuration *p2)
// 	{
// 		// get types for p1 and p2
// 		if (t1 == t2)
// 		{
// 			if (t1 == 1) //

// 		}
// 		return false;
// 	}
// 	Configuration ** configurations;
// 	int * types;
// };
#endif