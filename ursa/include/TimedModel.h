#ifndef _timed_model_h
#define _timed_model_h

//own api includes
#include "Model.h"
#include "SimulationTime.h"

/**
 * This class models a TimedModel. In this project, a TimedModel
 * is an abstraction which can execute an action in a given point
 * in time. For example, hardware can be modeled as TimedModeles that 
 * execute cycles given some period. */
class TimedModel : public Model{
	
public:
	/** Default Ctor. */
	explicit TimedModel(std::string name);
	
	/**
	 * Method which is called by the simulator when during the 
	 * execution of the TimedModel. Must be implemented by subclasses.*/
	virtual SimulationTime Run() = 0;
	
	/**
	 * Dtor. Must be implemented by subclasses. */
	virtual ~TimedModel() = 0;
	
	/**
	 * Resets the instance to its starting state. Must be implemented
	 * by subclasses */
	virtual void Reset() = 0;
};

#endif /* _timed_model_h */
