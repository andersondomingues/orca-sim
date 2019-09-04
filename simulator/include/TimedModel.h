#ifndef __TIMEDMODEL_H
#define __TIMEDMODEL_H

//lib dependent includes
#include <string>

//own api includes
#include "Model.h"
#include "SimulationTime.h"

using namespace std;

/**
 * @class TimedModel
 * @author Anderson Domingues
 * @date 08/08/18
 * @file TimedModel.h
 * @brief This class models a TimedModel. In this project, a TimedModel
 * is an abstraction which can execute an action in a given point
 * in time. For example, hardware can be modeled as TimedModeles that 
 * execute cycles given some period. */
class TimedModel : public Model{
	//inherits 
	//string _name;
	
public:
	//inherits 
	//string GetName();

	/** Default Ctor. */
	TimedModel(string name);
	
	/**
	 * @brief Method which is called by the simulator when during the 
	 * execution of the TimedModel. Must be implemented by subclasses.*/
	virtual SimulationTime Run() = 0;
	
	/**
	 * @brief Dtor. Must be implemented by subclasses. */
	virtual ~TimedModel() = 0;
	
	/**
	 * @brief Resets the instance to its starting state. Must be implemented
	 * by subclasses */
	virtual void Reset() = 0;
};

#endif /* TIMEDMODEL_H */
