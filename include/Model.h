#ifndef __MODEL_H
#define __MODEL_H

#include <string>

using namespace std;


/**
 * @class MODEL
 * @author Anderson Domingues
 * @date 08/13/18
 * @file Entity.h
 * @brief Represent some model to be simulated. It is the most 
 * primitive form of model and does not define whether some model
 * is clock-dependable or not. For clock-dependable models, see Discrete
 * class, for clock-independent models, see Continuous class. */
class Model{

private:
	
	/** 
	 * @brief Name of the entity. Usually, names are unique for each 
	 * instance of the class, but no policy is implemented to ensure such behavior. */
	string _name;

public:
	
	/** Default ctor. **/
	Model(string name);
	
	/**
	 * @brief Gets the name of this instance.
	 * @return The name, if any. */
	std::string GetName();
	
	/**
	 * @brief Sets a name for the current model
	 * @param s Name to be set
	 */
	void SetName(std::string s);
	
	
	double GetMetric(std::string metric);
};

#endif /* ENTITY_H */
