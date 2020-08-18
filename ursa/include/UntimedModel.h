#ifndef _untimed_model_h
#define _untimed_model_h

#include <string>
#include "Model.h"

/**
 * Untimed models represent hardware models whose clock period is irrelevant for
 * the simulation.
 */
class UntimedModel : public Model{

public:

	/** Default Ctor. */
	explicit UntimedModel(std::string name);
	
	/**
	 * @brief Dtor. Must be implemented by subclasses. */
	virtual ~UntimedModel() = 0;
	
	/**
	 * @brief Resets the instance to its starting state. Must be implemented
	 * by subclasses */
	virtual void Reset() = 0;
};

#endif /* _untimed_model_h */
