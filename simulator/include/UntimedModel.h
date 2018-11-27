#ifndef __UNTIMEDMODEL_H
#define __UNTIMEDMODEL_H

#include <string>
#include <Model.h>

using namespace std;

/**
 * @class Opaque
 * @author Anderson Domingues
 * @date 08/13/18
 * @file Opaque.h
 * @brief Opaque class models Entities in which the simulation time
 * doesn't case. For example, combinational parts of some circuit may
 * "instantly" operate without losing cycle precision. */
class UntimedModel : public Model{

public:

	/** Default Ctor. */
	UntimedModel(std::string name);
	
	/**
	 * @brief Dtor. Must be implemented by subclasses. */
	virtual ~UntimedModel() = 0;
	
	/**
	 * @brief Resets the instance to its starting state. Must be implemented
	 * by subclasses */
	virtual void Reset() = 0;
};

#endif /* UNTIMEDMODEL_H */
