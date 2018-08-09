#ifndef __PROCESS_H
#define __PROCESS_H

#include <string>

using namespace std;

/**
 * @class Process
 * @author Anderson Domingues
 * @date 08/08/18
 * @file Process.h
 * @brief This class models a process. In this project, a process
 * is an abstraction which can execute an action in a given point
 * in time. For example, hardware can be modeled as processes that 
 * execute cycles given some period. */
class Process{

private:
	
	/** 
	 * @brief Stores a name for this instance. Usually, names 
	 * are unique for each instance of the class, but no policy
	 * is implemented to ensure such behavior. */
	string _name;

public:
	///
	Process(string name);
	string GetName();
	
	/** runs current process and return at what time the next run must be scheduled */
	virtual unsigned long long Run() = 0;
	virtual ~Process() = 0;
};

#endif /* PROCESS_H */
