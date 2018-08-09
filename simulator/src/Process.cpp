#include "Process.h"

/**
 * @brief Ctor.
 * @param name An arbitrary name that identifies the instance. 
 */
Process::Process(string name){
	_name = name;
}

/**
 * @brief Dtor.
 */
Process::~Process(){

}

/**
 * @brief Gets the name of the instance which was set during construction.
 * @return The name of the instance.
 */
string Process::GetName(){
	return _name;
}