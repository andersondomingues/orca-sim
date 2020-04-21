#include <TimedModel.h>
#include <TProcessorBase.h>

template <typename T> 
TProcessorBase<T>::TProcessorBase(std::string name, MemoryAddr initial_pc) : TimedModel(name) {

	//reset registers
	for(int i = 0; i < NUMBER_OF_REGISTERS; i++)
		_state.regs[i] = 0;

	//reset PC
	_state.pc = initial_pc;
	_state.pc_next = _state.pc + sizeof(T);
}

template <typename T> 
TProcessorBase<T>::~TProcessorBase(){
	//last destructor to
	//be called, no elements 
	//bu the state to get rid
}

/**
 * Access the current state of the processor.
 * @return A pointer to the state of the processor.
 */
template <typename T> 
ProcessorState<T>* TProcessorBase<T>::GetState(){
	return &_state;
}

/**
 * Access the current memory interface of the processor.
 * @returns A pointer to the memory model of the processor.
 */
template <typename T> 
UMemory* TProcessorBase<T>::GetMemory(){
	return _memory;	
}


