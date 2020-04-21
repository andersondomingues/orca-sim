#ifndef __TPROCESSOR_BASE_H
#define __TPROCESSOR_BASE_H

#include <TimedModel.h>
#include <UMemory.h>

#define NUMBER_OF_REGISTERS 32

/** Defines a generic state model 
  * for use within processor models. */
template <typename T>
struct ProcessorState{

	T regs[NUMBER_OF_REGISTERS];
	T pc;
	T pc_next;
};

/** This class implements the base operation for 
 * generic processor implementations. This class
 * uses templates to handle the size of internal
 * registers (registers are limited to have the 
 * same size) */
template <typename T> 
class TProcessorBase : public TimedModel{

private:
	struct ProcessorState<T> _state;

	UMemory* _memory;	

public:
	TProcessorBase(std::string name, MemoryAddr initial_pc);
	~TProcessorBase();

	ProcessorState<T>* GetState();
	UMemory* GetMemory();
};

//Some of the most used instances. More can be added later.
template struct ProcessorState<uint8_t>;
template struct ProcessorState<uint16_t>;
template struct ProcessorState<uint32_t>;
template struct ProcessorState<uint64_t>;

//Some of the most used instances. More can be added later.
template class TProcessorBase<uint8_t>;
template class TProcessorBase<uint16_t>;
template class TProcessorBase<uint32_t>;
template class TProcessorBase<uint64_t>;

#endif
