#include <TimedModel.h>
#include <TProcessorBase.h>

#ifdef ORCA_ENABLE_GDBRSP
template <typename T> 
uint32_t TProcessorBase<T>::GDBSERVER_PORT = ORCA_GDBRSP_PORT;
#endif

template <typename T> 
TProcessorBase<T>::TProcessorBase(std::string name, MemoryAddr initial_pc) : TimedModel(name) {

	//reset registers
	for(int i = 0; i < NUMBER_OF_REGISTERS; i++)
		_state.regs[i] = 0;

	//reset PC
	_state.pc = initial_pc;
	_state.pc_next = _state.pc + sizeof(T);

	#ifdef ORCA_ENABLE_GDBRSP
	_gdbserver = new RspServer<T>("127.0.0.1", GDBSERVER_PORT++);
	#endif
}

template <typename T> 
TProcessorBase<T>::~TProcessorBase(){
	#ifdef ORCA_ENABLE_GDBRSP
	delete _gdbserver;
	#endif
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
 * Run method. Processors must either (i) run one 
 * instruction and return the number of cycles that 
 * the instruction took to leave the pipeline, or
 * (ii) execute one cycle and return 1.
 */
template <typename T>
SimulationTime TProcessorBase<T>::Run(){

	#ifdef ORCA_ENABLE_GDBRSP
	return _gdbserver->Receive(&_state);
	#else
	return 0;
	#endif
}
