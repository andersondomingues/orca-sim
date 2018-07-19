#include <Event.h>
#include <Simulator.h>

#include <HFRiscv.h>
#include <MemoryModel.h>
//#include <DmniModel.h>

#define MEM_SIZE  0x00100000
#define SRAM_BASE 0x40000000
#define CYCLES_TO_SIM 100000

int main(int argc, char** argv){

	if(argc < 2){
		std::cout << "Usage: teste <app>.bin" << endl;
		exit(0);
	}

    //creates a new memory
	MemoryModel mem1 = MemoryModel(MEM_SIZE, true);
	
	//a program into it
	mem1.LoadBin(argv[1], 0, MEM_SIZE);

	//creates a new processor
	HFRiscv hfr = HFRiscv("hf001", mem1.GetMemPtr(), MEM_SIZE, SRAM_BASE);	
	
	//start simulation
	Simulator s = Simulator(CYCLES_TO_SIM);
	s.Schedule(Event(0, &hfr));
//s.Schedule(Event(0, &dmni));
	
	s.Run();
	
	//dump mem on screen		
	//MemoryHelper::Dump(mem1, 0, MEM_SIZE);
}
