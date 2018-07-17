#include <Event.h>
#include <Simulator.h>

#include <HFRiscv.h>
#include <MemoryHelper.h>

#define MEM_SIZE  0x00100000
#define SRAM_BASE 0x40000000


int main(int argc, char** argv){

	if(argc < 2){
		std::cout << "Usage: teste <app>.bin" << endl;
		exit(0);
	}

    //creates a new memory and loads a program into it
	MemoryType mem1 = MemoryHelper::Create(MEM_SIZE);
	MemoryHelper::LoadBin(mem1, argv[1], 0, true);	
		
	//instantiate a new processor
	HFRiscv hfr = HFRiscv("hf001", mem1, MEM_SIZE);	
	
	Simulator s = Simulator(100000);
	s.Schedule(Event(0, &hfr));
	
	s.Run();
}