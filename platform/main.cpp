#include <Event.h>
#include <Simulator.h>

#include <HFRiscv.h>
#include <MemoryModel.h>

#define MEM_SIZE  0x00100000
#define SRAM_BASE 0x40000000


int main(int argc, char** argv){

	if(argc < 2){
		std::cout << "Usage: teste <app>.bin" << endl;
		exit(0);
	}

    //creates a new memory and loads a program into it
	MemoryType mem1 = MemoryHelper::Create(MEM_SIZE, true);
	MemoryHelper::LoadBin(mem1, argv[1], 0, MEM_SIZE);

	//dump mem on screen		
	MemoryHelper::Dump(mem1, 0, MEM_SIZE);
		
	//new processor
	HFRiscv hfr = HFRiscv("hf001", mem1, MEM_SIZE, SRAM_BASE);	
	
	//start simulation
	Simulator s = Simulator(100000);
	s.Schedule(Event(0, &hfr));
	
	s.Run();
	
	//dump mem on screen		
	MemoryHelper::Dump(mem1, 0, MEM_SIZE);
}