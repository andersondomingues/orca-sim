#include <Event.h>
#include <Simulator.h>

#include <HFRiscv.h>


int main(int argc, char** argv){

	


	HFRiscv hfr = HFRiscv("teste", ".");
	
	Simulator s = Simulator(10000);
	s.Schedule(Event(0, &hfr));
	
	s.Run();
}