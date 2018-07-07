#ifndef __PROCESS_H
#define __PROCESS_H

#include <string>

using namespace std;

class Process{
	
	public:
		Process(string name);
		string name;
		/** runs current process and return what time the next run must be scheduled */
		virtual unsigned long long Run() = 0;
		virtual ~Process() = 0;
};

#endif /* PROCESS_H */
