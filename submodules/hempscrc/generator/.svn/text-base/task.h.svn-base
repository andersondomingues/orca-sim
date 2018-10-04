#include <string>
using std::string;
#include <list>
using std::list;

class task {

		string name;
		int id;
		string application;
		int insertion_time;
		int code_size;
		int initial_address;
		int status;
		int x_proc;
		int y_proc;
		
	public:
		
		list<dependence> dependences;
		task();
		task(string namex, string applicationx, int statusx, int x_procx, int y_procx);
		string get_name();
		void set_name(string namex);
		string get_application();
		void set_application(string applicationx);
		int get_insertion_time();
		void set_insertion_time(int insertion_timex); //0 - Static Task, 1 - RunTime Task 
		int get_code_size();
		void set_code_size(int code_sizex);
		int get_initial_address();
		void set_initial_address(int initial_addressx);
		int get_status();
		void set_status(int statusx); //-1 mapped, 0 - repository plasma, 1- repository mblite
		int get_x_proc();
		void set_x_proc(int x_procx);
		int get_y_proc();
		void set_y_proc(int y_procx);
};

task::task()
{
}
task::task(string namex, string applicationx, int statusx, int x_procx, int y_procx)
{
	name = namex;
	application = applicationx;
	status = statusx;
	x_proc = x_procx;
	y_proc = y_procx;
}
string task::get_name()
{ 
	return name;
}
void task::set_name(string namex)
{
	name = namex;
}
string task::get_application()
{ 
	return application;
}
void task::set_application(string applicationx)
{
	application = applicationx;
}
int task::get_insertion_time()
{
	return insertion_time;
}
void task::set_insertion_time(int insertion_timex)
{
	insertion_time = insertion_timex;
}
int task::get_code_size()
{
	return code_size;
}
void task::set_code_size(int code_sizex)
{
	code_size = code_sizex;
}
int task::get_initial_address()
{
	return initial_address;
}
void task::set_initial_address(int initial_addressx)
{
	initial_address = initial_addressx;
}
int task::get_status()
{ 
	return status;
}
void task::set_status(int statusx)
{
	status = statusx;
}
int task::get_x_proc()
{ 
	return x_proc;
}
void task::set_x_proc(int x_procx)
{
	x_proc = x_procx;
}
int task::get_y_proc()
{ 
	return y_proc;
}
void task::set_y_proc(int y_procx)
{
	y_proc = y_procx;
}



