#include <string>
using std::string;
#include <list>
using std::list;

class application {

		string name;
		string path;
		string time;
		float  stime;
		
	public:

		list<task> tasks;
		list<task> initTasks;
		application();
		application(string namex, string pathx, string timex, float stimex);
		string get_name();
		string get_path();
		string get_time();
		float get_stime();
		void set_name(string namex);
		void set_path(string pathx);
		void set_time(string timex);
		void set_stime(float stimex);
		
};

application::application()
{
}
application::application(string namex, string pathx, string timex, float stimex)
{
	name = namex;
	path = pathx;
	time = timex;
	stime = stimex;
}
string application::get_name()
{ 
	return name;
}
string application::get_path()
{ 
	return path;
}
string application::get_time()
{ 
	return time;
}
float application::get_stime()
{ 
	return stime;
}
void application::set_name(string namex)
{
	name = namex;
}
void application::set_path(string pathx)
{
	path = pathx;
}
void application::set_time(string timex)
{
	time = timex;
}
void application::set_stime(float stimex)
{
	stime = stimex;
}
