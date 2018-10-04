#include <string>
using std::string;

class pe {

		int type;
		int master;
		
	public:
		
		pe();
		pe(int typex, int masterx);
		int get_type();
		void set_type(int typex);
		int get_master();
		void set_master(int masterx);
};

pe::pe()
{
}
pe::pe(int typex, int masterx)
{
	type = typex;
	master = masterx;
}
int pe::get_type()
{ 
	return type;
}
void pe::set_type(int typex)
{
	type = typex;
}
int pe::get_master()
{ 
	return master;
}
void pe::set_master(int masterx)
{
	master = masterx;
}


