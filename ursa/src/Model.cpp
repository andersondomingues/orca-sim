#include <Model.h>


Model::Model(std::string name){
	_name = name;
}

std::string Model::GetName(){
	return _name;
}	

void Model::SetName(std::string name){
	_name = name;
}
	