#include "Model.h"

Model::Model(const std::string name){
	_name = name;
}

std::string Model::GetName(){
	return _name;
}	

void Model::SetName(const std::string name){
	_name = name;
}
