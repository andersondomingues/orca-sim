#include <Metric.h>
		
Metric::Metric(Metrics name){
	_name = name;
	_accumulative = 0;
	_samples = 0;
}
Metric::~Metric(){}

Metrics Metric::GetName(){
	return _name;
}

double Metric::GetAccumulative(){
	return _accumulative;
}

uint32_t Metric::GetSamples(){
	return _samples;
}

void Metric::Sample(double data){
	_samples = _samples + 1;
	_accumulative = _accumulative + data;
}