#ifndef __METRIC_H
#define __METRIC_H

#include <string>

enum class Metrics{
	CYCLE, 
	ENERGY	
};

class Metric{
	
private:
	Metrics _name;
	
	double _accumulative;
	uint32_t _samples;
		
public:

	Metric(Metrics name);
	~Metric();

	double GetAccumulative();
	uint32_t GetSamples();
	void Sample(double data);
	Metrics GetName();
};

#endif /* __OBSERVABLE_MODEL_H */
