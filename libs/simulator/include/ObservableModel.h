#ifndef __OBSERVABLE_MODEL_H
#define __OBSERVABLE_MODEL_H

#include <string>
#include <Model.h>
#include <Metric.h>

using namespace std;


class ObservableModel{
	
	
public:

	/**
	 * @brief Must be implemented by subclasses. */
	Metric* GetMetric(Metrics m);
};

#endif /* __OBSERVABLE_MODEL_H */
