#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <list>

using namespace std;

class Obstacle {
	public:
	int obstaclestart;
	int obstacleend;

	list<float> datapoints;

	Obstacle(int s, int e);

	float getWidth();
	float getAvgDistance();
};

#endif

