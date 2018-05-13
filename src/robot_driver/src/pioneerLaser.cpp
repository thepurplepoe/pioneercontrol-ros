#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <list>
#include <vector>
//#include "obstacle.h"

using namespace std;

float getXOfPoint(float distance, int index) {
	float dist = distance;
	float preang = index/512.0 * M_PI;
	float ang = sin(preang);
	return (dist * ang);
}

float getYOfPoint(float distance, int index) {
	float dist = -1.0 * distance;
	float preang = index/512.0 * M_PI;
	float ang = cos(preang);
	return (dist * ang);
}

float getLength(float x1, float y1, float x2, float y2) {
	float xdif = x2 - x1;
	float ydif = y2 - y1;
	return sqrt((xdif * xdif) + (ydif * ydif));
}

class Obstacle {
	public:
	int obstaclestart;
	int obstacleend;

	vector<float> datapoints;

	Obstacle(int s, int e, vector<float> data);

	float getWidth();
	float getAvgDistance();
	float getAvgAngle();
	float getX();
	float getY();
	int getClosestPointIndex();
	float getRadius();
	float getDim1();
	float getDim2();

	void tellMeThings();
};


Obstacle::Obstacle(int s, int e, vector<float> data) {
	Obstacle::obstaclestart = s;
	Obstacle::obstacleend = e;
	datapoints = data;
}	

float Obstacle::getAvgDistance() {
	float total;
	for (int i = 0; i < Obstacle::datapoints.size(); i++) {
		total += Obstacle::datapoints[i];
	}
	return total/datapoints.size();
}

int Obstacle::getClosestPointIndex() {
	float minsofar = 1000000;
	int min = -1;
	for (int i = 0; i < datapoints.size(); i++) {
		if (datapoints[i] < minsofar) {
			minsofar = datapoints[i];
			min = i;
		}
	}
	return min;
}

float Obstacle::getAvgAngle() {
	float sang = obstaclestart/512.0 * M_PI;
	float eang = obstacleend/512.0 * M_PI;
	return (sang + eang) / 2.0;
}

float Obstacle::getX() {
	//float dist = getAvgDistance();
	//float ang = sin(getAvgAngle());
	//return (dist * ang);
	float x1 = getXOfPoint(Obstacle::datapoints[0], obstaclestart);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float x4 = (x1 + x3) / 2;
	return x4;
}

float Obstacle::getY() {
	//float dist = -1.0 * getAvgDistance();
	//float ang = cos(getAvgAngle());
	//return (dist * ang);
	float y1 = getYOfPoint(Obstacle::datapoints[0], obstaclestart);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float y4 = (y1 + y3) / 2;
	return y4;
}

float Obstacle::getRadius() {
	float x1 = getXOfPoint(Obstacle::datapoints[0], obstaclestart);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float x4 = (x1 + x3) / 2;

	float y1 = getYOfPoint(Obstacle::datapoints[0], obstaclestart);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float y4 = (y1 + y3) / 2;

	float l = getLength(x1, y1, x3, y3);
	float h = getLength(x2, y2, x4, y4);

	float ans = ((4 * h * h) + (l * l)) / (4 * h);
	return ans;
}

float Obstacle::getDim1() {
	float x1 = getXOfPoint(Obstacle::datapoints[0], obstaclestart);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float y1 = getYOfPoint(Obstacle::datapoints[0], obstaclestart);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);

	return getLength(x1, y1, x2, y2);
}

float Obstacle::getDim2() {
	float x1 = getXOfPoint(Obstacle::datapoints[0], obstaclestart);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float y1 = getYOfPoint(Obstacle::datapoints[0], obstaclestart);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);

	return getLength(x2, y2, x3, y3);
}

void Obstacle::tellMeThings() {
	ostringstream s;
	
	vector<float> somegradients = vector<float>();

	vector<float> somex = vector<float>();
	vector<float> somey = vector<float>();

	//s << obstaclestart << "|" << (obstaclestart + getClosestPointIndex()) << "|" << obstacleend;

	/*
	float x1 = getXOfPoint(Obstacle::datapoints[1], obstaclestart+1);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-1)], obstacleend-1);

	float y1 = getYOfPoint(Obstacle::datapoints[1], obstaclestart+1);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-1)], obstacleend-1);

	float grad1 = (y2 - y1) / (x2 - x1);
	float grad2 = (y3 - y2) / (x3 - x2);

	s << grad1 << "|" << grad2;
	*/



	for (int i = 3; i < (Obstacle::datapoints.size() - 7); i++) {
		float gradtot = 0;
		float xofp;
		float yofp;

		for (int j = 1; j < 4; j++) { 	// Go through the next 3 points
			float x1 = getXOfPoint(datapoints[i], obstaclestart + i);
			float x2 = getXOfPoint(datapoints[i + j], obstaclestart + i + j);
			float y1 = getYOfPoint(datapoints[i], obstaclestart + i);
			float y2 = getYOfPoint(datapoints[i + j], obstaclestart + i + j);
			xofp = x1;
			yofp = y1;
			gradtot += ((x2 - x1) / (y2 - y1));
		}
		gradtot /= 3; // Take an average of the gradients to the next 3 points 
		
		somegradients.push_back(gradtot);
		somex.push_back(xofp);
		somey.push_back(yofp);
	}

	bool square = false;

	for (int i = 0; i < (somegradients.size() - 1); i++) {
		if (abs(somegradients[i] - somegradients[i + 1]) > 0.6) {
			square = true;
		}
		
	}

	float x1 = getXOfPoint(Obstacle::datapoints[0], obstaclestart);
	float x2 = getXOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float x3 = getXOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float x4 = (x1 + x3) / 2;

	float y1 = getYOfPoint(Obstacle::datapoints[0], obstaclestart);
	float y2 = getYOfPoint(Obstacle::datapoints[getClosestPointIndex()], obstaclestart + getClosestPointIndex());
	float y3 = getYOfPoint(Obstacle::datapoints[(Obstacle::datapoints.size()-2)], obstacleend-2);
	float y4 = (y1 + y3) / 2;

	s << ((square) ? "SQUARE" : "CIRCLE");
	if (square) {
		s << " Dimensions: " << getDim1() << " x " << getDim2(); 	
	} else {
		s << " Radius: " << getRadius();	
	}
	s << " Y: " << getX();// << "," << x2 << "," << x3;
	s << " X: " << -1.0 * getY();// << "," << y2 << "," << y3;

	if (datapoints.size() < 15) {
		s << "NO";	
	}

	ROS_INFO_STREAM(s.str());
}

geometry_msgs::Twist velocityCommand; 

/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	ostringstream s;
	// Compute the number of data points
	// max angle and min angle of the laser scanner divide by the increment angle of each data point
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
	
	vector<Obstacle> obstacles = vector<Obstacle>();

	float prevrange = 0;
	int currentobstaclestart = 0;
	bool inobstacle = false;

	// Find obstacles and add them to the list
	for(int j = 0; j < rangeDataNum; ++j) // Go through the laser data
	{
		float range = laserScanData->ranges[j];
		float grad = range - prevrange;
		prevrange = range;
		
		if (grad < -0.2 && !inobstacle) {
			currentobstaclestart = j;
			inobstacle = true;
		}

		if (grad > 0.2 && inobstacle) {

			vector<float> obdata = vector<float>();
			for (int k = currentobstaclestart; k < j + 1; k++) {
				obdata.push_back(laserScanData->ranges[k]);
			}
			Obstacle o = Obstacle(currentobstaclestart, j, obdata);
			obstacles.push_back(o);
			inobstacle = false;
		}
	}
	
	s << "Start:: ";

	for (int j = 0; j < obstacles.size(); j++) {
		Obstacle o = obstacles[j];
		o.tellMeThings();
	}

	s << endl;
	ROS_INFO_STREAM(s.str());
	//ros::shutdown();
}

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);

	ros::Rate loop_rate(10);// loop 10 Hz

	while(ros::ok()) // publish the velocity set in the call back
	{
		ros::spinOnce();
		loop_rate.sleep();

		// publish to the twist to the topic
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
