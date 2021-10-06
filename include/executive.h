#include <ros/ros.h>
#include <iostream>
#include "states.h"

class BehavioralExecutive {
	double timer_freq_;
	State currentState;
	// set up the ros related stuff
	ros::NodeHandle nodeHandle_, privateNodeHandle_;
	ros::Subscriber humanDetectionsSub_, commandsSub_;
	ros::Publisher commandsPub_;
	ros::Timer timer_;

	void initRos();
	void run();
	void runIdle();
	void runExplore();
	void runInput();
	void runApproach();
	void runSweep();
	void timerCallback(const ros::TimerEvent &e);
	void humanDetectionCallback();
	void commandsCallback();

	public:
		BehavioralExecutive();
		~BehavioralExecutive(){}
};