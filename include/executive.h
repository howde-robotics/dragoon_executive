#include <ros/ros.h>
#include <iostream>
#include "states.h"
#include "dragoon_messages/stateCmd.h"
#include <std_msgs/Int32.h>

class BehavioralExecutive {
	double timer_freq_;
	State oldState;
	State currentState = IDLE_STATE;
	std_msgs::Int32 stateMsg;
	// set up the ros related stuff
	ros::NodeHandle nodeHandle_, privateNodeHandle_;
	ros::Subscriber humanDetectionsSub_, commandsSub_;
	ros::Publisher commandsPub_, statePub_;
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
	void commandsCallback(const dragoon_messages::stateCmdConstPtr stateCmd);

	public:
		BehavioralExecutive();
		~BehavioralExecutive(){}
};