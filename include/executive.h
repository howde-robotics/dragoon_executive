#include <ros/ros.h>
#include <iostream>
#include "states.h"
#include "dragoon_messages/stateCmd.h"
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
// #include "wire_msgs/WorldEvidence.h"
// #include "wire_msgs/WorldState"

class BehavioralExecutive {
	double timer_freq_;
	State oldState;
	State currentState = IDLE_STATE;
	std_msgs::Int32 stateMsg;
	/* Pose of a human */
	geometry_msgs::PoseStamped humanPose_;
	geometry_msgs::PoseStamped approachHumanGoal_;
	// set up the ros related stuff
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> moveBaseClient_;
	ros::NodeHandle nodeHandle_, privateNodeHandle_;
	ros::Subscriber humanDetectionsSub_, commandsSub_, poseSub_;
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
	void approachActiveCallback();
	void approachFeedbackCallback();
	void approachDoneCallback();

	public:
		BehavioralExecutive();
		~BehavioralExecutive(){}
};