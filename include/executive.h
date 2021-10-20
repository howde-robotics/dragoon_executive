#include <unordered_set>
#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include "states.h"
#include "dragoon_messages/stateCmd.h"
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>

// wire stuff
#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/WorldState.h"
#include "problib/conversions.h"

class BehavioralExecutive {
	double timer_freq_;
	State oldState;
	State currentState = IDLE_STATE;
	std_msgs::Int32 stateMsg;
	// set up the ros related stuff
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> moveBaseClient_;
	ros::NodeHandle nodeHandle_, privateNodeHandle_;
	ros::Subscriber humanDetectionsSub_, commandsSub_, poseSub_;
	ros::Publisher commandsPub_, statePub_;
	ros::Timer timer_;
	/* Detection related */
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	geometry_msgs::PoseStamped humanPose_;
	geometry_msgs::TransformStamped dragoonTransform_;
	geometry_msgs::PoseStamped humanEvidencePose_;
	double evidenceThreshold_;
	std::unordered_map<int, geometry_msgs::PoseStamped> detectedHumans_;

	// Sweep related
	ros::Subscriber imuSub_, moveBaseCmdVelSub_;
	ros::Publisher outCmdVelPub_;
	double sweepSpeed_;	// rad/s
	double sweepDegTurned_ = 0.0;
	ros::Time lastImuTime_;
	geometry_msgs::Twist currMoveBaseCmdVel_;
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

	void initRos();
	void run();
	void runIdle();
	void runExplore();
	void runInput();
	void runApproach();
	void runSweep();
	void timerCallback(const ros::TimerEvent &e);
	void humanEvidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& msg);
	bool evidenceClose(geometry_msgs::PoseStamped& human);
	void humanDetectionCallback(const wire_msgs::WorldState::ConstPtr& msg);
	void commandsCallback(const dragoon_messages::stateCmdConstPtr stateCmd);
	void approachActiveCallback();
	void approachFeedbackCallback();
	void approachDoneCallback();

	public:
		BehavioralExecutive();
		~BehavioralExecutive(){}
};