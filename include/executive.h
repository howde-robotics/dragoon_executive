#include <unordered_set>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include "states.h"
#include "dragoon_messages/stateCmd.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    ros::Subscriber humanStatesSub_, humanEvidencesSub_, commandsSub_, poseSub_;
    ros::Publisher commandsPub_, statePub_, humanEvidencePoseInBaseLinkPub_, stateTextPub_;
    ros::Timer timer_;
    /* Detection related */
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::TransformStamped dragoonTransform_;
    geometry_msgs::PoseStamped humanEvidencePose_, humanEvidencePoseInBaseLink_, humanPose_;
    double evidenceThreshold_;
    std_msgs::String behaviorStateTextMsg_;
    std::unordered_map<int, geometry_msgs::PoseStamped> detectedHumans_;
    std::string mapFrameName_ = "map";
    std::string robotFrameName_ = "base_link";

    // Approach related
    double timeAligningLimit_ = 5.0; // sec
    double reOrientClosenessThreshold_ = 0.2; // rad
    double reOrientPGain_ = 0.5; // P gain

    // Sweep related
    ros::Subscriber imuSub_, moveBaseCmdVelSub_;
    ros::Publisher outCmdVelPub_;
    bool isSweeping_ = false;
    double sweepSpeed_;	// rad/s
    double sweepDegTurned_ = 0.0;
    double sweepDegTarget_ = M_PI * 2;
    ros::Time lastImuTime_;
    geometry_msgs::Twist currMoveBaseCmdVel_;
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool finishedSweepSleep_ = false;

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
