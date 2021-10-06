#include "executive.h"

BehavioralExecutive::BehavioralExecutive() : privateNodeHandle_("~")
{
	this->initRos();

}

void
BehavioralExecutive::run()
{
	ROS_INFO("help");
	switch (currentState)
	{
	case IDLE_STATE:
		runIdle();
		break;

	case EXPLORE_STATE:
		runExplore();
		break;

	case INPUT_STATE:
		runInput();
		break;

	case APPROACH_STATE:
		runApproach();
		break;

	case SWEEP_STATE:
		runSweep();
		break;
	
	default:
		ROS_WARN("Invalid State!");
		break;
	}

}

void
BehavioralExecutive::runIdle()
{
	ROS_INFO("State: IDLE");

	/* Transition to Explore state */
	if (not eventDict[USER_CONTROL] and eventDict[START]) currentState = EXPLORE_STATE;
	 /* Transition to User input state */
	if (eventDict[USER_CONTROL] and eventDict[START]) currentState = INPUT_STATE;
}

void
BehavioralExecutive::runExplore()
{
	ROS_INFO("State: EXPLORE");

	/* Transition stationary sweep */
	if (eventDict[GOAL_REACHED]) currentState = SWEEP_STATE;
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	 /* Transition to IDLE */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

void 
BehavioralExecutive::runInput()
{
	ROS_INFO("State: INPUT");

	/* Transition stationary sweep */
	if (eventDict[GOAL_REACHED]) currentState = SWEEP_STATE;
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	 /* Transition to IDLE */
	if (eventDict[STOP]) currentState = IDLE_STATE;

}

void 
BehavioralExecutive::runApproach()
{
	ROS_INFO("State: APPROACH");

	 /* Transition to explore */
	if (eventDict[GOAL_REACHED] and not eventDict[USER_CONTROL]) currentState = INPUT_STATE;
	 /* Transition to user input */
	if (eventDict[GOAL_REACHED] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;

}

void
BehavioralExecutive::runSweep()
{
	ROS_INFO("State: SWEEP");

	/* Transition to Explore */
	if (eventDict[NO_HUMAN] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to input */
	if (eventDict[NO_HUMAN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;	
}

void
BehavioralExecutive::humanDetectionCallback()
{

}

void 
BehavioralExecutive::commandsCallback()
{

}

void
BehavioralExecutive::initRos()
{
	// humanDetectionsSub_ = nodeHandle_.subscribe("");
	// run the node at specific frequency
	privateNodeHandle_.param<double>("timer_freq_", timer_freq_, 10);
	timer_ = nodeHandle_.createTimer(ros::Rate(timer_freq_), &BehavioralExecutive::timerCallback, this);
}

void
BehavioralExecutive::timerCallback(const ros::TimerEvent &e)
{
	this->run();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dragoon_executive");
	BehavioralExecutive node;
	while (ros::ok())
		ros::spinOnce();
	return 0;
}