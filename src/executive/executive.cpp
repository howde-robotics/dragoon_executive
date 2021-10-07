#include "executive.h"

BehavioralExecutive::BehavioralExecutive() : privateNodeHandle_("~")
{
	this->initRos();

}

void
BehavioralExecutive::run()
{
	/* Stores the old state before running the current one */
	oldState = currentState;

	/* Run whatever state we're in */
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
		ROS_ERROR("Invalid State!");
		break;
	}

	/* Only publish the state when it changes */
	if (oldState != currentState)
	{	
		stateMsg.data = currentState;
		statePub_.publish(stateMsg);
	}
}

void
BehavioralExecutive::runIdle()
{
	std::cout << "\rCurrent State: Idle" << std::flush;
	/* Reset events */
	resetEvents(STOP);

	/* Transition to Explore state */
	if (not eventDict[USER_CONTROL] and eventDict[START]) currentState = EXPLORE_STATE;
	 /* Transition to User input state */
	if (eventDict[USER_CONTROL] and eventDict[START]) currentState = INPUT_STATE;
}

void
BehavioralExecutive::runExplore()
{
	std::cout << "\rCurrent State: Explore "<< std::flush;
	/* Reset events */
	resetEvents({START, HUMAN_REACHED, NO_HUMAN});

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
	std::cout << "\rCurrent State: User Input" << std::flush;
	/* Reset events. Note that we do not reset the 'USER_CONTROL' state. This needs to be set manually*/
	resetEvents({START, HUMAN_REACHED, NO_HUMAN});

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
	std::cout << "\rCurrent State: Approach" << std::flush;
	/* Reset events */
	resetEvents(NEW_HUMAN);

	 /* Transition to explore */
	if (eventDict[HUMAN_REACHED] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to user input */
	if (eventDict[HUMAN_REACHED] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;
	/* Transition to Idle */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

void
BehavioralExecutive::runSweep()
{
	std::cout << "\rCurrent State: Sweep  " << std::flush;
	/* Reset events */
	resetEvents(GOAL_REACHED);

	/* Transition to Explore */
	if (eventDict[NO_HUMAN] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to input */
	if (eventDict[NO_HUMAN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;	
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	/* Transition to Idle */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

void
BehavioralExecutive::humanDetectionCallback()
{

}

void 
BehavioralExecutive::commandsCallback(const dragoon_messages::stateCmdConstPtr stateCmd)
{
	std::string event = stateCmd->event;
	bool value = stateCmd->value;
	 /* Set the event correctly */
	eventDict[event] = value;
	std::cout << std::endl;
	ROS_INFO_STREAM("Received event: " << event << ". Set to: " << value);
}

void
BehavioralExecutive::initRos()
{
	// humanDetectionsSub_ = nodeHandle_.subscribe("");
	commandsSub_ = nodeHandle_.subscribe("commands", 1, &BehavioralExecutive::commandsCallback, this);
	// run the node at specific frequency
	privateNodeHandle_.param<double>("timer_freq_", timer_freq_, 10);
	timer_ = nodeHandle_.createTimer(ros::Rate(timer_freq_), &BehavioralExecutive::timerCallback, this);
	/* Publishers */
	statePub_ = nodeHandle_.advertise<std_msgs::Int32>("/behavior_state", 1);
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