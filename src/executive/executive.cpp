#include "executive.h"

BehavioralExecutive::BehavioralExecutive() : privateNodeHandle_("~"), moveBaseClient_("move_base_from_executive", false)
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

	/* Runtime of this function actually occurs in the explore package */

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
	resetEvents({NEW_HUMAN, GOAL_REACHED});

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
	resetEvents({GOAL_REACHED, HUMAN_REACHED});

	/* Transition to Explore */
	if (eventDict[NO_HUMAN] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to input */
	if (eventDict[NO_HUMAN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;	
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	/* Transition to Idle */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

namespace {

static const pbl::Gaussian* getBestGaussian(const pbl::PDF& pdf, double min_weight) {
	if (pdf.type() == pbl::PDF::GAUSSIAN) {
		return pbl::PDFtoGaussian(pdf);
	} else if (pdf.type() == pbl::PDF::MIXTURE) {
		const pbl::Mixture* mix = pbl::PDFtoMixture(pdf);

		if (mix){
			const pbl::Gaussian* G_best = NULL;
			double w_best = min_weight;
			for(int i = 0; i < mix->components(); ++i) {
				const pbl::PDF& pdf = mix->getComponent(i);
				const pbl::Gaussian* G = pbl::PDFtoGaussian(pdf);
				double w = mix->getWeight(i);
				if (G && w > w_best) {
					G_best = G;
					w_best = w;
				}
			}
			return G_best;
		}
	}

	return NULL;
}

}

void
BehavioralExecutive::humanDetectionCallback(const wire_msgs::WorldState::ConstPtr& msg)
{
	for (const wire_msgs::ObjectState& obj : msg->objects) {

		// Create marker
		for (auto prop : obj.properties)
		{
			if (prop.attribute == "position")
			{
				pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);
				const pbl::Gaussian* gauss = getBestGaussian(*pdf, 0);
				if (gauss) {
					const pbl::Vector& mean = gauss->getMean();
					humanPose_.header.stamp = msg->header.stamp;
					humanPose_.header.frame_id = "map";
					humanPose_.pose.position.x = mean(0);
					humanPose_.pose.position.y = mean(1);
					humanPose_.pose.position.z = mean(2);
					humanPose_.pose.orientation.w = 1.0;
				}
			}
		}
		// this ID is not in the human ID set yet
		if (humanIds_.find(obj.ID) == humanIds_.end())
		{
			eventDict[NEW_HUMAN] = true;
			humanIds_.insert(obj.ID);
		}
	}
	/* This needs to subscribe to a human detection and then calculate the path to the human */
}

void
BehavioralExecutive::approachDoneCallback()
{

}

void
BehavioralExecutive::approachActiveCallback()
{
	/* Set the current dragoon distance position */

}

void
BehavioralExecutive::approachFeedbackCallback()
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
	/* TODO: Fill in the human detection */
	humanDetectionsSub_ = nodeHandle_.subscribe("/world_state", 1, &BehavioralExecutive::humanDetectionCallback, this);
	commandsSub_ = nodeHandle_.subscribe("commands", 1, &BehavioralExecutive::commandsCallback, this);
	/* TODO: subscirbe to the current pose from TF or something */
	// poseSub_ = nodeHandle_.subscribe("")
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