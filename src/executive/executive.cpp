#include "executive.h"

BehavioralExecutive::BehavioralExecutive() : privateNodeHandle_("~"), moveBaseClient_("move_base_from_executive", false), tfListener(tfBuffer)
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

	/* Listen for dragoon's current pose */
	try {
		/* TODO: Put in correct things here */
		dragoonTransform_ = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
	}
	catch (tf2::TransformException& ex){
		ROS_WARN("%s", ex.what());
		/* I'd rather not sleep this but the tutorial has one */
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
	resetEvents({START, HUMAN_SEEN, NO_HUMAN});

	/* Runtime of this function actually occurs in the explore package */

	// publish move base cmd vel when running explore
	outCmdVelPub_.publish(currMoveBaseCmdVel_);

	/* Transition stationary sweep */
	if (eventDict[GOAL_REACHED])
	{
		finishedSweepSleep_ = false;
                currentState = SWEEP_STATE;
		sweepDegTurned_ = 0.0;
	}
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
	resetEvents({START, HUMAN_SEEN, NO_HUMAN});

	// publish move base cmd vel when running user input
	outCmdVelPub_.publish(currMoveBaseCmdVel_);

	/* Transition stationary sweep */
	if (eventDict[GOAL_REACHED]) 
	{
		currentState = SWEEP_STATE;
		sweepDegTurned_ = 0.0;
	}
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	 /* Transition to IDLE */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

void 
BehavioralExecutive::runApproach()
{
	std::cout << "\rCurrent State: Approach, human seen deg away" << std::flush;
	/* Reset events */
	resetEvents({NEW_HUMAN, GOAL_REACHED});

	/**
	 * We need dragoon to be orientated facing the last detected evidence
	 * This finds the angular displacement between dragoon and evidence in the correct frame
	 */
	double eviX = humanEvidencePose_.pose.position.x;
	double eviY = humanEvidencePose_.pose.position.y;
	double dragX = dragoonTransform_.transform.translation.x;
	double dragY = dragoonTransform_.transform.translation.y;
	double beta = std::atan2(
		eviY,
		eviX
	);

	double psi = std::atan2(
		dragX,
		dragY
	);

	/* The desired angle to close */
	double alpha = psi - beta;
	/* Time to reach the desired position */
	double time = 5.0;

	geometry_msgs::Twist reOrientCmdVel;
	reOrientCmdVel.angular.z = alpha / time;
	outCmdVelPub_.publish(reOrientCmdVel);

	/* Transition to explore */
	if (eventDict[HUMAN_SEEN] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to user input */
	if (eventDict[HUMAN_SEEN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;
	/* Transition to Idle */
	if (eventDict[STOP]) currentState = IDLE_STATE;
}

void
BehavioralExecutive::runSweep()
{
	std::cout << "\rCurrent State: Sweep  " << sweepDegTurned_ * 180 / M_PI << "deg" << std::flush;
	/* Reset events */
	resetEvents({GOAL_REACHED, HUMAN_SEEN});

	geometry_msgs::Twist sweepCmdVel;

        if (!finishedSweepSleep_)
        {
	  ROS_INFO("Decelerating...");
          sweepCmdVel.angular.x = 0.0;
          sweepCmdVel.angular.z = 0.0;
          outCmdVelPub_.publish(sweepCmdVel);
          ros::Duration(5.0).sleep();
        }

	sweepCmdVel.angular.z = sweepSpeed_;

	// publish sweep cmd vel when running sweep
	outCmdVelPub_.publish(sweepCmdVel);

	if (sweepDegTurned_ > 2 * M_PI)
	{
		sweepDegTurned_ = 0.0;
		eventDict[NO_HUMAN] = true;
	}

	/* Transition to Explore */
	if (eventDict[NO_HUMAN] and not eventDict[USER_CONTROL]) currentState = EXPLORE_STATE;
	 /* Transition to input */
	if (eventDict[NO_HUMAN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;	
	/* Transition to Approach human */
	if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
	/* Transition to Idle */
	if (eventDict[STOP]) currentState = IDLE_STATE;

        finishedSweepSleep_ = true;
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
		// this ID is not in the human ID map yet
		if (detectedHumans_.find(obj.ID) == detectedHumans_.end())
		{
			eventDict[HUMAN_SEEN] = true;
			/* Place the detected human in the map */
			detectedHumans_[obj.ID] = humanPose_;
		}
	}
}

void
BehavioralExecutive::humanEvidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& msg)
{
	/**
	 * This function is going to stop exporing at the first sight of human evidence and exit the state once human is found
	 */
	
	/* Extract message information for evidence location */
	int id = 0;
	for (const wire_msgs::ObjectEvidence& objectEvidence : msg->object_evidence){

		/* Store the most recent evidence position */
		for (auto prop : objectEvidence.properties)
		{
			if (prop.attribute == "position")
			{
				pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);
				const pbl::Gaussian* gauss = getBestGaussian(*pdf, 0);
				if (gauss) {
					const pbl::Vector& mean = gauss->getMean();
					humanEvidencePose_.header.stamp = msg->header.stamp;
					humanEvidencePose_.header.frame_id = "map";
					humanEvidencePose_.pose.position.x = mean(0);
					humanEvidencePose_.pose.position.y = mean(1);
					humanEvidencePose_.pose.position.z = mean(2);
					humanEvidencePose_.pose.orientation.w = 1.0;
				}
			}
		}

		/* To prevent evidence from the same human from constantly switching state back */
		if (not evidenceClose(humanEvidencePose_)) eventDict[NEW_HUMAN] = true;
	}
}

bool 
BehavioralExecutive::evidenceClose(geometry_msgs::PoseStamped& humanEvidence)
{
	/**
	 * Checks if the current evidence is close to any of the previously detected humans
	 * If it is, returns true, else false
	 */

	/* Empty array has no humans in the proximity */
	if (detectedHumans_.empty()) return false;

	/* Otherwise we need to check distance to each human */
	for (auto humanPair : detectedHumans_)
	{
		geometry_msgs::PoseStamped humanPose = humanPair.second;
		/* We only care about x, y position here */
		double eviX = humanEvidence.pose.position.x;
		double eviY = humanEvidence.pose.position.y;
		double detX = humanPose.pose.position.x;
		double detY = humanPose.pose.position.y;
		double dX = detX - eviX;
		double dY = detY - eviY;
		double distance = std::sqrt(dX*dX + dY*dY);

		if (distance < evidenceThreshold_) return true;
	}

	return false;
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

void BehavioralExecutive::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	ros::Duration dt = ros::Time::now() - lastImuTime_;
	sweepDegTurned_ += msg->angular_velocity.z * dt.toSec();
	lastImuTime_ = ros::Time::now();
}

void BehavioralExecutive::moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	currMoveBaseCmdVel_ = *msg;
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
	privateNodeHandle_.param<double>("timer_freq_", timer_freq_, 20);
	privateNodeHandle_.param<double>("sweep_speed", sweepSpeed_, 0.3);
	privateNodeHandle_.param<double>("evidence_threshold", evidenceThreshold_, 0.8);
	timer_ = nodeHandle_.createTimer(ros::Rate(timer_freq_), &BehavioralExecutive::timerCallback, this);
	/* Publishers */
	statePub_ = nodeHandle_.advertise<std_msgs::Int32>("/behavior_state", 1);

	// Sweep related
	imuSub_ = nodeHandle_.subscribe("imu", 1, &BehavioralExecutive::imuCallback, this);
	moveBaseCmdVelSub_ = nodeHandle_.subscribe("cmd_vel_move_base", 1, &BehavioralExecutive::moveBaseCmdVelCallback, this);
	outCmdVelPub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
