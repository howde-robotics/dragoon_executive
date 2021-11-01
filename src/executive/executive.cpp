#include "executive.h"

BehavioralExecutive::BehavioralExecutive() : privateNodeHandle_("~"), moveBaseClient_("move_base_from_executive", false), tfListener(tfBuffer)
{
    this->initRos();

    // To ensure that tfBuffer has a frame to look at even before any detection exist;
    humanEvidencePose_.header.frame_id = mapFrameName_;
    humanEvidencePoseInBaseLink_.header.frame_id = robotFrameName_;
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
    // std::cout << "\rCurrent State: Idle" << std::flush;
    behaviorStateTextMsg_.data = "Idle";
    stateTextPub_.publish(behaviorStateTextMsg_);
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
    // std::cout << "\rCurrent State: Explore "<< std::flush;
    behaviorStateTextMsg_.data = "Explore";
    stateTextPub_.publish(behaviorStateTextMsg_);
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
        isSweeping_ = true;
    }
    /* Transition to Approach human */
    if (eventDict[NEW_HUMAN]) currentState = APPROACH_STATE;
     /* Transition to IDLE */
    if (eventDict[STOP]) currentState = IDLE_STATE;
}

void 
BehavioralExecutive::runInput()
{
    // std::cout << "\rCurrent State: User Input" << std::flush;
    behaviorStateTextMsg_.data = "User Input";
    stateTextPub_.publish(behaviorStateTextMsg_);
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
    // std::cout << "\rCurrent State: Approach, human seen deg away" << std::flush;
    behaviorStateTextMsg_.data = "Approach";
    stateTextPub_.publish(behaviorStateTextMsg_);
    /* Reset events */
    resetEvents({NEW_HUMAN, GOAL_REACHED});

    // Lock on to the discovered human evidence so that this pose is not further updated
    const geometry_msgs::PoseStamped humanEvidencePoseLocked = humanEvidencePose_;

    geometry_msgs::Twist StopCmdVel;
    behaviorStateTextMsg_.data = "Decelerating in Approach";
    stateTextPub_.publish(behaviorStateTextMsg_);
    StopCmdVel.angular.x = 0.0;
    StopCmdVel.angular.z = 0.0;
    outCmdVelPub_.publish(StopCmdVel);
    ros::Duration(2.0).sleep();

    // Trying to align
    behaviorStateTextMsg_.data = "Aligning to detected human";
    stateTextPub_.publish(behaviorStateTextMsg_);
    ros::Time startApproachAlignmentTime = ros::Time::now();
    while (true)
    {
        // Cannot spend more than timeAligningLimit on this task
        ros::Duration timeSpentAligning = ros::Time::now() - startApproachAlignmentTime;
        if (timeSpentAligning.toSec() > timeAligningLimit_)
        {
            break;
        }

        /* Listen for dragoon's current pose */
        try {
            /* TODO: Put in correct things here */
            dragoonTransform_ = tfBuffer.lookupTransform(robotFrameName_, mapFrameName_, ros::Time(0));
        }
        catch (tf2::TransformException& ex){
            ROS_WARN("%s", ex.what());
            return;
        }

        tf2::doTransform(humanEvidencePoseLocked, humanEvidencePoseInBaseLink_, dragoonTransform_);

        humanEvidencePoseInBaseLinkPub_.publish(humanEvidencePoseInBaseLink_);

        double beta = std::atan2(
            humanEvidencePoseInBaseLink_.pose.position.y,
            humanEvidencePoseInBaseLink_.pose.position.x
        );

        // if dragoon orientation is close enough to human, stop
        if (std::fabs(beta) < reOrientClosenessThreshold_)
        {
            break;
        }

        geometry_msgs::Twist reOrientCmdVel;
    const double orientLimit = 0.5;
        reOrientCmdVel.angular.z = std::min(std::max(beta * reOrientPGain_, -orientLimit), orientLimit);
        outCmdVelPub_.publish(reOrientCmdVel);
    
        // small sleep to prevent CPU overload
        ros::Duration(0.05).sleep();
    }

    behaviorStateTextMsg_.data = "Finished Aligning";
    stateTextPub_.publish(behaviorStateTextMsg_);
    StopCmdVel.angular.x = 0.0;
    StopCmdVel.angular.z = 0.0;
    outCmdVelPub_.publish(StopCmdVel);
    ros::Duration(2.0).sleep();
    eventDict[HUMAN_SEEN] = true;

    /* Transition to Sweep */
    if (eventDict[HUMAN_SEEN] and not eventDict[USER_CONTROL] and isSweeping_) 
        currentState = SWEEP_STATE;
    /* Transition to explore */
    if (eventDict[HUMAN_SEEN] and not eventDict[USER_CONTROL] and not isSweeping_) 
        currentState = EXPLORE_STATE;
     /* Transition to user input */
    if (eventDict[HUMAN_SEEN] and eventDict[USER_CONTROL]) currentState = INPUT_STATE;
    /* Transition to Idle */
    if (eventDict[STOP]) currentState = IDLE_STATE;
}

void
BehavioralExecutive::runSweep()
{
    // std::cout << "\rCurrent State: Sweep  " << sweepDegTurned_ * 180 / M_PI << "deg" << std::flush;
    std::stringstream stream;
    // execute the last sweep before shutdown flag
    bool lastSweep = false;
    stream << std::fixed << std::setprecision(1) << sweepDegTurned_ * 180 / M_PI;
    behaviorStateTextMsg_.data = "Sweep " + stream.str() + " deg";
    stateTextPub_.publish(behaviorStateTextMsg_);
    /* Reset events */
    resetEvents({GOAL_REACHED, HUMAN_SEEN});

    geometry_msgs::Twist sweepCmdVel;

    if (!finishedSweepSleep_)
    {
        behaviorStateTextMsg_.data = "Decelerating to Sweep";
        stateTextPub_.publish(behaviorStateTextMsg_);
        sweepCmdVel.angular.x = 0.0;
        sweepCmdVel.angular.z = 0.0;
        outCmdVelPub_.publish(sweepCmdVel);
        ros::Duration(2.0).sleep();
    }

    sweepCmdVel.angular.z = sweepSpeed_;

    // publish sweep cmd vel when running sweep
    outCmdVelPub_.publish(sweepCmdVel);

    if (sweepDegTurned_ > sweepDegTarget_)
    {
        sweepDegTurned_ = 0.0;
        isSweeping_ = false;
        eventDict[NO_HUMAN] = true;
    }

    /* Transition to Explore */
    if (eventDict[NO_HUMAN] and not eventDict[USER_CONTROL])
    {
        ros::Duration(2.0).sleep();
        currentState = EXPLORE_STATE;
    }
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
    detectedHumans_.clear();
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
                    humanPose_.header.frame_id = mapFrameName_;
                    humanPose_.pose.position.x = mean(0);
                    humanPose_.pose.position.y = mean(1);
                    humanPose_.pose.position.z = mean(2);
                    humanPose_.pose.orientation.w = 1.0;
                }
            }
        }
        detectedHumans_[obj.ID] = humanPose_;


        // // this ID is not in the human ID map yet
        // if (detectedHumans_.find(obj.ID) == detectedHumans_.end())
        // {
        // 	// eventDict[HUMAN_SEEN] = true;
        // 	/* Place the detected human in the map */
        // 	detectedHumans_[obj.ID] = humanPose_;
        // }
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

    // bool noNewHuman = true;
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
                    humanEvidencePose_.header.frame_id = mapFrameName_;
                    humanEvidencePose_.pose.position.x = mean(0);
                    humanEvidencePose_.pose.position.y = mean(1);
                    humanEvidencePose_.pose.position.z = mean(2);
                    humanEvidencePose_.pose.orientation.w = 1.0;
                }
            }
        }

        /* To prevent evidence from the same human from constantly switching state back */
        if (not evidenceClose(humanEvidencePose_)) 
        {
            eventDict[NEW_HUMAN] = true;
            // noNewHuman = false;
            // break;
        }
    }
    // if (noNewHuman)
    // {
    // 	eventDict[HUMAN_SEEN] = true;
    // }
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
    // if not in sweep or approach, skip this
    if (currentState == SWEEP_STATE or currentState == APPROACH_STATE)
    {
        // Calculate degress turned from imu
        ros::Duration dt = ros::Time::now() - lastImuTime_;
        if (dt.toSec() > 0.2)
        {
            ROS_WARN("IMU in executive is too old");
        } else
        {
            sweepDegTurned_ += msg->angular_velocity.z * dt.toSec();
        }
    }
    lastImuTime_ = ros::Time::now();
}

void BehavioralExecutive::moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    currMoveBaseCmdVel_ = *msg;
}

void
BehavioralExecutive::initRos()
{
    humanStatesSub_ = nodeHandle_.subscribe("/world_state", 1, &BehavioralExecutive::humanDetectionCallback, this);
    humanEvidencesSub_ = nodeHandle_.subscribe("/world_evidence", 1, &BehavioralExecutive::humanEvidenceCallback, this);
    commandsSub_ = nodeHandle_.subscribe("commands", 1, &BehavioralExecutive::commandsCallback, this);
    // run the node at specific frequency
    privateNodeHandle_.param<double>("timer_freq_", timer_freq_, 20);
    privateNodeHandle_.param<double>("sweep_speed", sweepSpeed_, 0.3);
    privateNodeHandle_.param<double>("evidence_threshold", evidenceThreshold_, 1.0);
    privateNodeHandle_.param<double>("sweep_deg_target", sweepDegTarget_, M_PI * 2);
    privateNodeHandle_.param<double>("time_aligning_limit", timeAligningLimit_, 5.0);
    privateNodeHandle_.param<double>("aligning_closeness_threshold", reOrientClosenessThreshold_, 0.2);
    privateNodeHandle_.param<double>("aligning_p_gain", reOrientPGain_, 0.5);
    privateNodeHandle_.param("map_frame_name", mapFrameName_, std::string("map"));
    privateNodeHandle_.param("robot_frame_name", robotFrameName_, std::string("base_link"));

    timer_ = nodeHandle_.createTimer(ros::Rate(timer_freq_), &BehavioralExecutive::timerCallback, this);
    /* Publishers */
    statePub_ = nodeHandle_.advertise<std_msgs::Int32>("/behavior_state", 1);
    stateTextPub_ = nodeHandle_.advertise<std_msgs::String>("/behavior_state_text", 1);

    // Sweep related
    imuSub_ = nodeHandle_.subscribe("imu", 1, &BehavioralExecutive::imuCallback, this);
    moveBaseCmdVelSub_ = nodeHandle_.subscribe("cmd_vel_move_base", 1, &BehavioralExecutive::moveBaseCmdVelCallback, this);
    outCmdVelPub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    humanEvidencePoseInBaseLinkPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/humanEvidencePoseInBaseLink", 1);
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
