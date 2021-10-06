#include <string>
#include <vector>
#include <unordered_map>

// dictionary to describe the current events
const std::string STOP         = "STOP";
const std::string START        = "START";
const std::string USER_CONTROL = "USER CONTROL";
const std::string NEW_HUMAN    = "NEW HUMAN";
const std::string OLD_HUMAN    = "OLD HUMAN";
const std::string NO_HUMAN     = "NO HUMAN";
const std::string GOAL_REACHED = "GOAL REACHED";

std::unordered_map<std::string, bool> eventDict = {
	{STOP         , false} ,
	{START        , false} ,
	{USER_CONTROL , false} ,
	{NEW_HUMAN    , false} ,
	{OLD_HUMAN    , false} ,
	{NO_HUMAN     , false} ,
	{GOAL_REACHED , false} ,
};

enum State {
	IDLE_STATE, 
	EXPLORE_STATE,
	INPUT_STATE,
	APPROACH_STATE,
	SWEEP_STATE,
};