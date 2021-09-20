#include "rrt_data.h"
#include <random>

planning::DragoonConfig::DragoonConfig(const float x, const float y, const float psi, const bool seeded, const int seed) : x(x), y(y), psi(psi), seeded(seeded), seed(seed)
{
}

void
planning::DragoonConfig::sampleConfig(const float searchX, const float searchY, const float searchPsi)
{
	x   = random(-searchX, searchX, seeded, seed);
	y   = random(-searchY, searchY, seeded, seed);
	psi = random(-searchPsi, searchPsi, seeded, seed);
}

planning::DragoonConfig
planning::DragoonConfig::operator+(const DragoonConfig& other)
{
	DragoonConfig config;
	config.x   = x + other.x;
	config.y   = y + other.y;
	config.psi = psi + other.psi;
	return config;
}

planning::DragoonConfig
planning::DragoonConfig::operator-(const DragoonConfig& other)
{
	DragoonConfig config;
	config.x   = x - other.x;
	config.y   = y - other.y;
	config.psi = psi - other.psi;
	return config;
}

planning::DragoonConfig
planning::operator*(const float other, const DragoonConfig current)
{
	DragoonConfig config;
	config.x   = other * current.x;
	config.y   = other * current.y;
	config.psi = other * current.psi;
	return config;
}

std::ostream &
planning::operator<<(std::ostream &out, const DragoonConfig& config)
{
	out << "[ " << config.x << ", " << config.y << ", " << config.psi << "]" << std::endl;
	return out;
}

float 
planning::norm(const DragoonConfig & config)
{
	float x   = config.x;
	float y   = config.y;
	float psi = config.psi;
	return sqrt(x * x + y * y + psi * psi);
}

planning::DragoonAction::DragoonAction(const float xDot, const float psiDot, const bool seeded, const int seed) : xDot(xDot), psiDot(psiDot), seeded(seeded), seed(seed)
{

}

void
planning::DragoonAction::sampleAction(const float xLim, const float psiLim)
{
	xDot   = random(-xLim, xLim, seeded, seed);
	psiDot = random(-psiLim, psiLim, seeded, seed);
}

planning::DragoonAction
planning::DragoonAction::operator-(const DragoonAction& other)
{
	DragoonAction action;
	action.xDot   = xDot - other.xDot;
	action.psiDot = psiDot - other.psiDot;
	return action;
}

planning::DragoonAction
planning::DragoonAction::operator+(const DragoonAction& other)
{
	DragoonAction action;
	action.xDot   = xDot + other.xDot;
	action.psiDot = psiDot + other.psiDot;
	return action;
}

planning::DragoonAction
planning::operator*(const float other, const DragoonAction current)
{
	DragoonAction action;
	action.xDot   = other * current.xDot;
	action.psiDot = other * current.psiDot;
	return action;
}