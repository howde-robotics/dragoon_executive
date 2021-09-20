#include "constraints.h"

namespace planning
{
	
Constraints::Constraints(const int numberOfShots, const float dragoonLength) : numberOfShots(numberOfShots), dragoonLength(dragoonLength)
{
	// to be set by ros parameters
	xLim      = 0.2;
	psiLim    = M_PI;
	dt        = 0.01;
	timeSteps = 100;

}

bool
Constraints::handleConstraints(DragoonConfig &qNear, DragoonConfig &qRand, DragoonConfig &qOut, DragoonAction &actionOut)
{
	// initialize configurations and actions
	DragoonConfig compare;
	DragoonConfig sampState;
	DragoonAction sampAction;
	bool valid = false;
	float distanceToQRand = std::numeric_limits<float>::max();

	static constexpr float orientation_weightage = 0.2;

	for (int i = 0; i < numberOfShots; i++)
	{
		// sample from action space
		sampAction.sampleAction(xLim, psiLim);
		// expand the vertex
		bool collision = expandVertex(qNear, sampAction, sampState);

		// check for collision and check distance
		if (not collision)
		{
			compare = sampState - qRand;
			// weight the orientation
			compare.psi =  orientation_weightage * compare.psi;
			const float distance = norm(compare);
			// select action closest to qRand, and update distance
			if (distance < distanceToQRand)
			{
				distanceToQRand = norm(sampState - qRand);
				qOut.setConfig(sampState);
				actionOut.setAction(sampAction);
				valid = true;
			}

		}
	}
	return valid;

}

bool 
Constraints::expandVertex(const DragoonConfig &qInit, const DragoonAction &sampAction, DragoonConfig &qOut)
{
	// forward simulation
	qOut.setConfig(qInit);
	for (int i = 0; i < timeSteps; i++)
	{
		qOut.x   += dt*sampAction.xDot*cos(qOut.psi);
		qOut.y   += dt*sampAction.xDot*sin(qOut.psi);
		qOut.psi += dt*sampAction.psiDot;
	}
	
	return checkCollision();
}

bool
Constraints::checkCollision()
{
	return false;
}

} // namespace planning
