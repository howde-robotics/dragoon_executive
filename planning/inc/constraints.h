#pragma once
#include "rrt_data.h"
#include <string>
namespace planning
{
	

/**
 * @brief Constraint handler class for the RRT propagation
 */
class Constraints
{
	private:
		const int numberOfShots;
		const float dragoonLength;
		// limits for sampling actions
		float xLim;
		float psiLim;
		// simulation parameters
		float dt;
		int timeSteps;

	public:
		/**
		 * @brief Construct constraint handler class
		 * @param numberOfShots the number of times to sample different actions before using it in the RRT
		 * @param model the type of model to use for dragoon
		 * @param dragoonLength the width of dragoon
		 */
		Constraints(const int numberOfShots = 20, const float dragoonLength = 0.2);

		/** 
		 * @brief this function handles the constraints on the RRT including the collision checking and kinematic shooting algorithm
		 * @param qNear config closest to qRand
		 * @param qRand sample configuration
		 * @param qOut reference to configuration after shooting algo
		 * @param actionOut the action that drives the vehicle to qOut from qNear
		 */
		bool handleConstraints(DragoonConfig &qNear, DragoonConfig &qRand, DragoonConfig &qOut, DragoonAction &actionOut);
		/**
		 * @brief Expands the vertex based off of the shooting algo
		 * @param qInit starting point to forward simulate off of
		 * @param sampAction randomized sample action
		 * @param qOut the output configuration after shooting algo
		 */
		bool expandVertex(const DragoonConfig &qInit, const DragoonAction &sampAction, DragoonConfig &qOut);
		/**
		 * @brief Checks if the current edge encounters a collision
		 */
		bool checkCollision();
};

} // namespace planning