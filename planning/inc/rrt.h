#pragma once
#include "constraints.h"
#include <algorithm>
#include <array>

namespace planning
{
	

class RRT
{
	private:
		// constraints of the vehicle
		Constraints dragoonConstraints;
		std::vector<DragoonConfig> graph;
		std::vector<DragoonAction> actions;
		std::vector<int> parents;
		float goalBias = 0.3;
		float epsilon[3] = {0.3, 0.3, 0.1};
		const int searchThresh;
		float goalThresh;

	public:
		/**
		 * @brief constructs an RRT object
		 * @param numberOfShots the number of iterations to forward simulate the constraints
		 * @param model the model to use for the constraints
		 * @param dragoonLength length of dragoon
		 * @param searchThresh maximum nodes to expand
		 */
		RRT(int numberOfShots = 20, float dragoonLength = 0.2, int searchThresh = 3500);
		/**
		 * @brief runs the RRT
		 * @param start starting point of the RRT
		 * @param goal ending point of the RRT
		 * @param outPath reference to the path containing path from start to goal
		 */
		bool run(DragoonConfig start, DragoonConfig goal, std::vector<DragoonConfig> &outPath);
		/**
		 * @brief finds the index of the nearest vertex in the graph
		 * @param new_point the point to compare with everything in the graph
		 */
		int findNearestVertex(const DragoonConfig &new_point) const;
		/**
		 * @brief The goal sphere function samples from ellipsoid around the goal as a goal bias instead of always choosing the goal
		 * @param goal goal config
		 */
		DragoonConfig goalSphere(const DragoonConfig &goal) const;

		/**
		 * @brief set the graph to a specific list of configs
		 * @param newGraph the graph to set to 
		 */
		void setGraph(const std::vector<DragoonConfig>& newGraph);

		/**
		 * @brief returns the configuration at the index in the graph
		 * @param index index of config to return
		 * @return config if valid index, otherwise returns first config in the graph
		 */
		DragoonConfig getConfig(const int index) const;
};

} // namespace planning