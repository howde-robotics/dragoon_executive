#pragma once
#include <iostream>
#include <vector>
#include "utils.h"

namespace planning
{
/**
 * @brief This class contains a sample configuration of dragoon's state
 */
class DragoonConfig
	{
	private:
		bool seeded = false;
		int seed = 0.0;
	public:
		float x = 0.0;
		float y = 0.0;
		float psi = 0.0;
		/**
		 * @brief Construct new configuration. Params default to 0
		 * @param x The x location of dragoon
		 * @param y The y location of dragoon
		 * @param psi The yaw of dragoon
		 */
		DragoonConfig(const float x = 0.0, const float y = 0.0, const float psi = 0.0, const bool seeded = false, const int seed = 0.0);
		/**
		 * @brief produce a random sample configuration
		 * @param searchX range of x sample
		 * @param searchY range of y sample
		 * @param searchPsi range of psi sample
		 * @return randomized sample configuration [x, y, psi]
		 */
		void sampleConfig(const float searchX, const float searchY, const float searchPsi);

		// operator overloads
		DragoonConfig operator-(const DragoonConfig& other);
		DragoonConfig operator+(const DragoonConfig& other);
		friend DragoonConfig operator*(const float other, const DragoonConfig current);
		// calculating norms
		friend float norm(const DragoonConfig & config);
		// std output
		friend std::ostream& operator<<(std::ostream &out, const DragoonConfig& config);

		// getters and setters
		std::vector<float> getConfig() const {return std::vector<float> {x, y, psi};};
		void setSeed(int seed){seed = seed;}
		void setConfig(const DragoonConfig &config){
			x   = config.x;
			y   = config.y;
			psi = config.psi;
		}
		
};

/**
 * @brief The class that contains a sample action for dragoon's model
 */
class DragoonAction
{
	private:
		bool seeded = false;
		int seed = 0.0;
	public:
		float xDot = 0.0;
		float psiDot = 0.0;

		/**
		 * @brief Construct a new action for dragoon. Params default to zero
		 * @param xDot Velocity in the x direction
		 * @param psiDot Turning velocity in yaw or steering velocity
		 * @param seeded whether to seed the random generation
		 * @param seed value to use as seed
		 */
		DragoonAction(const float xDot = 0.0, const float psiDot = 0.0, const bool seeded = false, const int seed = 0.0);

		/**
		 * @brief generates a random sample configuration
		 * @param xLim range of sample is -xLim, xLim
		 * @param psiLim range of sample is -psiLim, psiLim
		 * @return [xDot, psiDot]
		 */
		void sampleAction(const float xLim, const float psiLim);

		// operator overloads
		DragoonAction operator-(const DragoonAction& other);
		DragoonAction operator+(const DragoonAction& other);
		friend DragoonAction operator*(const float other, const DragoonAction current);

		// getters and setters
		std::vector<float> getAction() const {return std::vector<float> {xDot, psiDot};}
		void setSeed(int seed){seed = seed;}
		void setAction(const DragoonAction &action){
			xDot   = action.xDot;
			psiDot = action.psiDot;
		}
};

// these friend functions can access the member variables of each class
DragoonConfig operator*(const float other, const DragoonConfig current);
DragoonAction operator*(const float other, const DragoonAction current);
float norm(const DragoonConfig & config);
std::ostream& operator<<(std::ostream &out, const DragoonConfig& config);
} // namespace planning
