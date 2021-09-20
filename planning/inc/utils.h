#pragma once
#include <random>

namespace planning
{

/**
 * @brief function that generates a random number between low and high
 * @param low low end for number
 * @param high high end for number
 * @param seeded whether to seed or not
 * @param seed the seed to use
 */
float random(const float low = 0.0, const float high = 1.0, bool seeded=false, int seed = 0);

	
} // namespace planning
