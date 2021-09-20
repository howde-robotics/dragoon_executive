#include "utils.h"

float
planning::random(const float low, const float high, bool seeded, int seed)
{

	// random device based on stochastic process
	std::random_device random;
	std::mt19937 generator;
	// if the seed is to be user defined use that here
	if (seeded)
	{
		generator = std::mt19937(seed);
	}
	else
	{
		// else use the random device to generate number
		generator = std::mt19937(random());
	}
	std::uniform_real_distribution<float> reals(low, high);
	return reals(generator);
}
