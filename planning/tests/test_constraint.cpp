#include <gtest/gtest.h>
#include "planning.h"
#include <vector>

TEST(TestConstraint, TestForwardSimulation)
{
	planning::Constraints constraints;
	planning::DragoonConfig out;
	planning::DragoonConfig init {0, 0, 0};
	planning::DragoonAction action {1, 0};
	constraints.expandVertex(
		init, 
		action,
		out
	);
	float compare[3] = {1.0, 0.0, 0.0};
	std::vector<float> test = out.getConfig();
	ASSERT_NEAR(test[0], compare[0], 1E-5);
	ASSERT_NEAR(test[1], compare[1], 1E-5);
	ASSERT_NEAR(test[2], compare[2], 1E-5);
}

int main(int arc, char **argv)
{
	testing::InitGoogleTest(&arc, argv);
	return RUN_ALL_TESTS();
}