#include <gtest/gtest.h>
#include "planning.h"
#include <vector>

TEST(TestRRT, TestFindNearestVertex)
{
	planning::RRT rrt;
	rrt.setGraph(
		{
			{1, 2, 3}, 
			{4, 2.5, 8},
			{9, 2, 1},
			{0.2, 0.6, 0.2}
		}
	);
	int compare = 3;
	planning::DragoonConfig config {0.0, 0.0, 0.0};
	int test = rrt.findNearestVertex(config);
	ASSERT_EQ(test, compare);
}

TEST(TestRRT, TestRetrieveNearestVertex)
{
	planning::RRT rrt;
	rrt.setGraph(
		{
			{0.6, -9.3, 0.23}, 
			{4, 2.5, 8.0},
			{-1.2, -2.2, 1.5},
			{0.2, -0.6, 0.2}
		}
	);
	int compare = 2;
	planning::DragoonConfig config {-4.3, -2.0, 8.4};
	int test = rrt.findNearestVertex(config);
	ASSERT_EQ(test, compare);


	std::vector<float> testConfig = rrt.getConfig(test).getConfig();
	ASSERT_FLOAT_EQ(-1.2, testConfig[0]);
	ASSERT_FLOAT_EQ(-2.2, testConfig[1]);
	ASSERT_FLOAT_EQ(1.5, testConfig[2]);

}

int main(int arc, char **argv)
{
	testing::InitGoogleTest(&arc, argv);
	return RUN_ALL_TESTS();
}