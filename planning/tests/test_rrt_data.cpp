#include <gtest/gtest.h>
#include "rrt_data.h"
#include <vector>

class RRTDataTest : public ::testing::Test
{
	protected:
		planning::DragoonAction testAction;
		planning::DragoonConfig testConfig;

	RRTDataTest() : testAction(0, 0, true, 10), testConfig(0, 0, 0, true, 10)
	{
		
	}
};

TEST_F(RRTDataTest,TestRandomSampleNumber){
	testConfig.sampleConfig(3, 3, 3);
	std::vector<float> test = testConfig.getConfig();
	float compare[3] = {
		1.62792,
		1.62792,
		1.62792,
	 };
	ASSERT_NEAR(test[0], compare[0], 1E-4);
	ASSERT_NEAR(test[1], compare[1], 1E-4);
	ASSERT_NEAR(test[2], compare[2], 1E-4);
}

TEST_F(RRTDataTest, TestRandomActionNumber){
	testAction.sampleAction(3, 3);
	std::vector<float> test = testAction.getAction();
	float compare[2] = {
		1.62792,
		1.62792,
	};
	ASSERT_NEAR(test[0], compare[0], 1E-4);
	ASSERT_NEAR(test[1], compare[1], 1E-4);
}

TEST(RRTData, TestRandomConfig)
{
	planning::DragoonConfig one;
	float xLim = 10;
	float yLim = 3;
	float psiLim = 23.5;
	std::vector<float> test;
	
	for (int i = 0; i < 100; i++)
	{
		one.sampleConfig(xLim, yLim, psiLim);
		test = one.getConfig();
		ASSERT_LE(test[0], xLim);
		ASSERT_GE(test[0], -xLim);
		ASSERT_LE(test[1], yLim);
		ASSERT_GE(test[1], -yLim);
		ASSERT_LE(test[2], psiLim);
		ASSERT_GE(test[2], -psiLim);

	}
}

TEST(RRTData, TestRandomAction)
{
	planning::DragoonAction one;
	float xLim = 10;
	float psiLim = 23.5;
	std::vector<float> test;
	
	for (int i = 0; i < 100; i++)
	{
		one.sampleAction(xLim, psiLim);
		test = one.getAction();
		ASSERT_LE(test[0], xLim);
		ASSERT_GE(test[0], -xLim);
		ASSERT_LE(test[1], psiLim);
		ASSERT_GE(test[1], -psiLim);

	}
}

TEST(RRTData, TestConfigAlgebra)
{
	planning::DragoonConfig one(1, 2, 3);
	planning::DragoonConfig two(1.3, 2.9, 3.32);
	planning::DragoonConfig sum = one + two;
	std::vector<float> test = sum.getConfig();
	float compare[3] = {
		2.3,
		4.9,
		6.32
	};
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
	ASSERT_FLOAT_EQ(test[2], compare[2]);
	// and now test subtraction
	sum = one - two;
	test = sum.getConfig();
	compare[0] = -0.3;
	compare[1] = -0.9;
	compare[2] = -0.32;
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
	ASSERT_FLOAT_EQ(test[2], compare[2]);
}

TEST(RRTData, TestActionAlgebra)
{
	planning::DragoonAction one(1, 2);
	planning::DragoonAction two(1.3, 2.9);
	planning::DragoonAction sum = one + two;
	std::vector<float> test = sum.getAction();
	float compare[3] = {
		2.3,
		4.9,
	};
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
	// and now test subtraction
	sum = one - two;
	test = sum.getAction();
	compare[0] = -0.3;
	compare[1] = -0.9;
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
}

TEST(RRTData, TestConfigScalarMultiplication)
{
	planning::DragoonConfig one(1, 2, 3);
	planning::DragoonConfig multiple = 0.5 * one;
	std::vector<float> test = multiple.getConfig();
	float compare[3] = {
		0.5,
		1.0,
		1.5,
	};
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
	ASSERT_FLOAT_EQ(test[2], compare[2]);
}

TEST(RRTData, TestActionScalarMultiplication)
{
	planning::DragoonAction one(1, 10);
	planning::DragoonAction multiple = 0.5 * one;
	std::vector<float> test = multiple.getAction();
	float compare[3] = {
		0.5,
		5.0,
	};
	ASSERT_FLOAT_EQ(test[0], compare[0]);
	ASSERT_FLOAT_EQ(test[1], compare[1]);
}

TEST(RRTData, TestNormFunctionConfig)
{
	planning::DragoonConfig one(1, 2, 3);
	planning::DragoonConfig two(1.3, 2.9, 3.32);
	float test = planning::norm(one);
	float compare = 3.74165;
	ASSERT_NEAR(test, compare, 1E-5);

	test = planning::norm(two);
	compare = 4.59591;
	ASSERT_NEAR(test, compare, 1E-5);

	test = planning::norm(one - two);
	compare = 1.00119;
	ASSERT_NEAR(test, compare, 1E-5);

	test = planning::norm(one + two);
	compare = 8.32120;
	ASSERT_NEAR(test, compare, 1E-5);

	test = planning::norm(1.45 * one);
	compare = 5.42540;
	ASSERT_NEAR(test, compare, 1E-5);

	one = planning::DragoonConfig(0.23, -0.5, 1.6);
	test = planning::norm(one);
	compare = 1.69201;
	ASSERT_NEAR(test, compare, 1E-5);

}
int main(int arc, char **argv)
{
	testing::InitGoogleTest(&arc, argv);
	return RUN_ALL_TESTS();
}