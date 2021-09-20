#include <gtest/gtest.h>
#include "utils.h"

TEST(UtilsTest, TestRandomNumberGenerator)
{
	float test;
	for (int i = 0; i < 100; i++)
	{
		test = planning::random();
		// make sure the value is within the range set for this function
		ASSERT_GE(test, 0.0);
		ASSERT_LE(test, 1.0);
	}
	
}

TEST(UtilsTest, TestRandomNumberRange)
{
	float test;
	float low = -3.89;
	float high = 90.0;
	for (int i = 0; i < 100; i++)
	{
		test = planning::random(low, high);
		ASSERT_GE(test, low);
		ASSERT_LE(test, high);
	}
}

TEST(UtilsTest, TestRandomNumberSeed)
{
	float test;
	float low = -3;
	float high = 3;
	int seed = 10;
	float expected = 1.62792; 
	for (int i = 0; i < 100; i++)
	{
		test = planning::random(low, high, true, seed);
		ASSERT_GE(test, low);
		ASSERT_LE(test, high);
		ASSERT_NEAR(expected, test, 1e-5);
	}

}

int main(int arc, char **argv)
{
	testing::InitGoogleTest(&arc, argv);
	return RUN_ALL_TESTS();
}