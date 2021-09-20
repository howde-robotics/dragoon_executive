find_package(GTest REQUIRED)

# the planning library
add_library(planning 
	# rrt data such as config and actions types
	src/rrt_data
	# rrt stuff
	src/rrt
	# constraints
	src/constraints
	# visualizer for rviz
	src/visualizer
	# utiility functions
	src/utils
	# the main planning stuff
	src/planning
)

# have planning target bring its include with it
target_include_directories(planning PUBLIC inc ${GTEST_INCLUDE_DIRS})

# deal with unit tests
set(TESTING_LIBS
	planning 
	# testing libraries
	${GTEST_LIBRARIES} 
	${catkin_LIBRARIES}
	pthread
)
# create the test binaries
set(TESTS
	test_rrt_data
	test_utils
	test_rrt
	test_constraint
)
# use this forloop to create all the tests
foreach(TEST ${TESTS})
	# add the executable
	catkin_add_gtest(${TEST} tests/${TEST})
	# link the libraries
	target_link_libraries(${TEST} ${TESTING_LIBS})

endforeach()