cmake_minimum_required(VERSION 3.0.2)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(artifacts_bci)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosneuro_msgs
  rosneuro_filters
  rosneuro_buffers
  rosneuro_buffers_ringbuffer
  rosneuro_filters_car
  rosneuro_filters_butterworth
  std_msgs
  message_generation
) 
find_package(PkgConfig)

find_package(Eigen3 REQUIRED)
find_library(RTF_LIBRARIES rtfilter)
find_path(RTF_INCLUDE_DIRS rtfilter.h)

add_message_files(
  FILES
  artifact_presence.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
	
  CATKIN_DEPENDS
  	roscpp
	  std_msgs
    rosneuro_filters
    rosneuro_buffers
    rosneuro_filters_car
    rosneuro_buffers_ringbuffer
    rosneuro_filters_butterworth
    rosneuro_msgs
    message_runtime
  DEPENDS
)



###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# online ARTIFACTS library
add_library(${PROJECT_NAME}
                          src/ArtifactDetector.cpp
                          )

target_link_libraries(${PROJECT_NAME}
                    ${Eigen3_LIBRARIES} 
									  ${catkin_LIBRARIES}
                    ${RTF_LIBRARIES}
                    )

add_dependencies(${PROJECT_NAME}
				 ${catkin_EXPORTED_TARGETS}
)



################ EXEC
# processing NODE
add_executable(artifactDetector_node src/artifactDetector_node.cpp)
target_link_libraries(artifactDetector_node 
                        ${PROJECT_NAME}
                        ) 

# test rosnode NODE
add_executable(test_publisher_artifacts_bci test/publisher.cpp)
target_link_libraries(test_publisher_artifacts_bci 
                      ${PROJECT_NAME}
                      )  

add_executable(test_logger_artifacts_bci test/logger.cpp)
target_link_libraries(test_logger_artifacts_bci 
                      ${PROJECT_NAME}
                      )  