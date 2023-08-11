CMAKE_MINIMUM_REQUIRED( VERSION 3.5)
PROJECT(learn_sophus)

set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV 4 REQUIRED)

SET(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS})


# add_executable(testSophus test.cpp)
add_executable(learn_sophus 
    # test_SE3.cpp
    test_SE3_translation.cpp
    )

target_link_libraries(learn_sophus ${Sophus_LIBRARIES} 
                                    ${Boost_LIBRARIES}
                                    ${OpenCV_LIBS})

# link_directories(test_boost ${Boost_LIBRARY_DIRS})
# target_link_libraries(test_boost )