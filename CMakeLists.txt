cmake_minimum_required( VERSION 2.8 )
project ( draw_trajectory )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
#set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
#Pangolin
find_package(Pangolin)
include_directories( ${Pangolin_INCLUDE_DIRS})
# Eigen
include_directories( "/usr/include/eigen3" )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )


add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory
            ${EIGEN3_LIBS}
            ${Pangolin_LIBRARIES}
            ${Sophus_LIBRARIES}
         )

add_executable(draw_gt_est draw_gt_est.cpp)
target_link_libraries(draw_gt_est
            ${EIGEN3_LIBS}
            ${Pangolin_LIBRARIES}
            ${Sophus_LIBRARIES}
         )