include_directories(
  inc
)
add_library(loam
            BasicScanRegistration.cpp
            MultiScanRegistration.cpp
            LaserOdometry.cpp
            BasicLaserOdometry.cpp
            LaserMapping.cpp
            BasicLaserMapping.cpp
            TransformMaintenance.cpp
            BasicTransformMaintenance.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
