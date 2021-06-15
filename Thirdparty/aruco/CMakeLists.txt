cmake_minimum_required(VERSION 3.0)
project(aruco)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

SET(sources_aruco
    aruco/cameraparameters.cpp  
    aruco/debug.cpp             
    aruco/dictionary.cpp       
    aruco/ippe.cpp    
    aruco/markerdetector.cpp       
    aruco/markerlabeler.cpp  
    aruco/posetracker.cpp
    aruco/cvdrawingutils.cpp    
    aruco/dictionary_based.cpp   
    aruco/marker.cpp  
    aruco/markerdetector_impl.cpp 
    aruco/markermap.cpp  
)
SET(headers_aruco
    aruco/aruco_cvversioning.h  
    aruco/cameraparameters.h  
    aruco/dictionary_based.h  
    aruco/ippe.h            
    aruco/markerdetector_impl.h  
    aruco/markermap.h    
    aruco/timers.h
    aruco/aruco_export.h        
    aruco/cvdrawingutils.h    
    aruco/dictionary.h        
    aruco/levmarq.h         
    aruco/marker.h               
    aruco/picoflann.h
    aruco/aruco.h               
    aruco/debug.h               
    aruco/markerdetector.h  
    aruco/markerlabeler.h        
    aruco/posetracker.h 
)


find_package(OpenCV 3.2.0 QUIET)
find_package(Eigen3 REQUIRED)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_library(aruco SHARED ${sources_aruco} )
target_link_libraries(aruco ${OpenCV_LIBS} ${EIGEN3_LIBS})
