# File: CMakeListst.txt for module 
PROJECT(SteerControllerLowLevel)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED hwdrivers ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CSteerControllerLowLevel.h CSteerControllerLowLevel.cpp CSteerControllerLowLevel_model.cpp
	steering-control-firmware2pc-structs.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )

