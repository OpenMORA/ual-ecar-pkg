# File: CMakeListst.txt for module eCarOdometry
PROJECT(eCarOdometry)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED base obs) 

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CeCarOdometryApp.h CeCarOdometryApp.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
