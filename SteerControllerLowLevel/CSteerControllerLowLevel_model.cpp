/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Interface to the embedded system controlling the vehicle steering wheel  */

#include "CSteerControllerLowLevel.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;



/** Convert encoder ticks into Ackermann central angle (radians) */
double CSteerControllerLowLevel::ackermannAngle(int32_t ticks) const
{
	MRPT_TODO("*** Put Ackermann model here!!!! *** ")
	return 1.0*ticks;
}


/** Convert Ackermann central angle (radians) to encoder ticks: */
int32_t CSteerControllerLowLevel::ackermannAngle_inverse(double angle_radians) const
{
	MRPT_TODO("*** Put inverse Ackermann model here!!!! *** ")
	return angle_radians;
}

