/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CeCarOdometryApp_H
#define CeCarOdometryApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/poses/CPose2D.h>

class CeCarOdometryApp : public COpenMORAApp
{
public:
    CeCarOdometryApp();
    virtual ~CeCarOdometryApp();

protected:
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

	bool DoRegistrations();

	// Local variables:
	bool  m_left_fresh, m_right_fresh;   //!< True when fresh data is here.
	int   m_cur_left_enc_pos, m_cur_right_enc_pos;
	int   m_last_left_enc_pos, m_last_right_enc_pos;
	bool  m_last_left_ok, m_last_right_ok; //!< Will be true when the first real data is received.

	bool   m_left_speed_fresh, m_right_speed_fresh;   //!< True when fresh data is here.
	double m_cur_left_enc_vel, m_cur_right_enc_vel;

	double m_cur_v, m_cur_w; // Latest velocity estimations

	mrpt::poses::CPose2D m_global_odometry; //!< Global current odometry pose

	// Odometry params:
	double m_left_K, m_right_K; //!< Ticks to meter constants for each encoder
	double m_wheels_dist;       //!< Distance between wheels (meters)
	bool   m_odometry_verbose;  //!< Display odo info to cout


	/** The odometry model: converts encoder ticks to pose increments, using the parameter loaded from the config file */
	void differentialOdometryModel(mrpt::poses::CPose2D &out_increment, int left_tick_incr, int right_tick_incr) const;

	/** The odometry model in velocity: converts encoder velocities to vehicle velocities, using the parameter loaded from the config file */
	void differentialOdometryModelVel(double &out_v, double &out_w, const double left_ticks_vel, const double right_ticks_vel) const;




};
#endif
