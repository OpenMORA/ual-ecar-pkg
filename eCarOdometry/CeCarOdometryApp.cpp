/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Differential-drive odometry estimation from a pair of encoders in the rear wheels.
  *  <br>
  *  It is assumed that ENC1_CH0_COUNT is the LEFT wheel, ENC1_CH1_COUNT is the RIGHT encoder.
  */

#include "CeCarOdometryApp.h"
#include <mrpt/slam/CObservationOdometry.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


CeCarOdometryApp::CeCarOdometryApp() :
	m_left_fresh(false), m_right_fresh(false),
	m_cur_left_enc_pos(0), m_cur_right_enc_pos(0),
	m_last_left_enc_pos (0), m_last_right_enc_pos(0),
	m_last_left_ok(false), m_last_right_ok(false),
	m_left_speed_fresh(false), m_right_speed_fresh(false),
	m_cur_left_enc_vel(0), m_cur_right_enc_vel(0),
	m_cur_v(0), m_cur_w(0),
	// Odometry params:
	m_left_K(0.01),
	m_right_K(0.01),
	m_wheels_dist(2.0),
	m_odometry_verbose(false)
{
}

CeCarOdometryApp::~CeCarOdometryApp()
{
}

bool CeCarOdometryApp::OnStartUp()
{
	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL

	// We want to be notified as soon as possible about changes in encoders:
	SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL);

	//! @moos_param left_K  Ticks to meters constant for left wheel encoders.
	m_MissionReader.GetConfigurationParam("left_K",m_left_K);

	//! @moos_param right_K  Ticks to meters constant for right wheel encoders.
	m_MissionReader.GetConfigurationParam("right_K",m_right_K);

	//! @moos_param wheels_dist  Wheel-to-wheel distance (meters)
	m_MissionReader.GetConfigurationParam("wheels_dist",m_wheels_dist);

	//! @moos_param odometry_verbose  Set to "true" to display live odometry to console.
	m_MissionReader.GetConfigurationParam("odometry_verbose",m_odometry_verbose);


	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CeCarOdometryApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

// Main module loop code.
bool CeCarOdometryApp::Iterate()
{
	if (m_left_speed_fresh && m_right_speed_fresh)
	{
		m_left_speed_fresh  = false;
		m_right_speed_fresh = false;

		differentialOdometryModelVel(m_cur_v,m_cur_w,m_cur_left_enc_vel,m_cur_right_enc_vel);

		//!  @moos_var   ODOMETRY_LIN_SPEED  The vehicle instantaneous linear velocity (m/s) (positive=forward)
		//!  @moos_publish   ODOMETRY_LIN_SPEED
		m_Comms.Notify("ODOMETRY_LIN_SPEED", m_cur_v );

		//!  @moos_var   ODOMETRY_ANG_SPEED  The vehicle instantaneous angular speed (rad/s) (positive=counterclockwise)
		//!  @moos_publish   ODOMETRY_ANG_SPEED
		m_Comms.Notify("ODOMETRY_ANG_SPEED", m_cur_w );

		//if (m_odometry_verbose) std::cout << "ODO: LinVel=" << m_cur_v << " m/s. AngVel=" << RAD2DEG(m_cur_w) << " deg/s\n";
	}

	if (m_left_fresh && m_right_fresh)
	{
		// Mark left & right data as "already used" (old):
		m_left_fresh = false;
		m_right_fresh = false;

		MRPT_TODO("Handle potential encoders overflow!!")

		// Compute increment:
		int left_incr  = m_cur_left_enc_pos - m_last_left_enc_pos;
		int right_incr = m_cur_right_enc_pos - m_last_right_enc_pos;

		m_last_left_enc_pos  = m_cur_left_enc_pos;
		m_last_right_enc_pos = m_cur_right_enc_pos;

		mrpt::poses::CPose2D odo_incr;
		this->differentialOdometryModel(odo_incr,left_incr,right_incr);

		m_global_odometry += odo_incr; // Cummulative odometry:

		//!  @moos_var   ODOMETRY   The robot absolute odometry in format "[x y phi]"
		{
			string sOdo;
			m_global_odometry.asString(sOdo);
			m_Comms.Notify("ODOMETRY", sOdo );
			if (m_odometry_verbose)
				std::cout << "ODO: " << sOdo << "\n";
		}

		//!  @moos_var   ODOMETRY_INCR   Robot odometry increments in format "[x y phi]"
		{
			string sOdoIncr;
			odo_incr.asString(sOdoIncr);
			m_Comms.Notify("ODOMETRY_INCR", sOdoIncr );
		}

		//!  @moos_var   ODOMETRY_OBS The robot absolute odometry as MRPT binary observation
		{
			mrpt::slam::CObservationOdometryPtr odom = mrpt::slam::CObservationOdometry::Create();
			odom->timestamp = mrpt::system::now();
			odom->sensorLabel = "ODOMETRY";
			odom->odometry = m_global_odometry;
			odom->hasVelocities = true;
			odom->velocityLin = m_cur_v;
			odom->velocityAng = m_cur_w;
			odom->hasEncodersInfo = true;
			odom->encoderLeftTicks = left_incr;
			odom->encoderRightTicks = right_incr;

			mrpt::vector_byte bOdom;
			mrpt::utils::ObjectToOctetVector(odom.pointer(), bOdom);
			m_Comms.Notify("ODOMETRY_OBS", bOdom );
		}

	} // odo is fresh

	return true;
}

bool CeCarOdometryApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CeCarOdometryApp::DoRegistrations()
{
	//! @moos_subscribe	ENC1_CH0_COUNT, ENC1_CH1_COUNT
	m_Comms.Register("ENC1_CH0_COUNT");
	m_Comms.Register("ENC1_CH1_COUNT");

	//! @moos_subscribe	ENC1_CH0_AVRG_VEL, ENC1_CH1_AVRG_VEL
	m_Comms.Register("ENC1_CH0_AVRG_VEL");
	m_Comms.Register("ENC1_CH1_AVRG_VEL");

	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CeCarOdometryApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	CMOOSMsg Msg;
	if(m_Comms.PeekMail(NewMail,"ENC1_CH0_COUNT",Msg))
	{
		m_cur_left_enc_pos = Msg.GetDouble();
		m_left_fresh = true;
		if (!m_last_left_ok)
		{
			m_last_left_ok = true;
			m_last_left_enc_pos = m_cur_left_enc_pos;
		}
	}
	if(m_Comms.PeekMail(NewMail,"ENC1_CH1_COUNT",Msg))
	{
		m_cur_right_enc_pos= Msg.GetDouble();
		m_right_fresh = true;
		if (!m_last_right_ok)
		{
			m_last_right_ok = true;
			m_last_right_enc_pos = m_cur_right_enc_pos;
		}
	}

	if(m_Comms.PeekMail(NewMail,"ENC1_CH0_AVRG_VEL",Msg))
	{
		m_cur_left_enc_vel = Msg.GetDouble();
		m_left_speed_fresh = true;
	}
	if(m_Comms.PeekMail(NewMail,"ENC1_CH1_AVRG_VEL",Msg))
	{
		m_cur_right_enc_vel = Msg.GetDouble();
		m_right_speed_fresh = true;
	}


	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}


/** The odometry model: converts encoder ticks to pose increments, using the parameter loaded from the config file */
void CeCarOdometryApp::differentialOdometryModel(mrpt::poses::CPose2D &out_increment, int left_tick_incr, int right_tick_incr) const
{
	double	As = 0.5* ( m_right_K*right_tick_incr + m_left_K*left_tick_incr );
	double	Aphi = ( m_right_K*right_tick_incr - m_left_K*left_tick_incr ) / m_wheels_dist;
//
//    cout << "As_left = " << m_left_K*left_tick_incr << " ticks: " << left_tick_incr << endl;
//    cout << "As_right = " << m_right_K*right_tick_incr << " ticks: " << right_tick_incr << endl;

	out_increment = mrpt::poses::CPose2D(
		cos(Aphi)*As,
		sin(Aphi)*As,
		Aphi);
}

/** The odometry model in velocity: converts encoder velocities to vehicle velocities, using the parameter loaded from the config file */
void CeCarOdometryApp::differentialOdometryModelVel(double &out_v, double &out_w, const double left_ticks_vel, const double right_ticks_vel) const
{
	out_v = 0.5* ( m_right_K*right_ticks_vel + m_left_K*left_ticks_vel );
	out_w = ( m_right_K*right_ticks_vel - m_left_K*left_ticks_vel ) / m_wheels_dist;
}
