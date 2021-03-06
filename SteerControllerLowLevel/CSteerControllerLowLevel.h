/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CSteerControllerLowLevel_H
#define CSteerControllerLowLevel_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/hwdrivers/CSerialPort.h>

class CSteerControllerLowLevel : public COpenMORAApp
{
public:
    CSteerControllerLowLevel();
    virtual ~CSteerControllerLowLevel();

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


	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	double      m_steer_report_freq;
	mrpt::hwdrivers::CSerialPort m_serial;  //!< The serial COMMS object

	// Local methods:
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	bool WriteBinaryFrame( const uint8_t *data, uint16_t len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)

	bool CMD_EnableAutoControl(bool enable); //!< Sets the automatic/manual control in the steering controller board. Return false on COMMS error.
	bool CMD_SetClutch(bool enable); //!< Sets the Clutch relay.
	bool CMD_SetReportFreq(double freq); //!< Sets the report frequency
	bool CMD_SetPWMValue(double pwm_duty_cycle); //!< Sets the PWM value (-1.0 to 1.0)
	bool CMD_SetPosControlSetPoint(int pos_ticks); //!< Sets the position setpoint


	/** Convert encoder ticks into Ackermann central angle (radians) */
	double ackermannAngle(int32_t ticks) const;

	/** Convert Ackermann central angle (radians) to encoder ticks: */
	int32_t ackermannAngle_inverse(double angle_radians) const;



};
#endif
