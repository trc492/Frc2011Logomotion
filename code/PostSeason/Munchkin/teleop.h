#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="teleop.h" />
///
/// <summary>
///     This main module contains the teleoperated mode code.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

/**
 * This function is called before starting the teleop mode.
 */
void
TrcRobot::TeleOpStart(
    void
    )
{
    TLevel(INIT);
    TEnter();
    
    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "TeleOp");
    //
    // Do other initializations for TeleOp mode here.
    //
#ifdef _USE_CAMERA
    AxisCamera::GetInstance();
#endif

    TExit();
}   //TeleOpStart

/**
 * This function is called before exiting the teleop mode.
 */
void
TrcRobot::TeleOpStop(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do clean up before exiting TeleOp mode here.
    //
#ifdef _USE_CAMERA
    AxisCamera::GetInstance().DeleteInstance();
#endif

    TExit();
}   //TeleOpStop

/**
 * This function is called periodically at fixed intervals in teleop mode.
 */
void
TrcRobot::TeleOpPeriodic(
    void
    )
{

    TLevel(HIFREQ);
    TEnter();

    //
    // Process input subsystems here.
    // (e.g. Reading joysticks, buttons, switches and sensors etc and
    //       computing the actions).
    //
#ifdef _USE_TANK_DRIVE
    float leftX = m_driverJoystickLeft->GetXWithDeadband();
    float leftY = -m_driverJoystickLeft->GetYWithDeadband();
    float rightX = m_driverJoystickLeft->GetZWithDeadband();
    float rightY = -m_driverJoystickLeft->GetTwistWithDeadband();

    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,
                        "L(%6.3f,%6.3f),R(%6.3f,%6.3f)",
                        leftX, leftY, rightX, rightY);
    //
    // We only do crabbing if both the left and right X axes are the same
    // sign. So to prevent accidental crabbing during tank drive, one would
    // either toe-in or toe-out the left and right joysticks while tank
    // driving.
    //
    float x = ((leftX < 0.0) && (rightX < 0.0) ||
               (leftX > 0.0) && (rightX > 0.0))? (leftX + rightX)/2.0: 0.0;
    float y = (leftY + rightY)/2.0;
    float rotation = (leftY - rightY)/2.0;
#else
    //
    // We only rotate when we are not crabbing. If we are crabbing, we kept
    // the same heading by ignoring rotation.
    //
    float x = m_driverJoystickRight->GetZWithDeadband();
    float rotation = (x == 0.0)? m_driverJoystickLeft->GetXWithDeadband(): 0.0;
    float y = -m_driverJoystickLeft->GetYWithDeadband();
#endif

    //
    // Perform output functions here.
    // (e.g. Programming motors, pneumatics and solenoids etc)
    //
    m_driveBase->MecanumDrive_Cartesian(x, y, rotation);
    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
                        "x=%6.3f,y=%6.3f,rot=%6.3f",
                        x, y, rotation);
    
    //
    // send the dashbaord data associated with the I/O ports
    //
    sendIOPortData();

    TExit();
}   //TeleopPeriodic

/**
 * This function is called by the TrcJoystick object when a button event
 * occurred.
 *
 * @param port Specifies the joystick port.
 * @param maskButton Specifies the button mask.
 * @param fPressed If true, specifies the button is pressed, false if
 *        released.
 */
void
TrcRobot::NotifyButton(
    __in UINT32 port,
    __in UINT16 maskButton,
    __in bool   fPressed
    )
{
    TLevel(EVENT);
    TEnterMsg(("port=%d,Button=0x%x,fPressed=%d",
               port, maskButton, fPressed));

    if (port == JSPORT_DRIVER_LEFT)
    {
        switch (maskButton)
        {
        }
    }
    else if (port == JSPORT_DRIVER_RIGHT)
    {
        switch (maskButton)
        {
        }
    }

    TExit();
}	//NotifyButton
