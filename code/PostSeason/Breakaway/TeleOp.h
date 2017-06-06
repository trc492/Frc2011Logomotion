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

    TExit();
}   //TeleOpStart

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

    float x = m_driverJoystickRight->GetZWithDeadband();
    float y = m_driverJoystickRight->GetYWithDeadband();

    if (m_fVisionTargetEnabled)
    {
        m_driveBase->VisionDrive(x, y);
    }
    else
    {
        float rotation = m_driverJoystickRight->GetXWithDeadband();

        m_driveBase->MecanumDrive_Polar(MAGNITUDE(x, y),
                                        DIR_DEGREES(x, -y),
                                        rotation);
    }

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

    static bool fLidUp = false;

    if (port == JSPORT_DRIVER_LEFT)
    {
        switch (maskButton)
        {
#if 0
#ifdef _DRIVE_CAL
            case Logitech_Trigger:
                if (fPressed)
                {
                    toggleAccel = !toggleAccel;
                    m_drive->SetAccelEnabled(toggleAccel);
                }
                break;
#endif
#ifdef _DRIVE_CAL
            case Logitech_Btn2:
                if(fPressed)
                {
                    fCalTurnPID = !fCalTurnPID;
                    TInfo(("Calibrating %s PID.",
                           fCalTurnPID? "turn": "drive"));
                }
                break;
#endif
#endif
        }
    }
    else if (port == JSPORT_DRIVER_RIGHT)
    {
        switch (maskButton)
        {
            /*case Logitech_Trigger:
                //
                // Activate/deactivate vision targeting.
                //
                m_fVisionTargetEnabled = fPressed;
                if (fPressed)
                {
                    GetWatchdog().SetExpiration(2.0);
                }
                else
                {
                    GetWatchdog().SetExpiration(0.5);
                    //
                    // Stop vision drive.
                    //
                    m_driveBase->Stop();
                }
                break;*/
#if 0
#ifdef _DRIVE_CAL
            case Logitech_Btn2:
                if (fPressed)
                {
                    if (m_fDriveCal)
                    {
                        m_fDriveCal = false;
                        m_drive->DriveStop();
                        m_drive->SetAccelEnabled(false);
                        TInfo(("Stop calibration."));
                    }
                    else
                    {
                        m_fDriveCal = true;
                        m_drive->SetAccelEnabled(true);
//                        m_drive->DrivePIDDistance(10.0, 0.0);
                        m_drive->DrivePID(10.0, 0.0, 0.0);
                        TInfo(("Start drive calibration."));
                    }
                }
                break;

            case Logitech_Btn3:
                if (fPressed)
                {
                    if (m_fDriveCal)
                    {
                        m_fDriveCal = false;
                        m_drive->DriveStop();
                        TInfo(("Stop calibration."));
                    }
                    else
                    {
                        m_fDriveCal = true;
//                        m_drive->DrivePIDAngle(360.0);
                        m_drive->DrivePID(0.0, 0.0, 360.0);
                        TInfo(("Start turn calibration."));
                    }
                }
                break;

            case Logitech_Btn7:
                if (fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(0.01, 0.0, 0.0);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(0.01, 0.0, 0.0);
                    }
                }
                break;

            case Logitech_Btn8:
                if (fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(-0.01, 0.0, 0.0);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(-0.01, 0.0, 0.0);
                    }
                }
                break;

            case Logitech_Btn9:
                if (fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(0.0, 0.001, 0.0);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(0.0, 0.001, 0.0);
                    }
                }
                break;

            case Logitech_Btn10:
                if (fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(0.0, -0.001, 0.0);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(0.0, -0.001, 0.0);
                    }
                }
                break;

            case Logitech_Btn11:
                if(fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(0.0, 0.0, 0.01);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(0.0, 0.0, 0.01);
                    }
                }
                break;

            case Logitech_Btn12:
                if(fPressed)
                {
                    if (fCalTurnPID)
                    {
                        m_drive->ChangeTurnPID(0.0, 0.0, -0.01);
                    }
                    else
                    {
                        m_drive->ChangeDrivePID(0.0, 0.0, -0.01);
                    }
                }
                break;
#endif
#endif
        }
    /*}
    else if (port == JSPORT_KICKER)
    {*/
        switch (maskButton)
        {
            case Logitech_Trigger:
                //
                // Instantaneous kick, no arming required.
                //
                if (fPressed)
                {
                    m_kicker->Kick();
                }
                break;

            case Logitech_Btn2:
                m_kicker->SetArmedState(fPressed);
                break;

            case Logitech_Btn3:
                if (fPressed)
                {
                    fLidUp = !fLidUp;
                    m_lid->SetLidUp(fLidUp);
                }
                break;
        }
    }

    TExit();
}	//NotifyButton
