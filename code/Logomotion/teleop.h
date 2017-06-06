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
#define MOD_ID                  MOD_TELEOP
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TeleOp"

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
    
    m_ladder->Start();

    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,
                        "Mode: TeleOp");
    m_dsLCD->UpdateLCD();

    AxisCamera::GetInstance();

#if 0
    //
    // Stop autonomous mode.
    //
    AutonomousStop();
    m_ladder->Init();
//    m_lightSensorPower->Set(true);
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
    static bool fJoystickOverride = false;

    TLevel(HIFREQ);
    TEnter();

    //
    // Process inputs.
    //
    m_driverJoystickLeft->ButtonTask();
    m_driverJoystickRight->ButtonTask();
    m_hangerJoystick->ButtonTask();
    m_digitalIn->DigitalInTask();

    //
    // Process outputs and subsystems.
    //
    float x = m_driverJoystickLeft->GetXWithDeadband();
    float y = m_driverJoystickRight->GetYWithDeadband();
    float speed;
    float rotation;

    speed = MAGNITUDE(x, y);
    rotation = m_driverJoystickRight->GetXWithDeadband();
    if (m_driveSlowSpeed)
    {
        speed *= DRIVE_SLOW_SPEED_CONSTANT;
        rotation *= DRIVE_SLOW_SPEED_CONSTANT;
    }
    m_driveBase->MecanumDrive_Polar(speed, DIR_DEGREES(x, -y), rotation);
    
    float z = m_hangerJoystick->GetYWithDeadband();
    
    //
    // Process output tasks.
    //
    if (z != 0.0)
    {
        // Joystick is contolling the ladder.
        fJoystickOverride = true;
        m_ladder->SetSpeed(z, true);
    }
    else if (fJoystickOverride)
    {
        // Exit joystick control.
        m_ladder->SetSpeed(0.0, false);
        fJoystickOverride = false;
    }
    else
    {
        m_ladder->Task(CONTEXT_TELEOP);
    }
#if 0
    else
    {
        m_ladder->Task();
    }

    m_hanger->Task();
    m_driveBase->Task();
#endif
    static DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
    dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Height:%f", m_ladder->GetHeight());
    dsLCD->UpdateLCD();

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
            // deploy minibot
        case Logitech_Btn6:
            if (fPressed)
            {
                m_deployer->SetDeploy(true);
            }
            break;
            
        case Logitech_Btn7:
            if (fPressed)
            {
                m_deployer->SetDeploy(false);
            }
            break;

            case Logitech_Btn2:
                if (fPressed)
                {
                    m_driveBase->DriveFollowLine(0.0,
                                                 LNFOLLOW_LS_T,
                                                 FOLLOW_LINE_MAXIMUM_SPEED,
                                                 FIND_LINE_DRIVE,
                                                 FIND_LINE_TURN,
                                                 m_lineFollowEvent);
                    TInfo(("Starting line following sequence"));
                }
                else
                {
                    m_driveBase->Stop(CONTEXT_NONE);
                    TInfo(("FINISHED!!!~!~~~!@~!@#~~"));
                }
                break;

            case Logitech_Btn3:
                m_driveSlowSpeed = fPressed;
                break;
        }
    }
    else if (port == JSPORT_DRIVER_RIGHT)
    {
        switch (maskButton)
        {
        }
    }
    else if (port == JSPORT_HANGER)
    {
        switch (maskButton)
        {
            case Logitech_Trigger:
                // only pull in while holding down on the trigger
                if (fPressed)
                {
                    m_hanger->SetPull();
                }
                else
                {
                    m_hanger->Stop(CONTEXT_NONE);
                }
                break;

            case Logitech_Btn2:
                // only push out while holding down button 2
                if (fPressed)
                {
                    m_hanger->SetPush();
                }
                else
                {
                    m_hanger->Stop(CONTEXT_NONE);
                }
                break;

            case Logitech_Btn5:
                // flip up tube
                if(fPressed)
                {
                    m_hanger->FlipUp();
                }
                else
                {
                    m_hanger->Stop(CONTEXT_NONE);
                }
                break;

            case Logitech_Btn3:
                // flip down tube
                if(fPressed)
                {
                    m_hanger->FlipDown();
                }
                else
                {
                    m_hanger->Stop(CONTEXT_NONE);
                }
                break;

            case Logitech_Btn6:
                // feeding slot
                if(fPressed)
                {
                    m_ladder->SetHeight(FEEDING_SLOT);
                }
                break;

            case Logitech_Btn4:
                // ground
                if(fPressed)
                {
                    m_ladder->Calibrate();
                }
                break;

            case Logitech_Btn11:
                // buttons to raise ladder to fixed heights
                if (fPressed)
                {
                    m_ladder->SetHeight(LOW_PEG);
                }
                break;

            case Logitech_Btn9:
                if (fPressed)
                {
                    m_ladder->SetHeight(MID_PEG);
                }
                break;

            case Logitech_Btn7:
                if (fPressed)
                {
                    m_ladder->SetHeight(TOP_PEG);
                }
                break;

            case Logitech_Btn12:
                if (fPressed)
                {
                    m_ladder->SetHeight(LOW_PEG_OFFSET);
                }
                break;

            case Logitech_Btn10:
                if (fPressed)
                {
                    m_ladder->SetHeight(MID_PEG_OFFSET);
                }
                break;

            case Logitech_Btn8:
                if (fPressed)
                {
                    m_ladder->SetHeight(TOP_PEG_OFFSET);
                }
                break;
        }
    }

    TExit();
}	//NotifyButton
