#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="auto.h" />
///
/// <summary>
///     This main module contains the autonomous mode code.
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
 * This function is called before starting the autonomous mode.
 */
void
TrcRobot::AutonomousStart(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Start autonomous state machine.
    //
    m_numBalls = DIPSwitchValue(m_digitalIn->GetDIO());
    TInfo(("NumBalls=%d", m_numBalls));

    m_kicker->SetArmedState(true);
    m_autoSM->Start();

    TExit();
}   //AutonomousStart

/**
 * This function is called before exiting the autonmous mode.
 */
void
TrcRobot::AutonomousStop(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Stop autonomous mode.
    //
    m_autoSM->Stop();
    m_kicker->SetArmedState(false);

    TExit();
}   //AutonomousStop

/**
 * This function is called periodically at fixed intervals in autonomous
 * mode.
 */
void
TrcRobot::AutonomousPeriodic(
    void
    )
{
    TLevel(HIFREQ);
    TEnter();

    //
    // Process state machine.
    //
    if (m_autoSM->IsReady())
    {
        UINT32 currState = m_autoSM->GetCurrentState();

        m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,
                            "AutoState=%d",
                            currState - SMSTATE_STARTED);
        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Get to the actual starting position.
                //
                m_driveBase->EnableAccel(true);
                m_driveBase->DriveSetTarget(0.0, 72.0, 0.0, true, m_driveEvent);
                m_autoSM->WaitForEvents(&m_driveEvent, 1, currState + 1, 0);
                break;

            case SMSTATE_STARTED + 1:
                //
                // Move forward and kick the ball. Since the kicker is armed,
                // the moment it touches the ball, the kicker will kick.
                //
                m_driveBase->EnableAccel(true);
                m_driveBase->VisionDriveSetTarget(0.0, 36.0, true, m_driveEvent);
                m_autoSM->WaitForEvents(&m_driveEvent, 1, currState + 1, 0);
                break;

            case SMSTATE_STARTED + 2:
                //
                // Move backward to the left to get in place for the next ball.
                //
                if (--m_numBalls)
                {
                    m_driveBase->EnableAccel(true);
                    m_driveBase->DriveSetTarget(-36.0, -36.0, 0.0, true, m_driveEvent);
                    m_autoSM->WaitForEvents(&m_driveEvent, 1, currState - 1, 0);
                }
                else
                {
                    m_driveBase->EnableAccel(false);
                    m_autoSM->SetCurrentState(currState + 1);
                }
                break;

            default:
                m_autoSM->Stop();
                break;
        }
    }

    //
    // send the dashbaord data associated with the I/O ports
    //
    sendIOPortData();

    TExit();
}   //AutonomousPeriodic
