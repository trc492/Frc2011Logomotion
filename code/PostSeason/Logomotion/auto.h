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
    UINT32 bcdSwitch = BCDSwitchValue(m_digitalIn->GetDIO());
    switch (bcdSwitch)
    {
        case BCDSW_STRAIGHT_LOW:
            m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;
            m_pegHeight = LOW_PEG_OFFSET;
            break;

        case BCDSW_STRAIGHT_MID:
            m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;
            m_pegHeight = MID_PEG_OFFSET;
            break;

        case BCDSW_STRAIGHT_HIGH:
            m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;
            m_pegHeight = TOP_PEG_OFFSET;
            break;

        case BCDSW_FORKLEFT_LOW:
            m_lineFollowDir = LNFOLLOW_DIR_FORKLEFT;
            m_pegHeight = LOW_PEG;
            break;

        case BCDSW_FORKLEFT_MID:
            m_lineFollowDir = LNFOLLOW_DIR_FORKLEFT;
            m_pegHeight = MID_PEG;
            break;

        case BCDSW_FORKLEFT_HIGH:
            m_lineFollowDir = LNFOLLOW_DIR_FORKLEFT;
            m_pegHeight = TOP_PEG;
            break;

        case BCDSW_FORKRIGHT_LOW:
            m_lineFollowDir = LNFOLLOW_DIR_FORKRIGHT;
            m_pegHeight = LOW_PEG;
            break;

        case BCDSW_FORKRIGHT_MID:
            m_lineFollowDir = LNFOLLOW_DIR_FORKRIGHT;
            m_pegHeight = MID_PEG;
            break;

        case BCDSW_FORKRIGHT_HIGH:
            m_lineFollowDir = LNFOLLOW_DIR_FORKRIGHT;
            m_pegHeight = TOP_PEG;
            break;

        default:
            m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;
            m_pegHeight = MID_PEG_OFFSET;
            break;
    }
    TInfo(("BCDSwitch=%x: Dir=%d, Height=%f",
           bcdSwitch, m_lineFollowDir, m_pegHeight));

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
    m_autoTimer->CancelTimer();
    m_autoSM->Stop();

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
            // Start raising the ladder.
            //
            m_ladder->SetHeight(m_pegHeight - LADDER_INITIAL_HEIGHT);
            //
            // Start following the line at high speed for 4 seconds.
            //
            m_driveBase->DriveFollowLine(
                0.0,
                INVALID_SENSOR_VALUE,
                FOLLOW_LINE_MAXIMUM_SPEED,
                FIND_LINE_DRIVE,
                (((m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1: 1)*
                 FIND_LINE_TURN),
                m_lineFollowEvent,
                4000);
            //
            // Wait for follow line to end.
            //
            m_autoSM->WaitForEvents(&m_lineFollowEvent, 1, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 1:
            //
            // Continue to follow the line at slow speed until reaching the T.
            //
            m_driveBase->DriveFollowLine(
                0.0,
                LNFOLLOW_LS_T,
                FOLLOW_LINE_SLOW_SPEED,
                FIND_LINE_DRIVE,
                (((m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1: 1)*
                 FIND_LINE_TURN),
                m_lineFollowEvent,
                0);
            //
            // Wait for follow line to end.
            //
            m_autoSM->WaitForEvents(&m_lineFollowEvent, 1, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 2:
            //
            // Backup a little bit.
            //
            m_driveBase->MecanumDrive_Polar(1.0, 180.0, 0.0);
            m_autoTimer->SetTimer(0.5, m_autoTimerEvent);
            m_autoSM->WaitForEvents(&m_autoTimerEvent, 1, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 3:
            //
            // Push the tube out for 1.5 seconds.
            //
            m_hanger->SetPush();
            m_autoTimer->SetTimer(1.5, m_autoTimerEvent);
            //
            // Wait for Timer.
            //
            m_autoSM->WaitForEvents(&m_autoTimerEvent, 1, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 4:
            //
            // Backup and lower the ladder and wait 2 seconds.
            //
            m_driveBase->MecanumDrive_Polar(FOLLOW_LINE_MAXIMUM_SPEED,
                                            180.0,
                                            0.0);
            m_ladder->Calibrate();
            m_autoTimer->SetTimer(2.0, m_autoTimerEvent);
            m_autoSM->WaitForEvents(&m_autoTimerEvent, 1, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 5:
            //
            // Stop drive.
            //
            m_driveBase->Stop();
            //
            // Let it fall through.
            //
        default:
            //
            // Stop state machine.
            //
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
