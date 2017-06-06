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
#define MOD_ID                  MOD_AUTO
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Auto"

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
    
    m_ladder->Start();
    
    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Mode: Autonomous");
    m_dsLCD->UpdateLCD();

#if 0
    m_driveBase->Init();
    m_ladder->Init();
#endif
    
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

    //m_autoSM->Start();
    TInfo(("Started Autonimouis!"));

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
    m_hangerTimer->CancelTimer();
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
    // Process inputs.
    //
    m_digitalIn->DigitalInTask();
    
    
    m_ladder->Task(CONTEXT_AUTONOMOUS);
    
    //
    // Process state machine.
    //
    if (m_autoSM->IsReady())
    {
        UINT32 currState = m_autoSM->GetCurrentState();

        m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
                            "AutoState=%d",
                            currState - SMSTATE_STARTED);
        switch (currState)
        {
        case SMSTATE_STARTED:
            m_ladder->SetHeight(m_pegHeight - LADDER_INITIAL_HEIGHT);
            // Follow the line.
            m_driveBase->DriveFollowLine(
                (m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1.0:
                (m_lineFollowDir == LNFOLLOW_DIR_FORKRIGHT)? 1.0: 0.0,
                LNFOLLOW_LS_T,
                FOLLOW_LINE_MAXIMUM_SPEED,
                FIND_LINE_DRIVE,
                (((m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1: 1)*
                 FIND_LINE_TURN),
                m_lineFollowEvent,
                4000);
            // Wait for follow line end.
            m_autoSM->WaitForEvents(&m_lineFollowEvent, 1, currState + 2, SMF_WAIT_ALL);
            break;

#if 0
        case SMSTATE_STARTED + 1:   //skip this stage
            // Follow the line.
            m_driveBase->DriveFollowLine(0.0,
                                         LNFOLLOW_LS_T,
                                         FOLLOW_LINE_MAXIMUM_SPEED,
                                         FIND_LINE_DRIVE,
                                         FIND_LINE_TURN,
                                         m_lineFollowEvent,
                                         4000);
            m_autoSM->WaitForEvents(&m_lineFollowEvent, 1, currState + 1, SMF_WAIT_ALL);
            break;
#endif
           
        case SMSTATE_STARTED + 2:
            // Follow the line.
            m_driveBase->DriveFollowLine(
                (m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1.0:
                (m_lineFollowDir == LNFOLLOW_DIR_FORKRIGHT)? 1.0: 0.0,
                LNFOLLOW_LS_T,
                FOLLOW_LINE_SLOW_SPEED,
                FIND_LINE_DRIVE,
                (((m_lineFollowDir == LNFOLLOW_DIR_FORKLEFT)? -1: 1)*
                 FIND_LINE_TURN),
                m_lineFollowEvent,
                0);
            m_autoSM->WaitForEvents(&m_lineFollowEvent, 1, currState + 1, SMF_WAIT_ALL);
            break;

        case SMSTATE_STARTED + 3:
            // Go BACKWARD x SECONDS
            m_driveBase->MecanumDrive_Polar(1.0, 180.0, 0.0);
            m_driveTimer->SetTimer(1.0, m_driveTimerEvent);
            m_autoSM->WaitForEvents(&m_driveTimerEvent, 1, currState + 1, SMF_WAIT_ALL);
            break;

        case SMSTATE_STARTED + 4:
            // Push the tube out.
            m_hanger->SetPush();
            // Set timer.
            m_hangerTimer->SetTimer(1.5, m_hangerTimerEvent);
            // Wait for Timer.
            m_autoSM->WaitForEvents(&m_hangerTimerEvent, 1, currState + 1, SMF_WAIT_ALL);
            break;

        case SMSTATE_STARTED + 5:
            m_driveBase->MecanumDrive_Polar(FOLLOW_LINE_MAXIMUM_SPEED, 180.0, 0.0);
            // Lower ladder.
            m_ladder->Calibrate();
            // Turn 180.
            m_driveBase->DriveSetTarget(0.0, 180.0, true);
            // Wait for turn.
            m_autoSM->WaitForEvents(&m_driveEvent, 1, currState + 1, SMF_WAIT_ALL);
            break;

        case SMSTATE_STARTED + 6:
            // Stop drive.
            m_driveBase->Stop(CONTEXT_NONE);
            //
            // Let it fall through.
            //
        default:
            // Stop state machine.
            m_autoSM->Stop();
            break;
        }
        m_dsLCD->UpdateLCD();
    }

    //
    // Process output tasks.
    //
    m_ladder->Task(CONTEXT_AUTONOMOUS);
    m_driveBase->Task(CONTEXT_AUTONOMOUS);

    //
    // send the dashbaord data associated with the I/O ports
    //
    sendIOPortData();
//    sendVisionData();

    TExit();
}   //AutonomousPeriodic
