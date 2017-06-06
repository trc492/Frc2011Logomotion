#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CoopMTRobot.h" />
///
/// <summary>
///     This module contains the definitions of the CoopMTRobot class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _COOPMTTASK_H
#define _COOPMTTASK_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_COOPMTROBOT
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "CoopMTRobot"

/**
 * This class defines and implements the CoopMTRobot object. The CoopMTRobot
 * object implements a cooperative multitasking robot. It inherits the
 * subsystem manager that allows registration of subsystems. In the robot
 * loop at various points, it will do callbacks to the registered subsystems.
 * This basically simulates a cooperative multitasking scheduler that task
 * switches between different subsystems allowing them to execute in various
 * contexts.
 */
class CoopMTRobot: public RobotBase
{
private:
    SubSystemMgr   *m_subsysMgr;
    double          m_loopPeriod;
    Timer           m_loopTimer;
    UINT32          m_periodPacket;     //in ms

    /**
     * This function is called to determine if the next period has
     * began so that the periodic functions should be called.
     * If m_loopPeriod > 0.0, call the periodic function every
     * m_loopPeriod as compared to Timer.Get(). If m_loopPeriod == 0.0,
     * call the periodic functions whenever a packet is received
     * from the Driver Station, or about every 20ms.
     *
     * @return Returns true if the next period is ready, false otherwise.
     */
    bool
    NextPeriodReady(
        void
        )
    {
        bool rc;

        TLevel(HIFREQ);
        TEnter();

        static UINT32 timestamp = 0;
        if (m_loopPeriod > 0.0)
        {
            rc = m_loopTimer.HasPeriodPassed(m_loopPeriod);
        }
        else
        {
            rc = m_ds->IsNewControlData();
            if (rc)
            {
                //
                // Determine the packet interval.
                //
                UINT32 timeCurr = GetMsecTime();
                m_periodPacket = timeCurr - timestamp;
                timestamp = timeCurr;
            }
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //NextPeriodReady

public:
    /*
     * The default period for the periodic function calls (seconds)
     * Setting the period to 0.0 will cause the periodic functions to
     * follow the Driver Station packet rate of about 50Hz.
     */
    static const double kDefaultPeriod = 0.0;

    /**
     * This function is called one time to do robot-wide initialization.
     */
    virtual
    void
    RobotInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //RobotInit

    /**
     * This function is called before entering autonomous mode.
     */
    virtual
    void
    AutonomousStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //AutonomousStart

    /**
     * This function is called before exiting autonomous mode.
     */
    virtual
    void
    AutonomousStop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //AutonomousStop

    /**
     * This function is called periodically at fixed intervals in autonomous
     * mode.
     */
    virtual
    void
    AutonomousPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //AutonomousPeriodic

    /**
     * This function is called continuously in autonomous mode.
     */
    virtual
    void
    AutonomousContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //AutonomousContinuous

    /**
     * This function is called before entering teleop mode.
     */
    virtual
    void
    TeleOpStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TeleOpStart

    /**
     * This function is called before exiting teleop mode.
     */
    virtual
    void
    TeleOpStop(
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
    virtual
    void
    TeleOpPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TeleOpPeriodic

    /**
     * This function is called continuously in teleop mode.
     */
    virtual
    void
    TeleOpContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TeleOpContinuous

    /**
     * This function sets the period for the periodic functions.
     *
     * @param period The period of the periodic function calls.
     *        0.0 means sync to driver station control data.
     */
    void
    SetPeriod(
        __in double period
        )
    {
        TLevel(API);
        TEnterMsg(("period=%f", period));

        if (period != 0.0)
        {
            //
            // Not sync with DS, so start the timer for the main loop.
            //
            m_loopTimer.Reset();
            m_loopTimer.Start();
        }
        else
        {
            //
            // Sync with DS, don't need timer.
            //
            m_loopTimer.Stop();
        }
        m_loopPeriod = period;

        TExit();
    }   //SetPeriod
    
    /**
     * This function gets the period for the periodic functions.
     *
     * @return Returns period of the periodic function calls.
     */
    double
    GetPeriod(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double period = (m_loopPeriod == 0.0)?
                        ((double)m_periodPacket)/1000.0: m_loopPeriod;
        TExitMsg(("=%f", period));
        return period;
    }   //GetPeriod

    /**
     * This function gets the number of loops per second for the
     * main loop.
     *
     * @return Returns frequency of the periodic function calls.
     */
    double
    GetLoopsPerSec(
        void
        )
    {
        double freq;

        TLevel(API);
        TEnter();

        freq = 1.0/GetPeriod();

        TExitMsg(("=%f", freq));
        return freq;
    }   //GetLoopsPerSec

    /**
     * Start a competition.
     * This specific StartCompetition() implements "main loop" behavior like
     * that of the FRC control system in 2008 and earlier, with a primary
     * (slow) loop that is called periodically, and a "fast loop" (a.k.a.
     * "spin loop") that is called as fast as possible with no delay between
     * calls. This code needs to track the order of the field starting to
     * ensure that everything happens in the right order. Repeatedly run the
     * correct method, either Autonomous or OperatorControl when the robot is
     * enabled. After running the correct method, wait for some state to
     * change, either the other mode starts or the robot is disabled. Then go
     * back and wait for the robot to be enabled again.
     */
    void
    StartCompetition(
        void
        )
    {
        TLevel(API);
        TEnter();

        UINT32 context = CONTEXT_DISABLED;
        UINT32 timeAutonomousPeriod = 0;
        UINT32 timeTeleOpPeriod = 0;
        UINT32 cntLoops = 0;
        UINT32 timeBegin = 0;
        UINT32 timeUsed = 0;
        DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

        //
        // One-time robot initialization.
        //
        TEnable(true);

        //
        // Disable the watchdog while doing initialization.
        //
        GetWatchdog().SetEnabled(false);

        RobotInit();
        m_subsysMgr->InitSubSystems();

        //
        // Set normal watchdog timeout.
        //
        GetWatchdog().Feed();
        GetWatchdog().SetExpiration(0.5);
        GetWatchdog().SetEnabled(true);

        TEnable(true);

        //
        // Loop forever, calling the appropriate mode-dependent functions.
        //
        while (true)
        {
            GetWatchdog().Feed();

            //TPeriodStart();

            dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,
                              "Context=%d", context);
            dsLCD->UpdateLCD();

            switch (context)
            {
                case CONTEXT_DISABLED:
                    if (IsEnabled())
                    {
                        TInfo(("Entering enabled state."));
                        if (IsAutonomous())
                        {
                            GetWatchdog().Feed();
                            context = CONTEXT_AUTONOMOUS;
                            TInfo(("Starting autonomous."));
                            AutonomousStart();
                        }
                        else
                        {
                            GetWatchdog().Feed();
                            context = CONTEXT_TELEOP;
                            TInfo(("Starting teleop."));
                            TeleOpStart();
                        }
                        TInfo(("Starting subsystems."));
                        m_subsysMgr->StartSubSystems(context);
                    }
                    break;

                case CONTEXT_AUTONOMOUS:
                    if (IsEnabled() && IsAutonomous())
                    {
                        GetWatchdog().Feed();
                        if (NextPeriodReady())
                        {
                            GetWatchdog().Feed();
                            timeBegin = GetMsecTime();
                            m_subsysMgr->SubSystemInputTasks(context);
                            AutonomousPeriodic();
                            m_subsysMgr->SubSystemActionTasks(context);
                            timeUsed = GetMsecTime() - timeBegin;
                            timeAutonomousPeriod += timeUsed;
                            cntLoops++;
                            if ((float)timeUsed/1000.0 > GetPeriod()*0.9)
                            {
                                GetWatchdog().Feed();
                                //
                                // Execution time exceeds 90% of the period.
                                //
                                TWarn(("Autonomous period executed too long (%d ms)",
                                       timeUsed));
                            }
                        }
                        GetWatchdog().Feed();
                        AutonomousContinuous();
                    }
                    else
                    {
                        GetWatchdog().Feed();
                        m_subsysMgr->StopSubSystems(context);
                        AutonomousStop();
                        context = CONTEXT_DISABLED;
                    }
                    break;

                case CONTEXT_TELEOP:
                    GetWatchdog().Feed();
                    if (IsEnabled() && IsOperatorControl())
                    {
                        GetWatchdog().Feed();
                        if (NextPeriodReady())
                        {
                            GetWatchdog().Feed();
                            timeBegin = GetMsecTime();
                            TInfo(("TeleOp Input Task:"));
                            m_subsysMgr->SubSystemInputTasks(context);
                            TInfo(("TeleOp Periodic:"));
                            TeleOpPeriodic();
                            TInfo(("TeleOp Action Tasks:"));
                            m_subsysMgr->SubSystemActionTasks(context);
                            timeUsed = GetMsecTime() - timeBegin;
                            timeTeleOpPeriod += timeUsed;
                            cntLoops++;
                            if ((float)timeUsed/1000.0 > GetPeriod()*0.9)
                            {
                                GetWatchdog().Feed();
                                //
                                // Execution time exceeds 90% of the period.
                                //
                                TWarn(("TeleOp period executed too long (%d ms)",
                                       timeUsed));
                            }
                        }
                        GetWatchdog().Feed();
                        TeleOpContinuous();
                    }
                    else
                    {
                        GetWatchdog().Feed();
                        m_subsysMgr->StopSubSystems(context);
                        TeleOpStop();
                        context = CONTEXT_DISABLED;
                    }
                    break;
            }

            //TPeriodEnd();
        }

        TExit();
    }   //StartCompetition

protected:
    /**
     * Constructor for the TrcRobot class.
     */
    CoopMTRobot(
        void
        ): m_loopPeriod(kDefaultPeriod),
           m_periodPacket(0)
    {
        TLevel(INIT);
        TEnter();

        m_subsysMgr = SubSystemMgr::GetInstance();
        m_watchdog.SetEnabled(false);

        TExit();
    }   //CoopMTRobot

    /**
     * Destructor for the TrcRobot class.
     */
    ~CoopMTRobot(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SubSystemMgr::DeleteInstance();

        TExit();
    }   //~CoopMTRobot
};  //class CoopMTRobot

#endif  //ifndef _COOPMTTASK_H
