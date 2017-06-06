#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DriveCal.cpp" />
///
/// <summary>
///     This program does the base drive calibration.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

//
// Library includes.
//
#include "WPILib.h"

#define _DBGTRACE_ENABLED
#include "..\trclib\TrcLib.h"

//
// Tracing info.
//
#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

#define MOD_BASEDRIVE           TGenModId(1)

#define TRACE_MODULES           (MOD_MAIN | MOD_BASEDRIVE)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// Project includes.
//
#include "..\main\DashboardDataSender.h"
#include "..\main\RobotInfo.h"
#include "..\main\DriveBase.h"

/**
 * This class defines and implements the main robot object. It inherits the
 * TrcRobot object which is identical to the IterativeRobot class from the
 * WPI library. It also inherits the ButtonNotify interface so that it can
 * provide the notification callback for joystick button events.
 */
class Robot: public TrcRobot, public PIDDriveNotify, public ButtonNotify
{
private:
    DashboardDataSender *m_dds;
    //
    // Drive subsystem.
    //
    Victor     *m_victorLeft;
    Victor     *m_victorRight;
    DriveBase  *m_driveBase;
    //
    // Input subsystem.
    //
    TrcJoystick *m_jsDriveLeft;
    TrcJoystick *m_jsDriveRight;

    bool        m_fDriveCal;

public:
    /**
     * Constructor for the Robot class.
     * Create instances of all the components.
     */
    Robot(
        void
        )
    {
        TLevel(INIT);
        TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);
        TEnter();

        m_dds = new DashboardDataSender();
        //
        // Drive subsystem.
        //
        m_victorLeft = new Victor(PWMDRIVE_LEFT);
        m_victorRight = new Victor(PWMDRIVE_RIGHT);
        m_driveBase = new DriveBase(m_victorLeft,
                                    m_victorRight,
                                    this);
        //
        // Input subsystem.
        //
        m_jsDriveLeft = new TrcJoystick(JSPORT_DRIVE_LEFT, this);
        m_jsDriveRight = new TrcJoystick(JSPORT_DRIVE_RIGHT, this);

        m_fDriveCal = false;

        TExit();
    }   //Robot

    /**
     * Destructor for the Robot class.
     * Destroy instances of components that were created in the constructor.
     */
    ~Robot(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Input subsystem.
        //
        SAFE_DELETE(m_jsDriveLeft);
        SAFE_DELETE(m_jsDriveRight);
        //
        // Drive subsystem.
        //
        SAFE_DELETE(m_driveBase);
        SAFE_DELETE(m_victorRight);
        SAFE_DELETE(m_victorLeft);

        SAFE_DELETE(m_dds);

        TExit();
    }   //~Robot

    /**
     * This function is called one time to do robot-wide initialization.
     */
    void
    RobotInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Enable the watchdog with long timeout while doing initialization.
        //
        GetWatchdog().SetExpiration(30.0);
        GetWatchdog().SetEnabled(true);
        //
        // Initialize subsystems.
        //
        m_driveBase->DriveInit();
        //
        // Set normal watchdog timeout.
        //
        GetWatchdog().SetExpiration(4.0);
        GetWatchdog().SetEnabled(true);

        TExit();
    }   //RobotInit

    /**
     * This function is called before starting the disabled loop. This is
     * used to stop the drive subsystem if we exit out of autonomous or teleop.
     */
    void
    DisabledInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_driveBase->DriveStop();
        m_fDriveCal = false;

        TExit();
    }   //DisabledInit

    /**
     * This function is called before starting the autonomous mode.
     */
    void
    AutonomousInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //AutonomousInit

    /**
     * This function is called periodically at fixed intervals in autonomous
     * mode.
     */
    void
    AutonomousPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        GetWatchdog().Feed();
        TExit();
    }   //AutonomousPeriodic

    /**
     * This function is called before starting the teleop mode.
     */
    void
    TeleopInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TeleopInit

    /**
     * This function is called periodically at fixed intervals in teleop mode.
     */
    void
    TeleopPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        GetWatchdog().Feed();
        //
        // Process inputs and sensors.
        //
        m_jsDriveLeft->ButtonTask();
        m_jsDriveRight->ButtonTask();

        //
        // Process outputs.
        //
        if (m_fDriveCal)
        {
            m_driveBase->PIDDriveTask();
        }
        else
        {
            m_driveBase->TankDrive(m_jsDriveLeft->GetYWithDeadband(),
                                   m_jsDriveRight->GetYWithDeadband());
        }
        // send the dashbaord data associated with the I/O ports
        m_dds->sendIOPortData();

        TExit();
    }   //TeleopPeriodic

    /**
     * This function is called by the TrcPIDDrive object when a PID drive
     * completion event occurred.
     *
     * @param pidDrive Points to the PIDDrive object that initiated the
     *        notification.
     */
    void
    NotifyPIDDrive(
        __in TrcPIDDrive *pidDrive
        )
    {
        TLevel(EVENT);
        TEnterMsg(("pidDrive=%p", pidDrive));

        m_fDriveCal = false;

        TExit();
    }  //NotifyPIDDrive

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
    NotifyButton(
        __in UINT32 port,
        __in UINT16 maskButton,
        __in bool   fPressed
        )
    {
        TLevel(EVENT);
        TEnterMsg(("port=%d,Button=0x%x,fPressed=%d",
                   port, maskButton, fPressed));

        if (port == JSPORT_DRIVE_LEFT)
        {
            switch (maskButton)
            {
                case Logitech_Btn1:
                    if (fPressed)
                    {
                        if (m_fDriveCal)
                        {
                            m_fDriveCal = false;
                            m_driveBase->DriveStop();
                            TInfo(("Stop calibration."));
                        }
                        else
                        {
                            m_fDriveCal = true;
                            m_driveBase->DriveSetTarget(96.0, DRIVE_TOLERANCE,
                                                        0.0, TURN_TOLERANCE,
                                                        true);
                            TInfo(("Start drive calibration."));
                        }
                    }
                    break;

                case Logitech_Btn2:
                    if (fPressed)
                    {
                        if (m_fDriveCal)
                        {
                            m_fDriveCal = false;
                            m_driveBase->DriveStop();
                            TInfo(("Stop calibration."));
                        }
                        else
                        {
                            m_fDriveCal = true;
                            m_driveBase->DriveSetTarget(0.0, DRIVE_TOLERANCE,
                                                        360.0, TURN_TOLERANCE);
                            TInfo(("Start turn calibration."));
                        }
                    }
                    break;
                }
        }

        TExit();
    }   //NotifyButton
};  //class Robot

START_ROBOT_CLASS(Robot);
