#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcRobot.h" />
///
/// <summary>
///     This main module contains the definitions and implementation of the
///     TrcRobot class.
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
 * This class defines and implements the main robot object. It inherits the
 * CoopMTRobot object which is similar to the IterativeRobot class from the
 * WPI library. It also inherits the ButtonNotify interface so that it can
 * provide the notification callback for joystick button events.
 */
class TrcRobot: public CoopMTRobot,
                public ButtonNotify     //providing the Button handler
{
private:
    //
    // Driver Station LCD.
    //
    DriverStationLCD   *m_dsLCD;

    //
    // Used by Autonomous.
    //
    StateMachine       *m_autoSM;
#ifdef _USE_LINE_FOLLOWER
    int                 m_lineFollowDir;
#endif

    //
    // Input Subsystem.
    //
    TrcJoystick        *m_driverJoystickLeft;
    TrcJoystick        *m_driverJoystickRight;
    DigitalIn          *m_digitalIn;
#ifdef _USE_LINE_FOLLOWER
    Solenoid           *m_lightSensorPower;
#endif

    //
    // DriveBase Subsystem.
    //
    Victor             *m_victorFrontLeft;
    Victor             *m_victorRearLeft;
    Victor             *m_victorFrontRight;
    Victor             *m_victorRearRight;
    Event              *m_driveEvent;
#ifdef _USE_LINE_FOLLOWER
    Event              *m_lineFollowEvent;
#endif
    DriveBase          *m_driveBase;

public:
    /**
     * Constructor for the TrcRobot class.
     * Create instances of all the components.
     */
    TrcRobot(
        void
        )
    {
        TLevel(INIT);
        TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);
        TEnter();

        //
        // Driver Station LCD.
        //
        m_dsLCD = DriverStationLCD::GetInstance();

        //
        // Used by Autonomous.
        //
        m_autoSM = new StateMachine();
#ifdef _USE_LINE_FOLLOWER
        m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;
#endif

        //
        // Create Input Subsystem.
        //
        m_driverJoystickLeft = new TrcJoystick(JSPORT_DRIVER_LEFT, this);
        m_driverJoystickRight = new TrcJoystick(JSPORT_DRIVER_RIGHT, this);
        m_digitalIn = new DigitalIn();
#ifdef _USE_LINE_FOLLOWER
        m_lightSensorPower = new Solenoid(SOL_LIGHT_SENSOR_POWER);
        m_lightSensorPower->Set(true);
#endif

        //
        // Create DriveBase Subsystem.
        //
        m_victorFrontLeft = new Victor(PWM_FRONT_LEFT_MOTOR);
        m_victorRearLeft = new Victor(PWM_REAR_LEFT_MOTOR);
        m_victorFrontRight = new Victor(PWM_FRONT_RIGHT_MOTOR);
        m_victorRearRight = new Victor(PWM_REAR_RIGHT_MOTOR);
        m_driveEvent = new Event();
#ifdef _USE_LINE_FOLLOWER
        m_lineFollowEvent = new Event();
#endif
        m_driveBase = new DriveBase(m_victorFrontLeft,
                                    m_victorRearLeft,
                                    m_victorFrontRight,
                                    m_victorRearRight);

        TExit();
    }   //TrcRobot

    /**
     * Destructor for the TrcRobot class.
     * Destroy instances of components that were created in the constructor.
     */
    ~TrcRobot(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Destroy DriveBase Subsystem.
        //
        SAFE_DELETE(m_driveBase);
#ifdef _USE_LINE_FOLLOWER
        SAFE_DELETE(m_lineFollowEvent);
#endif
        SAFE_DELETE(m_driveEvent);
        SAFE_DELETE(m_victorRearRight);
        SAFE_DELETE(m_victorFrontRight);
        SAFE_DELETE(m_victorRearLeft);
        SAFE_DELETE(m_victorFrontLeft);

        //
        // Destroy Input Subsystem.
        //
#ifdef _USE_LINE_FOLLOWER
        SAFE_DELETE(m_lightSensorPower);
#endif
        SAFE_DELETE(m_digitalIn);
        SAFE_DELETE(m_driverJoystickRight);
        SAFE_DELETE(m_driverJoystickLeft);

        SAFE_DELETE(m_autoSM);
        
        TExit();
    }   //~TrcRobot

    //
    // The following functions are in auto.h
    //
    void
    AutonomousStart(
        void
        );
    
    void
    AutonomousStop(
        void
        );
    
    void
    AutonomousPeriodic(
        void
        );

    //
    // The following functions are in teleop.h
    //
    void
    TeleOpStart(
        void
        );
    
    void
    TeleOpStop(
        void
        );
    
    void
    TeleOpPeriodic(
        void
        );

    void
    NotifyButton(
        __in UINT32 port,
        __in UINT16 maskButton,
        __in bool   fPressed
        );
};  //class TrcRobot

