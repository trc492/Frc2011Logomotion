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
 * provide the notification callback for joystick button events. In addition,
 * it inherits the DigitalInNotify to process any digital input channel
 * changes.
 */
class TrcRobot: public CoopMTRobot,
                public ButtonNotify     //providing the Button handler
{
private:
    //
    // Miscellaneous.
    //
    DriverStationLCD   *m_dsLCD;
    StateMachine       *m_autoSM;
    Compressor         *m_compressor;
    int                 m_numBalls;
    int                 m_fVisionTargetEnabled;

    //
    // Input Subsystem.
    //
    TrcJoystick        *m_driverJoystickLeft;
    TrcJoystick        *m_driverJoystickRight;
    TrcJoystick        *m_kickerJoystick;
    DigitalIn          *m_digitalIn;

    //
    // DriveBase Subsystem.
    //
    Victor             *m_victorFrontLeft;
    Victor             *m_victorRearLeft;
    Victor             *m_victorFrontRight;
    Victor             *m_victorRearRight;
    Event              *m_driveEvent;
    DriveBase          *m_driveBase;

    //
    // Kicker Subsystem.
    //
    Kicker             *m_kicker;

    //
    // Lid Subsystem.
    //
    Lid                *m_lid;

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
        // Miscellaneous.
        //
        m_dsLCD = DriverStationLCD::GetInstance();
        m_autoSM = new StateMachine();
        m_compressor = new Compressor(DIN_CMP_PRESSURE_SW, RELAY_COMPRESSOR);
        m_numBalls = 0;
        m_fVisionTargetEnabled = false;

        //
        // Create Input Subsystem.
        //
        m_driverJoystickLeft = new TrcJoystick(JSPORT_DRIVER_LEFT, this);
        m_driverJoystickRight = new TrcJoystick(JSPORT_DRIVER_RIGHT, this);
        m_kickerJoystick = new TrcJoystick(JSPORT_KICKER, this);
        m_digitalIn = new DigitalIn();

        //
        // Create DriveBase Subsystem.
        //
        m_victorFrontLeft = new Victor(PWM_FRONTLEFT_MOTOR);
        m_victorRearLeft = new Victor(PWM_REARLEFT_MOTOR);
        m_victorFrontRight = new Victor(PWM_FRONTRIGHT_MOTOR);
        m_victorRearRight = new Victor(PWM_REARRIGHT_MOTOR);
        m_driveEvent = new Event();
        m_driveBase = new DriveBase(m_victorFrontLeft,
                                    m_victorRearLeft,
                                    m_victorFrontRight,
                                    m_victorRearRight);

        //
        // Create Kicker Subsystem.
        //
        m_kicker = new Kicker();
        m_digitalIn->RegisterNotification(DInMask(DIN_FEELER_LEFT) |
                                          DInMask(DIN_FEELER_RIGHT),
                                          m_kicker);

        //
        // Create Lid Subsystem.
        //
        m_lid = new Lid();

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
        // Destroy Lid Subsystem.
        //
        SAFE_DELETE(m_lid);

        //
        // Destroy Kicker Subsystem.
        //
        SAFE_DELETE(m_kicker);

        //
        // Destroy DriveBase Subsystem.
        //
        SAFE_DELETE(m_driveBase);
        SAFE_DELETE(m_driveEvent);
        SAFE_DELETE(m_victorRearRight);
        SAFE_DELETE(m_victorFrontRight);
        SAFE_DELETE(m_victorRearLeft);
        SAFE_DELETE(m_victorFrontLeft);

        //
        // Destroy Input Subsystem.
        //
        SAFE_DELETE(m_digitalIn);
        SAFE_DELETE(m_kickerJoystick);
        SAFE_DELETE(m_driverJoystickRight);
        SAFE_DELETE(m_driverJoystickLeft);

        SAFE_DELETE(m_compressor);
        SAFE_DELETE(m_autoSM);

        TExit();
    }   //~TrcRobot

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
        // Initialize subsystems.
        //
        m_compressor->Start();

        TExit();
    }   //RobotInit

    //
    // The following functions are in Auto.h
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
    // The following functions are in TeleOp.h
    //
    void
    TeleOpStart(
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

