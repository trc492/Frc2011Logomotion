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
 * changes.
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
    TrcTimer           *m_autoTimer;
    Event              *m_autoTimerEvent;
    float               m_pegHeight;
    int                 m_lineFollowDir;

    //
    // Used by TeleOp.
    //
    bool                m_driveSlowSpeed;

    //
    // Input Subsystem.
    //
    TrcJoystick        *m_driverJoystickLeft;
    TrcJoystick        *m_driverJoystickRight;
    TrcJoystick        *m_hangerJoystick;
    DigitalIn          *m_digitalIn;
    Solenoid           *m_lightSensorPower;

    //
    // DriveBase Subsystem.
    //
    Victor             *m_victorFrontLeft;
    Victor             *m_victorRearLeft;
    Victor             *m_victorFrontRight;
    Victor             *m_victorRearRight;
    Event              *m_driveEvent;
    Event              *m_lineFollowEvent;
    DriveBase          *m_driveBase;

    //
    // Ladder Subsystem.
    //
    Ladder             *m_ladder;
    Event              *m_setHeightEvent;

    //
    // Hanger Subsystem.
    //
    Hanger             *m_hanger;

    //
    // Deployer Subsystem.
    //
    Deployer           *m_deployer;
    
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
        m_autoTimer = new TrcTimer();
        m_autoTimerEvent = new Event();
        m_pegHeight = MID_PEG_OFFSET;
        m_lineFollowDir = LNFOLLOW_DIR_STRAIGHT;

        //
        // Used by TeleOp.
        //
        m_driveSlowSpeed = false;

        //
        // Create Input Subsystem.
        //
        m_driverJoystickLeft = new TrcJoystick(JSPORT_DRIVER_LEFT, this);
        m_driverJoystickRight = new TrcJoystick(JSPORT_DRIVER_RIGHT, this);
        m_hangerJoystick = new TrcJoystick(JSPORT_HANGER, this);
        m_digitalIn = new DigitalIn();
        m_lightSensorPower = new Solenoid(SOL_LIGHT_SENSOR_POWER);
        m_lightSensorPower->Set(true);

        //
        // Create DriveBase Subsystem.
        //
        m_victorFrontLeft = new Victor(PWM_FRONTLEFT_MOTOR);
        m_victorRearLeft = new Victor(PWM_REARLEFT_MOTOR);
        m_victorFrontRight = new Victor(PWM_FRONTRIGHT_MOTOR);
        m_victorRearRight = new Victor(PWM_REARRIGHT_MOTOR);
        m_driveEvent = new Event();
        m_lineFollowEvent = new Event();
        m_driveBase = new DriveBase(m_victorFrontLeft,
                                    m_victorRearLeft,
                                    m_victorFrontRight,
                                    m_victorRearRight);

        //
        // Create Ladder Subsystem.
        //
        m_ladder = new Ladder();
        m_setHeightEvent = new Event();
        m_digitalIn->RegisterNotification(DInMask(DIN_LADDER_SWITCH), m_ladder);

        //
        // Create Hanger Subsystem.
        //
        m_hanger = new Hanger();
        m_digitalIn->RegisterNotification(DInMask(DIN_HANGER_SWITCH), m_hanger);

        //
        // Create Deployer Subsystem.
        //
        m_deployer = new Deployer();

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
        // Destroy Deployer Subsystem.
        //
        SAFE_DELETE(m_deployer);

        //
        // Destroy Hanger Subsystem.
        //
        SAFE_DELETE(m_hanger);

        //
        // Destroy Ladder Subsystem.
        //
        SAFE_DELETE(m_setHeightEvent);
        SAFE_DELETE(m_ladder);

        //
        // Destroy DriveBase Subsystem.
        //
        SAFE_DELETE(m_driveBase);
        SAFE_DELETE(m_lineFollowEvent);
        SAFE_DELETE(m_driveEvent);
        SAFE_DELETE(m_victorRearRight);
        SAFE_DELETE(m_victorFrontRight);
        SAFE_DELETE(m_victorRearLeft);
        SAFE_DELETE(m_victorFrontLeft);

        //
        // Destroy Input Subsystem.
        //
        SAFE_DELETE(m_lightSensorPower);
        SAFE_DELETE(m_digitalIn);
        SAFE_DELETE(m_hangerJoystick);
        SAFE_DELETE(m_driverJoystickRight);
        SAFE_DELETE(m_driverJoystickLeft);

        SAFE_DELETE(m_autoTimerEvent);
        SAFE_DELETE(m_autoTimer);
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

