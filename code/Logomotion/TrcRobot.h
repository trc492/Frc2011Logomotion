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
                public ButtonNotify,    //providing the Button handler
                public DigitalInNotify  //providing the DigitalIn handler
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
    TrcTimer           *m_driveTimer;
    Event              *m_driveTimerEvent;

    //
    // Ladder Subsystem.
    //
    Ladder             *m_ladder;
    Event              *m_setHeightEvent;

    //
    // Hanger Subsystem.
    //
    Hanger             *m_hanger;
    TrcTimer           *m_hangerTimer;
    Event              *m_hangerTimerEvent;

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
        m_digitalIn = new DigitalIn(DInMask(DIN_HANGER_SWITCH) | 
                                    DInMask(DIN_LADDER_SWITCH),
                                    this);
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
                                    m_victorRearRight,
                                    m_driveEvent,
                                    m_lineFollowEvent);
        m_driveTimer = new TrcTimer();
        m_driveTimerEvent = new Event();

        //
        // Create Ladder Subsystem.
        //
        m_ladder = new Ladder();
        m_setHeightEvent = new Event();

        //
        // Create Hanger Subsystem.
        //
        m_hanger = new Hanger();
        m_hangerTimer = new TrcTimer();
        m_hangerTimerEvent = new Event();

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
        SAFE_DELETE(m_hangerTimerEvent);
        SAFE_DELETE(m_hangerTimer);
        SAFE_DELETE(m_hanger);

        //
        // Destroy Ladder Subsystem.
        //
        SAFE_DELETE(m_setHeightEvent);
        SAFE_DELETE(m_ladder);

        //
        // DriveBase Subsystem.
        //
        SAFE_DELETE(m_driveTimerEvent);
        SAFE_DELETE(m_driveTimer);
        SAFE_DELETE(m_driveBase);
        SAFE_DELETE(m_lineFollowEvent);
        SAFE_DELETE(m_driveEvent);
        SAFE_DELETE(m_victorRearRight);
        SAFE_DELETE(m_victorFrontRight);
        SAFE_DELETE(m_victorRearLeft);
        SAFE_DELETE(m_victorFrontLeft);

        //
        // Input Subsystem.
        //
        SAFE_DELETE(m_lightSensorPower);
        SAFE_DELETE(m_digitalIn);
        SAFE_DELETE(m_hangerJoystick);
        SAFE_DELETE(m_driverJoystickRight);
        SAFE_DELETE(m_driverJoystickLeft);

        SAFE_DELETE(m_autoSM);
        
        AxisCamera::GetInstance().DeleteInstance();

        TExit();
    }   //~TrcRobot

    /**
     * This function is called by the DigitalIn object when a DigitalIn event
     * occurred.
     *
     * @param slot Specifies the Digital Module slot.
     * @param channel Specifies the DigitalIn channel that changed state.
     * @param fActive If true, specifies the DigitalIn channel is active,
     *        false if released.
     */
    void
    NotifyDIn(
        __in UINT32 slot,
        __in UINT32 channel,
        __in bool   fActive
        )
    {
        TLevel(EVENT);
        TEnterMsg(("slot=%d,channel=%d,fActive=%d", slot, channel, fActive));

        if (slot == SensorBase::GetDefaultDigitalModule())
        {
            switch (channel)
            {
            case DIN_LADDER_SWITCH:
                //
                // Stop ladder when it hits the bottom.
                //
                m_ladder->LadderNotify(fActive);
                break;
                
            case DIN_HANGER_SWITCH:
                //
                // Stop the jaws when the tube is in.
                //
                m_hanger->HangerNotify(fActive);
                break;
            }
        }

        TExit();
    }   //NotifyDIn

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

#if 0
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
        // Initialize all subsystems.
        //
        m_deployer->Init();
        m_hanger->Init();
        m_ladder->Init();
        m_driveBase->Init();

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

        m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line1,
                            "%s %s",
                            __DATE__, __TIME__);
        m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,
                            "Stage: Disabled");
        m_dsLCD->UpdateLCD();
        
        //
        // Disable all subsystems.
        //
        m_deployer->Stop();
        m_hanger->Stop();
        m_ladder->Stop();
        m_driveBase->Stop();
 
//        m_lightSensorPower->Set(false);

        TExit();
    }   //DisabledInit

    void
    AutonomousInit(
        void
        );

    void
    TeleopInit(
        void
        );

    void
    AutonomousStop(
        void
        );
#endif
};  //class TrcRobot

