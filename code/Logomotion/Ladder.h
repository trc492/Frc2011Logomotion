#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Ladder.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Ladder class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_LADDER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Ladder"

/**
 * This module defines and implements the Ladder subsytem. The Ladder
 * subsystem consists of a tilting motor that tilts the ladder forward
 * or backward and two limit switches to detect if the ladder has reached
 * full forward or backward position so it can stop the tilting motor.
 * It also consists of a winch motor that raises and lowers the ladder
 * through a pulley system. In order to detect the height of the ladder,
 * an encoder is installed in the path of the pulley system.
 */
class Ladder: public SubSystem,
              public PIDInput
{
private:
    Victor          *m_winchMotor;
    Encoder         *m_winchEncoder;
    TrcPIDCtrl      *m_winchPIDCtrl;
    TrcPIDMotor     *m_winchPIDMotor;
    bool             m_fOverride;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Ladder(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Ladder winch.
        //
        m_winchMotor = new Victor(PWM_LADDERHEIGHT_MOTOR);
        m_winchEncoder = new Encoder(DIN_LADDER_ENCODER_A,
                                     DIN_LADDER_ENCODER_B);
        m_winchEncoder->SetDistancePerPulse(HEIGHT_PER_CLICK);
        m_winchPIDCtrl = new TrcPIDCtrl(LADDER_KP, LADDER_KI, LADDER_KD,
                                        LADDER_TOLERANCE, LADDER_SETTLING,
                                        PIDCTRLO_ABS_SETPT);
        m_winchPIDMotor = new TrcPIDMotor(m_winchMotor, m_winchPIDCtrl, this);
        m_fOverride = false;
        RegisterSubSystem(SUBSYS_START | SUBSYS_STOP | SUBSYS_ACTION_TASKS);

        TExit();
    }   //Ladder
        
    /**
     * This function is called from the PID controllers to get the PID input
     * value.
     *
     * @param pidCtrl Specifies the PID controller that requires the input
     *        value.
     */
    float
    GetInput(
        __in TrcPIDCtrl *pidCtrl
        )
    {
        float value;

        TLevel(HIFREQ);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        value = (float)m_winchEncoder->GetDistance();

        TExitMsg(("=%f", value));
        return value;
    }    //GetInput

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Ladder(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        
        SAFE_DELETE(m_winchPIDMotor);
        SAFE_DELETE(m_winchPIDCtrl);
        SAFE_DELETE(m_winchEncoder);
        SAFE_DELETE(m_winchMotor);

        TExit();
    }   //~Ladder
    
    /**
     * This function is called by the Disabled or Initialize code to stop the
     * subsystem.
     * 
     * @param context Specifies the calling context (autonomous or teleop).
     */
    void
    Stop(
        __in UINT32 context
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("context=%d", context));

        m_winchEncoder->Stop();
        m_winchPIDMotor->Stop();

        TExit();
    }   //Stop

    /**
     * This function gets called for either bringing the ladder all the way
     * to the ground, or to calibrate the encoder on the ladder
     */
    void
    Calibrate(
        void
        )
    {
        TLevel(API);
        TEnter();

        Stop(CONTEXT_NONE);
        m_winchMotor->Set(-0.5);

        TExit();
    }

    /**
     * This function is called one time to do subsystem initialization.
     *
     * @param tiltForwardTrigger Specifies the initial tilt forward trigger
     *        state.
     * @param tiltBackwardTrigger Specifies the initial tilt backward trigger
     *        state.
     */
    void
    Start(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TInfo(("Ladder Started"));
        
        m_winchEncoder->Reset();
        m_winchEncoder->Start();

        TExit();
    }   //Start

    /**
     * This function is called periodically to perform the subsystem tasks.
     * 
     * @param context Specifies the calling context (autonomous or teleop).
     */
    void
    Task(
        __in UINT32 context
        )
    {
        TLevel(TASK);
        TEnterMsg(("context=%d", context));

        if (!m_fOverride)
        {
            m_winchPIDMotor->Task();
            TSampling(("Encoder clicks: %d, distance: %f",
                       m_winchEncoder->GetRaw(),
                       m_winchEncoder->GetDistance()));
        }
        

        TExit();
    }   //Task
    
    /**
     * This function sets the ladder height in inches.
     *
     * @param height Specifies the ladder height in inches.
     * @param setHeightEvent Specifies the event to notify when operation is
     *        completed.
     */
    void
    SetHeight(
        __in float  height,
        __in Event *setHeightEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("height=%f,event=%p", height, setHeightEvent));

        m_winchPIDMotor->SetTarget(height - PEG_ZERO_HEIGHT,
                                   true,
                                   setHeightEvent);

        TExit();
    } //SetHeight

    /**
     * This function gets the ladder height in inches.
     *
     * @return Returns the ladder height in inches.
     */
    float
    GetHeight(
        void
        )
    {
        float height;

        TLevel(HIFREQ);
        TEnter();

        height = (float)m_winchEncoder->GetDistance();

        TExitMsg(("=%f", height));
        return height;
    } //GetHeight

    /**
     * This function moves the ladder up (or down)
     * 
     * @param speed This is a floating point between 0 (low power) and 1
     *        (high power).
     * @param fOverride If true, the set speed overrides the PID controlled
     *        speed.
     */
    void
    SetSpeed(
        __in float speed,
        __in bool  fOverride
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("speed=%f,fOverride=%d", speed, fOverride));
        
        m_winchPIDMotor->Stop();
        m_winchMotor->Set(speed);
        m_fOverride = fOverride;
        
        TExit();
    }

    /**
     * This function stops the ladder at the bottom
     */
    void
    LadderNotify(
        __in bool fActive
        )
    {
        TLevel(API);
        TEnterMsg(("Ladder stopped, encoder reset %x",fActive));
        
        if(!fActive) 
        {
            Stop(CONTEXT_NONE);
            Start();
        }
        
        TExit();
    }
};  //class Ladder
