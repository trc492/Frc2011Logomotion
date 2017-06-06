#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Hanger.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Hanger class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_HANGER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Hanger"

/**
 * This module defines and implements the Hanger subsytem. The Hanger
 * subsystem consists of two motors driving an upper and lower converyer
 * belts. By controlling the direction of the conveyer belts, one can
 * suck in, spit out, flip up or flip down a game piece.
 */
class Hanger: public SubSystem
{
private:
    Victor  *m_upperMotor;
    Victor  *m_lowerMotor;
    bool     m_pullTriggered;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Hanger(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_upperMotor = new Victor(PWM_HANGERTOP_MOTOR);
        m_lowerMotor = new Victor(PWM_HANGERBOTTOM_MOTOR);
        m_pullTriggered = false;
        RegisterSubSystem(SUBSYS_STOP);

        TExit();
    }   //Hanger

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Hanger(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SAFE_DELETE(m_lowerMotor);
        SAFE_DELETE(m_upperMotor);

        TExit();
    }   //~Hanger

#if 0
    /**
     * This function is called one time to do subsystem initialization.
     */
    void
    Init(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //Init
#endif

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

        m_upperMotor->Set(0.0);
        m_lowerMotor->Set(0.0);

        TExit();
    }   //Stop

#if 0
    /**
     * This function is called periodically to perform the subsystem tasks.
     */
    void
    Task(
        void
        )
    {
        TLevel(TASK);
        TEnter();
        TExit();
    }   //Task
#endif

    /**
     * This function sets the hanger to push the game piece out.
     */
    void
    SetPush(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_upperMotor->Set(HANGER_TOPOUT_SPEED);
        m_lowerMotor->Set(HANGER_BOTTOMOUT_SPEED);

        TExit();
    }   //SetPush

    /**
     * This function sets the hanger to pull the game piece in.
     */
    void
    SetPull(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_pullTriggered)
        {
            m_upperMotor->Set(0.0);
            m_lowerMotor->Set(0.0);
        }
        else
        {
            m_upperMotor->Set(HANGER_TOPIN_SPEED);
            m_lowerMotor->Set(HANGER_BOTTOMIN_SPEED);
        }

        TExit();
    }   //SetPull

    /**
     * This function sets the hanger to flip the game piece up.
     */
    void
    FlipUp(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_upperMotor->Set(HANGER_TOPIN_SPEED);
        m_lowerMotor->Set(HANGER_BOTTOMOUT_SPEED);

        TExit();
    }   //FlipUp

    /**
     * This function sets the hanger to flip the game piece down.
     */
    void
    FlipDown(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_upperMotor->Set(HANGER_TOPOUT_SPEED);
        m_lowerMotor->Set(HANGER_BOTTOMIN_SPEED);

        TExit();
    }   //FlipDown
    /**
     * This function gets called when the hanger switched is triggered and it stops the motors
     */
    void
    HangerNotify(
       __in bool fActive
       )
    {
        TLevel(API);
        TEnter();

        m_pullTriggered = !fActive;
        if(m_pullTriggered)
        {
            m_upperMotor->Set(0.0);
            m_lowerMotor->Set(0.0);
        }
        
        TExit();
    }
};  //class Hanger
