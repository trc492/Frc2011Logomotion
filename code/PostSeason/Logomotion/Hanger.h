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
class Hanger: public CoopTask,
              public DigitalInNotify
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
        RegisterTask(TASK_STOP);

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

    /**
     * This function stops the Hanger.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_upperMotor->Set(0.0);
        m_lowerMotor->Set(0.0);

        TExit();
        return;
    }   //Stop

    /**
     * This function is called by TaskMgr to stop the Hanger.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    StopTask(
        __in UINT32 context
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("context=%d", context));

        Stop();

        TExit();
        return;
    }   //StopTask

    /**
     * This function is called by the DigitalIn object when the hanger limit
     * switch has changed state.
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

        m_pullTriggered = !fActive;
        if(m_pullTriggered)
        {
            m_upperMotor->Set(0.0);
            m_lowerMotor->Set(0.0);
        }

        TExit();
    }   //NotifyDIn

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
};  //class Hanger
