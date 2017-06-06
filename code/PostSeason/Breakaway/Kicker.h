#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Kicker.h" />
///
/// <summary>
///     This module contains the definitions of the Kicker class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_KICKER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Kicker"

/**
 * This module defines and implements the Kicker subsytem. The Kicker
 * subsystem consists of four solenoid channels connected to two pneumatics.
 */
class Kicker: public CoopTask,
              public DigitalInNotify
{
private:
    typedef enum
    {
        kOff,
        kExtend,
        kRetract
    } KickerState;

    Solenoid       *m_solKickerLeftOut;
    Solenoid       *m_solKickerLeftIn;
    Solenoid       *m_solKickerRightOut;
    Solenoid       *m_solKickerRightIn;
    StateMachine   *m_kickerSM;
    TrcTimer       *m_timer;
    Event          *m_event;
    bool            m_fKickerArmed;

    /**
     * This function sets the kicker state.
     *
     * @param state Specifies the kicker state.
     */
    void
    SetKickerState(
        __in KickerState state 
        )
    {
        TLevel(FUNC);
        TEnterMsg(("state=%d", state));

        //
        // It is important to deactivate one valve before activating
        // another. You must not have two activated valve at the same
        // time. In other words, set a valve to false before setting
        // another to true.
        //
        switch (state)
        {
            case kOff:
                m_solKickerLeftIn->Set(false);
                m_solKickerRightIn->Set(false);
                m_solKickerLeftOut->Set(false);
                m_solKickerRightOut->Set(false);               
                break;

            case kExtend:
                m_solKickerLeftIn->Set(false);
                m_solKickerRightIn->Set(false);
                m_solKickerLeftOut->Set(true);
                m_solKickerRightOut->Set(true);               
                break;

            case kRetract:
                m_solKickerLeftOut->Set(false);
                m_solKickerRightOut->Set(false);               
                m_solKickerLeftIn->Set(true);
                m_solKickerRightIn->Set(true);
                break;
        }

        TExit();
    }   //SetKickerState

public:
    /**
     * Constructor for the Kicker class.
     */
    Kicker(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_solKickerLeftOut = new Solenoid(SOL_KICKER_LEFT_OUT);
        m_solKickerLeftIn = new Solenoid(SOL_KICKER_LEFT_IN);
        m_solKickerRightOut = new Solenoid(SOL_KICKER_RIGHT_OUT);
        m_solKickerRightIn = new Solenoid(SOL_KICKER_RIGHT_IN);
        m_kickerSM = new StateMachine();
        m_timer = new TrcTimer();
        m_event = new Event();
        m_fKickerArmed = false;
        RegisterTask(TASK_INIT | TASK_STOP | TASK_PROCESS_ACTION);

        TExit();
    }   //Kicker

    /**
     * Destructor for the Kicker class.
     */
    virtual
    ~Kicker(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SAFE_DELETE(m_event);
        SAFE_DELETE(m_timer);
        SAFE_DELETE(m_kickerSM);
        SAFE_DELETE(m_solKickerRightIn);
        SAFE_DELETE(m_solKickerRightOut);
        SAFE_DELETE(m_solKickerLeftIn);
        SAFE_DELETE(m_solKickerLeftOut);

        TExit();
    }   //~Kicker

    /**
     * This function is called to initialize the Kicker subsystem.
     */
    void
    InitTask(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Perform a kick to make sure it is retracted.
        //
        m_kickerSM->Start();

        TExit();
    }   //KickerInit

    /**
     * This function is called by the Disabled code to stop the kicker.
     *
     * @param context Specifies the caller context.
     */
    void
    StopTask(
        __in UINT32 context
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("context=%d", context));

        m_timer->CancelTimer();
        m_event->ClearEvent();
        m_kickerSM->Stop();
        SetKickerState(kOff);

        TExit();
    }   //StopTask

    /**
     * This function activates the kicker to kick by starting the kicker
     * state machine.
     */
    void
    Kick(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_kickerSM->Start();

        TExit();
    }   //Kick

    /**
     * This function arms the kicker so it will automatically kick when
     * the left or right feelers are activated.
     *
     * @param fArmed If true, kicker is armed.
     */
    void
    SetArmedState(
       bool fArmed
       )
    {
        TLevel(API);
        TEnterMsg(("fArmed=%d", fArmed));

        m_fKickerArmed = fArmed;

        TExit();
    }   //SetArmedState

    /**
     * This function is called by the DigitalIn object when the feeler limit
     * switches have changed state.
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
        static UINT32 lastKickTime = 0;
        UINT32 currTime = GetMsecTime();

        TLevel(EVENT);
        TEnterMsg(("slot=%d,channel=%d,fActive=%d", slot, channel, fActive));

        //
        // The timer is to debounce the feeler switches for 1 second.
        //
        if (m_fKickerArmed && !fActive && (currTime - lastKickTime > 1000))
        {
            Kick();
            lastKickTime = currTime;
        }

        TExit();
    }   //NotifyDIn

    /**
     * This function is called periodically to execute the kicker state
     * machine.
     *
     * @param context Specifies the caller context.
     */
    void
    ProcessAction(
        __in UINT32 context
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("context=%d", context));

        if (m_kickerSM->IsReady())
        {
            UINT32 stateCurr = m_kickerSM->GetCurrentState();

            switch (stateCurr)
            {
                case SMSTATE_STARTED:
                    SetKickerState(kExtend);
                    m_timer->SetTimer(KICKER_EXTEND_PERIOD, m_event);
                    m_kickerSM->WaitForEvents(&m_event, 1, stateCurr + 1, 0);
                    break;

                case SMSTATE_STARTED + 1:
                    SetKickerState(kRetract);
                    m_timer->SetTimer(KICKER_RETRACT_PERIOD, m_event);
                    m_kickerSM->WaitForEvents(&m_event, 1, stateCurr + 1, 0);
                    break;

                case SMSTATE_STARTED + 2:
                    SetKickerState(kOff);
                    m_kickerSM->Stop();
                    break;
            }
        }

        TExit();
    }   //ProcessAction
};  //class Kicker
