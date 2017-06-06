#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Lid.h" />
///
/// <summary>
///     This module contains the definitions of the Lid class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_LID
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Lid"

/**
 * This module defines and implements the Lid subsytem. The Lid subsystem
 * consists of two solenoid channels connected to a pneumatic.
 */
class Lid: public CoopTask
{
private:
    typedef enum
    {
        kOff,
        kUp,
        kDown
    } LidState;

    Solenoid       *m_solLidUp;
    Solenoid       *m_solLidDown;
    StateMachine   *m_lidSM;
    Event          *m_event;
    TrcTimer       *m_timer;

    /**
     * This function sets the Lid state.
     *
     * @param state Specifies the Lid state.
     */
    void
    SetLidState(
        __in LidState state 
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
                m_solLidUp->Set(false);
                m_solLidDown->Set(false);
                break;

            case kUp:
                m_solLidDown->Set(false);
                m_solLidUp->Set(true);
                break;

            case kDown:
                m_solLidUp->Set(false);
                m_solLidDown->Set(true);
                break;
        }

        TExit();
    }   //SetLidState

public:
    /**
     * Constructor for the Lid class.
     */
    Lid(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_solLidUp = new Solenoid(SOL_LID_UP);
        m_solLidDown = new Solenoid(SOL_LID_DOWN);
        m_lidSM = new StateMachine();
        m_event = new Event();
        m_timer = new TrcTimer();
        RegisterTask(TASK_STOP | TASK_PROCESS_ACTION);

        TExit();
    }   //Lid

    /**
     * Destructor for the Lid class.
     */
    virtual
    ~Lid(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SAFE_DELETE(m_timer);
        SAFE_DELETE(m_event);
        SAFE_DELETE(m_lidSM);
        SAFE_DELETE(m_solLidDown);
        SAFE_DELETE(m_solLidUp);

        TExit();
    }   //~Lid

    /**
     * This function is called by the Disabled code to stop the Lid.
     *
     * @param context Specifies the caller context.
     */
    void
    StopTask(
        __in UINT32 context
        )
    {
        TLevel(INIT);
        TEnterMsg(("context=%d", context));

        m_timer->CancelTimer();
        m_event->ClearEvent();
        m_lidSM->Stop();
        SetLidState(kOff);

        TExit();
    }   //StopTask

    /**
     * This function set the Lid up or down.
     */
    void
    SetLidUp(
        __in bool fUp
        )
    {
        TLevel(API);
        TEnter();

        if (fUp)
        {
            //
            // Set the pneumatics to extend and hold.
            //
            SetLidState(kUp);
        }
        else
        {
            //
            // Set the state machine to retract, hold briefly and then off.
            //
            m_lidSM->Start();
        }

        TExit();
    }   //SetLidUp

    /**
     * This function is called periodically to execute the Lid state machine.
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

        if (m_lidSM->IsReady())
        {
            UINT32 stateCurr = m_lidSM->GetCurrentState();

            switch (stateCurr)
            {
                case SMSTATE_STARTED:
                    SetLidState(kDown);
                    m_timer->SetTimer(LID_HOLD_PERIOD, m_event);
                    m_lidSM->WaitForEvents(&m_event, 1, stateCurr + 1, 0);
                    break;

                case SMSTATE_STARTED + 1:
                    SetLidState(kOff);
                    m_lidSM->Stop();
                    break;
            }
        }

        TExit();
    }   //ProcessAction
};  //class Lid
