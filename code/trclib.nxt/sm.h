#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="sm.h" />
///
/// <summary>
///     This module contains the library functions to handle state machines.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SM_H
#define _SM_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SM

//
// Constants.
//
#define SMF_READY               0x0100
#define SMF_TIMEOUT             0x0200
#define SMF_MASK                0x00ff
#define SMF_WAIT_ALL            0x0001
#define SMF_CLEAR_EVENTS        0x0002

#define SMSTATE_DISABLED        0
#define SMSTATE_STARTED         1

#define EVTTYPE_NONE            0

#ifndef MAX_WAIT_EVENTS
#define MAX_WAIT_EVENTS         4
#endif

//
// Macros.
//
#define IsSMDisabled(s)         (s.currState == SMSTATE_DISABLED)
#define IsSMEnabled(s)          (s.currState != SMSTATE_DISABLED)
#define IsSMTimedOut(s)         (s.smFlags & SMF_TIMEOUT)

//
// Type definitions.
//
typedef struct
{
    int  evtType;
    int  evtID;
    int  evtData;
    int  evtParam1;
    int  evtParam2;
    bool fSignaled;
} WAIT_EVT;

typedef struct
{
    int  currState;
    int  nextState;
    long timeout;
    int  smFlags;
    bool fClearEvts;
    int  nWaitEvents;
    WAIT_EVT WaitEvents[MAX_WAIT_EVENTS];
} SM;

/**
 *  This function clears all wait events in the state machine.
 *
 *  @param sm Points to the SM structure.
 */
void
SMClearAllEvents(
    __out SM &sm
    )
{
    TFuncName("SMClearAllEvents");
    TLevel(API);
    TEnter();

    for (int i = 0; i < MAX_WAIT_EVENTS; ++i)
    {
        sm.WaitEvents[i].evtType = EVTTYPE_NONE;
        sm.WaitEvents[i].evtID = 0;
        sm.WaitEvents[i].evtData = 0;
        sm.WaitEvents[i].evtParam1 = 0;
        sm.WaitEvents[i].evtParam2 = 0;
        sm.WaitEvents[i].fSignaled = false;
    }
    sm.nWaitEvents = 0;
    sm.timeout = 0;
    sm.smFlags &= ~SMF_TIMEOUT;

    TExit();
    return;
}   //SMClearAllEvents

/**
 *  This function initializes the state machine.
 *
 *  @param sm Points to the SM structure to be initialized.
 */
void
SMInit(
    __out SM &sm
    )
{
    TFuncName("SMInit");
    TLevel(INIT);
    TEnter();

    sm.currState = SMSTATE_DISABLED;
    sm.nextState = SMSTATE_DISABLED;
    sm.smFlags = 0;
    sm.fClearEvts = false;
    SMClearAllEvents(sm);

    TExit();
    return;
}   //SMInit

/**
 *  This function starts the state machine.
 *
 *  @param sm Points to the SM structure to be initialized.
 */
void
SMStart(
    __out SM &sm
    )
{
    TFuncName("SMStart");
    TLevel(API);
    TEnter();

    if (sm.currState == SMSTATE_DISABLED)
    {
        sm.currState = SMSTATE_STARTED;
        sm.nextState = SMSTATE_STARTED;
        sm.smFlags = SMF_READY;
        SMClearAllEvents(sm);
    }

    TExit();
    return;
}   //SMStart

/**
 *  This function stops the state machine.
 *
 *  @param sm Points to the SM structure to be initialized.
 */
void
SMStop(
    __out SM &sm
    )
{
    TFuncName("SMStop");
    TLevel(API);
    TEnter();

    SMInit(sm);

    TExit();
    return;
}   //SMStop

/**
 *  This function checks if the state machine is ready or timed out.
 *
 *  @param sm Points to the SM structure to be initialized.
 */
bool
SMIsReady(
    __out SM &sm
    )
{
    bool fReady;

    TFuncName("SMIsReady");
    TLevel(API);
    TEnter();

    if (((sm.smFlags & SMF_READY) == 0) &&
        (sm.timeout > 0) &&
        ((long)nPgmTime >= sm.timeout))
    {
        sm.smFlags |= (SMF_TIMEOUT | SMF_READY);
        if (sm.fClearEvts)
        {
            sm.fClearEvts = false;
            SMClearAllEvents(sm);
        }
        sm.currState = sm.nextState;
    }
    fReady = (sm.smFlags & SMF_READY) != 0;

    TExitMsg(("%=d", (byte)fReady));
    return fReady;
}   //SMIsReady

/**
 *  This function adds a wait event to the state machine.
 *
 *  @param sm Points to the SM structure.
 *  @param evtType Specifies the event type to wait for.
 *  @param evtID Specifies the event ID to wait for.
 *  @param evtData Specifies the event data to wait for.
 *
 *  @return Success: Returns true.
 *  @return Failure: Returns false.
 */
bool
SMAddWaitEvent(
    __inout SM &sm,
    __in    int evtType,
    __in    int evtID,
    __in    int evtData
    )
{
    TFuncName("SMAddWaitEvent");
    TLevel(API);
    TEnterMsg(("Type=%x,ID=%x", evtType, evtID));

    bool fAdded = false;

    if (sm.nWaitEvents < MAX_WAIT_EVENTS)
    {
        sm.WaitEvents[sm.nWaitEvents].evtType = evtType;
        sm.WaitEvents[sm.nWaitEvents].evtID = evtID;
        sm.WaitEvents[sm.nWaitEvents].evtData = evtData;
        sm.WaitEvents[sm.nWaitEvents].evtParam1 = 0;
        sm.WaitEvents[sm.nWaitEvents].evtParam2 = 0;
        sm.WaitEvents[sm.nWaitEvents].fSignaled = false;
        sm.nWaitEvents++;
        fAdded = true;
    }
    TInfo(("nEvts=%d", sm.nWaitEvents));

    TExitMsg(("fOK=%d", (byte)fAdded));
    return fAdded;
}   //SMAddWaitEvent

/**
 *  This function sets the wait event mode and the next state to advance to
 *  when the wait is fulfilled.
 *
 *  @param sm Points to the SM structure.
 *  @param nextState Specifies the next state to advance to.
 *  @param timeout Specifies the timeout value in msec for the wait.
 *         If 0, it will wait indefinitely for the events.
 *  @param flags Specifies the SMF flags.
 */
void
SMWaitEvents(
    __inout SM &sm,
    __in    int nextState,
    __in    int timeout,
    __in    int flags
    )
{
    TFuncName("SMWaitEvents");
    TLevel(API);
    TEnterMsg(("Next=%d,timout=%d", nextState, timeout));

    if (sm.nWaitEvents > 0)
    {
        sm.nextState = nextState;
        sm.timeout = (timeout > 0)? nPgmTime + timeout: 0;
        sm.smFlags |= flags & SMF_MASK;
        sm.smFlags &= ~(SMF_READY | SMF_TIMEOUT);
        if (sm.smFlags & SMF_CLEAR_EVENTS)
        {
            sm.fClearEvts = true;
            sm.smFlags &= ~SMF_CLEAR_EVENTS;
        }
        else
        {
            sm.fClearEvts = false;
        }
    }

    TExit();
    return;
}   //SMWaitEvents

/**
 *  This function is called when an event has occurred. It will determine if
 *  the state machine is waiting for the event and will advance the state
 *  machine to the next state if necessary.
 *
 *  @param sm Points to the SM structure.
 *  @param evtType Specifies the event type.
 *  @param evtID Specifies the event ID.
 *  @param evtData Specifies the event data.
 *  @param evtParam1 Specifies the event parameter 1.
 *  @param evtParam2 Specifies the event parameter 2.
 */
void
SMSetEvent(
    __inout SM &sm,
    __in    int evtType,
    __in    int evtID,
    __in    int evtData,
    __in    int evtParam1,
    __in    int evtParam2
    )
{
    TFuncName("SMSetEvent");
    TLevel(API);
    TEnterMsg(("Type=%x,ID=%x", evtType, evtID));

    for (int i = 0; i < sm.nWaitEvents; ++i)
    {
        if (!sm.WaitEvents[i].fSignaled &&
            (sm.WaitEvents[i].evtType == evtType) &&
            ((sm.WaitEvents[i].evtID == -1) ||
             (sm.WaitEvents[i].evtID == evtID)) &&
            ((sm.WaitEvents[i].evtData == -1) ||
             (sm.WaitEvents[i].evtData == evtData)))
        {
            //
            // If the all event attributes matched or we don't care some of
            // them, we mark the event signaled.
            //
            TInfo(("Type=%x,ID=%x",
                   sm.WaitEvents[i].evtType, sm.WaitEvents[i].evtID));
            sm.WaitEvents[i].fSignaled = true;
            sm.WaitEvents[i].evtID = evtID;
            sm.WaitEvents[i].evtData = evtData;
            sm.WaitEvents[i].evtParam1 = evtParam1;
            sm.WaitEvents[i].evtParam2 = evtParam2;

            bool fAdvanceState = true;
            if (sm.smFlags & SMF_WAIT_ALL)
            {
                for (int j = 0; j < sm.nWaitEvents; ++j)
                {
                    if (!sm.WaitEvents[j].fSignaled)
                    {
                        fAdvanceState = false;
                        break;
                    }
                }
            }

            if (fAdvanceState)
            {
                TInfo(("AdvanceState"));
                sm.smFlags |= SMF_READY;
                if (sm.fClearEvts)
                {
                    sm.fClearEvts = false;
                    SMClearAllEvents(sm);
                }
                sm.currState = sm.nextState;
            }
            break;
        }
    }

    TExit();
  return;
}   //SMSetEvent

#endif  //ifndef _SM_H
