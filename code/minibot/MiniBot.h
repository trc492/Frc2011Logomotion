#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="MiniBot.h" />
///
/// <summary>
///     This module contains the main MiniBot code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//#define _USE_ACCEL

#include "..\trclib.nxt\batt.h"
#include "..\trclib.nxt\touch.h"
#include "..\trclib.nxt\timer.h"

#ifdef _USE_ACCEL
#include "..\trclib.nxt\accel.h"
#include "..\trclib.nxt\pidctrl.h"
PIDCTRL g_PIDCtrl[1];
#include "..\trclib.nxt\pidmotor.h"
#endif

#include "..\trclib.nxt\sm.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             TASK
#define MSG_LEVEL               INFO

//
// Constants.
//
#define LATCH_DELAY             300

#ifndef _USE_ACCEL
#define MOTOR_POWER_ASCEND      100
#define MOTOR_POWER_DESCEND     -5
#define DECELERATE_PERIOD       200
#define MOTOR_POWER_DECREMENT   (100*LOOP_TIME/DECELERATE_PERIOD)
#define DESCEND_PERIOD          1000
#else
#define PIDMOTOR_KP             180.0
#define PIDMOTOR_KI             0.0
#define PIDMOTOR_KD             0.0
#endif

#define EVTTYPE_TOUCH           (EVTTYPE_NONE + 1)
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 2)

#ifdef _USE_ACCEL
#define EVTTYPE_PIDMOTOR        (EVTTYPE_NONE + 3)
#endif

//
// Global data.
//
BATT    g_Batt;

//
// Input and sensors.
//
TOUCH       g_startButton;
TOUCH       g_topButton;
TIMER       g_timer;

#ifdef _USE_ACCEL
ACCEL       g_accel;
PIDMOTOR    g_pidMotor;
#endif

SM          g_autoSM;
long        g_startTime;
#ifndef _USE_ACCEL
int         g_motorPower;
#endif

/**
 *  This function handles the touch notification events.
 *
 *  @param touch Points to the TOUCH structure that generated the event.
 *  @param fActive Specifies the state of the touch sensor.
 */
void
TouchEvent(
    __in TOUCH &touch,
    __in bool fActive
    )
{
    TFuncName("TouchEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_autoSM))
    {
        SMSetEvent(g_autoSM, EVTTYPE_TOUCH, touch.sensorID, fActive, 0, 0);
    }
    PlayTone(fActive? 800: 400, 15);

    TExit();
    return;
}   //TouchEvent

/**
 *  This function handles the timer notification events.
 *
 *  @param timer Points to the TIMER structure that generated the event.
 */
void
TimerEvent(
    __in TIMER &timer
    )
{
    TFuncName("TimerEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_autoSM))
    {
        SMSetEvent(g_autoSM, EVTTYPE_TIMER, 0, 0, 0, 0);
    }

    TExit();
    return;
}   //TimerEvent

#ifdef _USE_ACCEL
/**
 *  This function handles the PID motor notification events.
 *
 *  @param pidMotor Points to the PIDMOTOR structure that generated the event.
 */
void
PIDMotorEvent(
    __in PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_autoSM))
    {
        SMSetEvent(g_autoSM, EVTTYPE_PIDMOTOR, 0, 0, 0, 0);
    }

    TExit();
    return;
}   //PIDMotorEvent

/**
 *  This function provides the input value for the PID controller.
 *
 *  @param pidCtrlID Specifies the PID Controller ID.
 *
 *  @return Returns the input value for the PID controller.
 */
float
PIDCtrlGetInput(
    __in int pidCtrlID
    )
{
    float inputValue = 0.0;

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", pidCtrlID));

    switch (pidCtrlID)
    {
        case 0:
            //
            // PIDMotor.
            //
            inputValue = (float)(AccelGetYVel(g_accel));
            break;
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput
#endif

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 5, true);

    //
    // Initialize the input subsystems.
    //
    TouchInit(g_startButton, startButton, TOUCHF_ENABLE_EVENTS);
    TouchInit(g_topButton, topButton, TOUCHF_ENABLE_EVENTS);
    TimerInit(g_timer, TIMERF_ENABLE_EVENTS);

#ifdef _USE_ACCEL
    AccelInit(g_accel, accel, 0);
    PIDCtrlInit(g_PIDCtrl[0], 0,
                PIDMOTOR_KP, PIDMOTOR_KI, PIDMOTOR_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                PIDCTRLF_ABS_SETPOINT);
    PIDMotorInit(g_pidMotor, motor1, 0, PIDMOTORF_ENABLE_EVENTS);
#endif

    SMInit(g_autoSM);
#ifndef _USE_ACCEL
    g_motorPower = 0;
#endif
    SMStart(g_autoSM);

    TExit();
    return;
}   //RobotInit

/**
 *  This function processes all the high frequency tasks that needs to run
 *  more often than other tasks such as sensor integration tasks.
 */
void
HiFreqTasks()
{
    TFuncName("HiFreqTasks");
    TLevel(TASK);
    TEnter();
    TExit();
    return;
}   //HiFreqTasks

/**
 *  This function processes all the input tasks.
 */
void
InputTasks()
{
    TFuncName("InputTasks");
    TLevel(TASK);
    TEnter();

    TouchTask(g_startButton);
    TouchTask(g_topButton);
    TimerTask(g_timer);
#ifdef _USE_ACCEL
    AccelTask(g_accel);
#endif

    TExit();
    return;
}   //InputTasks

/**
 *  This function processes all the main tasks.
 */
void
MainTasks()
{
    TFuncName("MainTasks");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(g_autoSM))
    {
        nxtDisplayTextLine(0, "Auto=%d", g_autoSM.currState);
        switch (g_autoSM.currState)
        {
#ifndef _USE_ACCEL
            case SMSTATE_STARTED:
                // Waiting for the start button.
                SMAddWaitEvent(g_autoSM, EVTTYPE_TOUCH, startButton, true);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Wait a little until we latched on securely.
                TimerSet(g_timer, LATCH_DELAY);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Start motors to climb with max velocity and wait for stop button.
                g_startTime = nPgmTime;
                g_motorPower = MOTOR_POWER_ASCEND;
                SMAddWaitEvent(g_autoSM, EVTTYPE_TOUCH, topButton, true);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 3:
                // Reached target, note the ascend time.
                g_motorPower = 0;
                nxtDisplayTextLine(2, "AscendTime=%d", nPgmTime - g_startTime);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TOUCH, startButton, false);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

#if 0
            case SMSTATE_STARTED + 4:
                // Decelerate to stop to make sure it hits target with enough force.
                g_motorPower -= MOTOR_POWER_DECREMENT;
                if (g_motorPower <= 0)
                {
                    g_motorPower = 0;
                    nxtDisplayTextLine(3, "TotalTime=%d", nPgmTime - g_startTime);
                    g_autoSM.currState++;
                }
                break;

            case SMSTATE_STARTED + 5:
                // Debounce delay.
                TimerSet(g_timer, 2000);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 5:
                // Descend for a short period of time.
                g_motorPower = MOTOR_POWER_DESCEND;
                TimerSet(g_timer, DESCEND_PERIOD);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;
#endif

            case SMSTATE_STARTED + 6:
                // Stop the motor and coast down the rest of the way.
                g_motorPower = 0;
                g_autoSM.currState = SMSTATE_STARTED;
                break;

#else
            case SMSTATE_STARTED:
                // Waiting for the start button.
                SMAddWaitEvent(g_autoSM, EVTTYPE_TOUCH, startButton, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Wait a little until we latched on securely.
                TimerSet(g_timer, 200);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Start motors to climb up with max velocity and wait for stop button.
                g_startTime = nPgmTime;
                PIDMotorSetTarget(g_pidMotor, 1000.0, 1.0, false);
                SMAddWaitEvent(g_autoSM, EVTTYPE_TOUCH, startButton, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 3:
                nxtDisplayTextLine(2, "Ascend=%d", nPgmTime - g_startTime);
                // Decelerate to zero velocity.
                PIDMotorSetTarget(g_pidMotor, 0.0, 1.0, true);
                SMAddWaitEvent(g_autoSM, EVTTYPE_PIDMOTOR, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 4:
                // Descend with constant velocity.
                nxtDisplayTextLine(3, "Ascend+decel=%d", nPgmTime - g_startTime);
                PIDMotorSetTarget(g_pidMotor, -10.0, 1.0, false);
                SMAddWaitEvent(g_autoSM, EVTTYPE_PIDMOTOR, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 5:
                // Decelerate to zero velocity.
                PIDMotorSetTarget(g_pidMotor, 0.0, 1.0, true);
                SMAddWaitEvent(g_autoSM, EVTTYPE_PIDMOTOR, -1, -1);
                SMWaitEvents(g_autoSM, g_autoSM.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;
#endif

            default:
                // We are done.
                SMStop(g_autoSM);
                break;
        }
    }

    TExit();
    return;
}   //MainTasks

/**
 *  This function processes all the output tasks. Output tasks are where all
 *  the actions are taking place. All other tasks are just changing states of
 *  various objects. There is no action taken until the output tasks.
 */
void
OutputTasks()
{
    TFuncName("OutputTasks");
    TLevel(TASK);
    TEnter();

#ifdef _USE_ACCEL
    nxtDisplayTextLine(1, "Power=%d", g_pidMotor.motorPower);
    PIDMotorTask(g_pidMotor);
    motor[motor2] = g_pidMotor.motorPower;
#else
    nxtDisplayTextLine(1, "Power=%d", g_motorPower);
    motor[motor1] = g_motorPower;
    motor[motor2] = g_motorPower;
#endif
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
