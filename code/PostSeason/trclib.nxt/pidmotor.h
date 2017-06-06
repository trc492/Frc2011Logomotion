#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidmotor.h" />
///
/// <summary>
///     This module contains the library functions for the PID motor
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDMOTOR_H
#define _PIDMOTOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDMOTOR

//
// Constants.
//
#define PIDMOTORF_PIDMODE_ON    0x0100
#define PIDMOTORF_STOP_ONTARGET 0x0200
#define PIDMOTORF_USER_MASK     0x00ff
#define PIDMOTORF_ENABLE_EVENTS 0x0001

//
// Type definitions.
//
typedef struct
{
    int motorID;
    int pidCtrl;
    int pidMotorFlags;
    int motorPower;
} PIDMOTOR;

//
// Import function prototypes.
//
void
PIDMotorEvent(
    __in PIDMOTOR &pidMotor
    );

/**
 *  This function resets the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be reset.
 */
void
PIDMotorReset(
    __out PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorReset");
    TLevel(API);
    TEnter();

    pidMotor.pidMotorFlags &= ~PIDMOTORF_PIDMODE_ON;
    motor[pidMotor.motorID] = 0;
    PIDCtrlReset(g_PIDCtrl[pidMotor.pidCtrl]);
    pidMotor.motorPower = 0;

    TExit();
    return;
}   //PIDMotorReset

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be initialized.
 *  @param motorID Specifies the motor ID.
 *  @param pidCtrl Points to the PIDCTRL structure.
 *  @param pidMotorFlags Specifies the flags.
 */
void
PIDMotorInit(
    __out PIDMOTOR &pidMotor,
    __in  int motorID,
    __in  int pidCtrl,
    __in  int pidMotorFlags
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    pidMotor.motorID = motorID;
    pidMotor.pidCtrl = pidCtrl;
    pidMotor.pidMotorFlags = pidMotorFlags & PIDMOTORF_USER_MASK;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint Specifies the set motor target.
 *  @param tolerance Specifies the tolerance when determining OnTarget.
 *  @param fStopOnTarget If true, stop PIDMotor when target is reached.
 *         Otherwise, continue to monitor the target and readjust if necessary.
 */
void
PIDMotorSetTarget(
    __out PIDMOTOR &pidMotor,
    __in  float setPoint,
    __in  float tolerance,
    __in  bool fStopOnTarget
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("setPt=%5.1f", setPoint));

    PIDCtrlSetTarget(g_PIDCtrl[pidMotor.pidCtrl], setPoint, tolerance);
    if (fStopOnTarget)
    {
        pidMotor.pidMotorFlags |= PIDMOTORF_STOP_ONTARGET;
    }
    else
    {
        pidMotor.pidMotorFlags &= ~PIDMOTORF_STOP_ONTARGET;
    }
    pidMotor.pidMotorFlags |= PIDMOTORF_PIDMODE_ON;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function performs the PID motor task.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 */
void
PIDMotorTask(
    __inout PIDMOTOR &pidMotor
    )
{
    int output;

    TFuncName("PIDMotorTask");
    TLevel(TASK);
    TEnter();

    if (pidMotor.pidMotorFlags & PIDMOTORF_PIDMODE_ON)
    {
        pidMotor.motorPower = (int)PIDCtrlOutput(g_PIDCtrl[pidMotor.pidCtrl]);
        if (PIDCtrlIsOnTarget(g_PIDCtrl[pidMotor.pidCtrl]))
        {
            if (pidMotor.pidMotorFlags & PIDMOTORF_STOP_ONTARGET)
            {
                PIDMotorReset(pidMotor);
                if (pidMotor.pidMotorFlags & PIDMOTORF_ENABLE_EVENTS)
                {
                    PIDMotorEvent(pidMotor);
                }
            }
            else
            {
                pidMotor.motorPower = 0;
                motor[pidMotor.motorID] = 0;
            }
        }
        else
        {
            motor[pidMotor.motorID] = pidMotor.motorPower;
        }
    }

    TExit();
    return;
}   //PIDMotorTask

#endif  //ifndef _PIDMOTOR_H
