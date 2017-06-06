#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidctrl.h" />
///
/// <summary>
///     This module contains the library functions for PID Control.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDCTRL_H
#define _PIDCTRL_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDCTRL

//
// Constants.
//
#define PIDCTRLF_USER_MASK      0x00ff
#define PIDCTRLF_INVERSE        0x0001
#define PIDCTRLF_ABS_SETPOINT   0x0002

#define PID_TOLERANCE_NO_STOP   -1.0

#define PIDCtrlGetTarget(p)     (p.setPoint)

//
// Type definitions.
//
typedef struct
{
    int   pidCtrlID;
    float Kp;
    float Ki;
    float Kd;
    float minOutput;
    float maxOutput;
    int   pidCtrlFlags;
    long  time;
    float input;
    float prevError;
    float totalError;
    float setPoint;
    float targetTolerance;
} PIDCTRL;

//
// Import function prototypes.
//
float
PIDCtrlGetInput(
    __in int pidCtrlID
    );

/**
 *  This macro determines if we are on target by comparing the last error to
 *  the given tolerance.
 *
 *  @param p Points to the PID Control structure.
 *  @param t Specifies the error tolerance.
 *
 *  @returns Returns true if the last error is within the given tolerance.
 */
#define PIDCtrlIsOnTarget(p)    (abs(p.prevError) <= p.targetTolerance)

/**
 *  This function resets the PID control.
 *
 *  @param pidCtrl Points to the PID Control structure to be reset.
 */
void
PIDCtrlReset(
    __out PIDCTRL &pidCtrl
    )
{
    TFuncName("PIDCtrlReset");
    TLevel(API);
    TEnter();

    pidCtrl.time = nPgmTime;
    pidCtrl.input = PIDCtrlGetInput(pidCtrl.pidCtrlID);
    pidCtrl.prevError = 0.0;
    pidCtrl.totalError = 0.0;

    TExit();
    return;
}   //PIDCtrlReset

/**
 *  This function initializes the PID control object.
 *
 *  @param pidCtrl Points to the PID Control structure to be initialized.
 *  @param pidCtrlID Specifies the PID controller ID.
 *  @param Kp Specifies the Kp constant.
 *  @param Ki Specifies the Ki constant.
 *  @param Kd Specifies the Kd constant.
 *  @param minOutput Specifies the minimum value of the output range.
 *  @param maxOutput Specifies the maximum value of the output range.
 *  @param pidCtrlFlags Specifies the PID controller flags.
 */
void
PIDCtrlInit(
    __out PIDCTRL &pidCtrl,
    __in  int pidCtrlID,
    __in  float Kp,
    __in  float Ki,
    __in  float Kd,
    __in  float minOutput,
    __in  float maxOutput,
    __in  int pidCtrlFlags
    )
{
    TFuncName("PIDCtrlInit");
    TLevel(INIT);
    TEnter();

    pidCtrl.pidCtrlID = pidCtrlID;
    pidCtrl.Kp = Kp;
    pidCtrl.Ki = Ki;
    pidCtrl.Kd = Kd;
    pidCtrl.minOutput = minOutput;
    pidCtrl.maxOutput = maxOutput;
    pidCtrl.pidCtrlFlags = pidCtrlFlags & PIDCTRLF_USER_MASK;
    pidCtrl.setPoint = 0.0;
    pidCtrl.targetTolerance = 0.0;
    PIDCtrlReset(pidCtrl);

    TExit();
    return;
}   //PIDCtrlInit

/**
 *  This function sets the output power limit.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param minOutput Specifies the minimum output level.
 *  @param maxOutput Specifies the maximum output level.
 */
void
PIDCtrlSetPowerLimit(
    __out PIDCTRL &pidCtrl,
    __in  float minOutput,
    __in  float maxOutput
    )
{
    TFuncName("PIDCtrlSetPwrLimit");
    TLevel(API);
    TEnterMsg(("Min=%5.1f,Max=%5.1f", minOutput, maxOutput));

    pidCtrl.minOutput = minOutput;
    pidCtrl.maxOutput = maxOutput;

    TExit();
    return;
}   //PIDCtrlSetPowerLimit

/**
 *  This function sets the SetPoint.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param setPoint Specifies the SetPoint target.
 *  @param targetTolerance Specifies the the target tolerance of PID control.
 */
void
PIDCtrlSetTarget(
    __inout PIDCTRL &pidCtrl,
    __in    float setPoint,
    __in    float targetTolerance
    )
{
    TFuncName("PIDCtrlSetTarget");
    TLevel(API);
    TEnterMsg(("Tg=%5.1f,To=%5.1f", setPoint, targetTolerance));

    if (!(pidCtrl.pidCtrlFlags & PIDCTRLF_ABS_SETPOINT))
    {
        setPoint += PIDCtrlGetInput(pidCtrl.pidCtrlID);
    }
    pidCtrl.setPoint = setPoint;
    pidCtrl.targetTolerance = targetTolerance;

    TExit();
    return;
}   //PIDCtrlSetTarget

/**
 *  This function calculates the output based on the current input.
 *
 *  @param pidCtrl Points to the PID structure.
 *
 *  @return Returns the calculate output value.
 */
float
PIDCtrlOutput(
    __inout PIDCTRL &pidCtrl
    )
{
    float output;
    long currTime;
    float currInput;
    float error;
    float adjTotalError;

    TFuncName("PIDCtrlOutput");
    TLevel(API);
    TEnter();

    currTime = nPgmTime;
    currInput = PIDCtrlGetInput(pidCtrl.pidCtrlID);
    error = pidCtrl.setPoint - currInput;
    if (pidCtrl.pidCtrlFlags & PIDCTRLF_INVERSE)
    {
        error = -error;
    }
    adjTotalError = pidCtrl.Ki*(pidCtrl.totalError + error);
    if ((adjTotalError >= pidCtrl.minOutput) &&
        (adjTotalError <= pidCtrl.maxOutput))
    {
        pidCtrl.totalError += error;
    }

    output = pidCtrl.Kp*error +
             pidCtrl.Ki*pidCtrl.totalError +
             pidCtrl.Kd*(error - pidCtrl.prevError);

    pidCtrl.time = currTime;
    pidCtrl.input = currInput;
    pidCtrl.prevError = error;
    if (output < pidCtrl.minOutput)
    {
        output = pidCtrl.minOutput;
    }
    else if (output > pidCtrl.maxOutput)
    {
        output = pidCtrl.maxOutput;
    }

    TExitMsg(("=%f", output));
    return output;
}   //PIDCtrlOutput

#endif  //ifndef _PIDCTRL_H
