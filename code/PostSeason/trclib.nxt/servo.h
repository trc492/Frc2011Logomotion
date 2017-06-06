#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="servo.h" />
///
/// <summary>
///     This module contains the library functions for the servo subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SERVO_H
#define _SERVO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SERVO

//
// Constants.
//
#define MIN_SERVO_VALUE         0
#define MAX_SERVO_VALUE         255
#if 0
#define SERVOF_USER_MASK        0x00ff
#define SERVOF_ENABLE_EVENTS    0x0001
#endif

//
// Type definitions.
//
typedef struct
{
    int   servoMotor;
    float posPerDegree;
#if 0
    int   servoFlags;
    int   servoPos;
#endif
} SERVO;

#if 0
//
// Import function prototypes.
//
void
ServoEvent(
    __in SERVO &serv
    );
#endif

/**
 *  This function initializes the servo system.
 *
 *  @param serv Points to the SERVO structure to be initialized.
 *  @param servoMotor Specifies the servo motor ID.
 *  @param posPerDegree Specifies the position value per degree.
 *  @param initDegrees Specifies the initial position in degrees.
 *  @param servoFlags Specifies the servo flags.
 */
void
ServoInit(
    __out SERVO &serv,
    __in  int servoMotor,
    __in  float posPerDegree,
    __in  float initDegrees
#if 0
    __in  int servoFlags
#endif
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servoMotor = servoMotor;
    serv.posPerDegree = posPerDegree;
#if 0
    serv.servoFlags = servoFlags & SERVOF_USER_MASK;
    serv.servoPos = 0;
#endif
    servo[servoMotor] = initDegrees*posPerDegree;

    TExit();
    return;
}   //ServoInit

/**
 *  This function gets the current servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *
 *  @return Returns the current servo position in degrees.
 */
float
ServoGetAngle(
    __out SERVO &serv
    )
{
    float degree;

    TFuncName("ServoGetPos");
    TLevel(API);
    TEnter();

    degree = (float)ServoValue[serv.servoMotor]/serv.posPerDegree;

    TExitMsg(("=%5.1f", degree));
    return degree;
}   //ServoGetAngle

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param degree Specifies the servo position in degrees.
 */
void
ServoSetAngle(
    __out SERVO &serv,
    __in  float degree
    )
{
    int servoPos;

    TFuncName("ServoSetPos");
    TLevel(API);
    TEnterMsg(("degree=%5.1f", degree));

    servoPos = (int)(degree*serv.posPerDegree);
    if (servoPos > MAX_SERVO_VALUE)
    {
        servoPos = MAX_SERVO_VALUE;
    }
    else if (servoPos < MIN_SERVO_VALUE)
    {
        servoPos = MIN_SERVO_VALUE;
    }
    servo[serv.servoMotor] = servoPos;

    TExit();
    return;
}   //ServoSetAngle

#if 0
/**
 *  This function performs the servo task.
 *
 *  @param drive Points to the SERVO structure.
 */
void
ServoTask(
    __inout SERVO &serv
    )
{
    int output;

    TFuncName("ServoTask");
    TLevel(TASK);
    TEnter();

    servo[serv.servoMotor] = serv.servoPos;
    if ((serv.servoFlags & SERVOF_ENABLE_EVENTS) &&
        (ServoValue[serv.servoMotor] == serv.servoPos))
    {
        ServoEvent(serv);
    }

    TExit();
    return;
}   //ServoTask
#endif

#endif  //ifndef _SERVO_H
