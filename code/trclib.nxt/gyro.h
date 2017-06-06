#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="gyro.h" />
///
/// <summary>
///     This module contains the library functions for the gyro sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _GYRO_H
#define _GYRO_H

#include "..\HTDriversV1.6\drivers\HTGYRO-driver.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_GYRO

//
// Constants.
//
#define GYROF_USER_MASK         0x00ff
#define GYROF_INVERSE           0x0001
#ifdef HTSMUX_STATUS
  #define GYROF_HTSMUX          0x0080
#endif

#define GYRO_NUM_CAL_SAMPLES    50
#define GYRO_CAL_INTERVAL       10

//
// Macros
//
#define GyroGetTurnRate(p)      (p.turnRate)
#define GyroGetHeading(p)       (p.heading)

//
// Type definitions.
//
typedef struct
{
    int   sensorID;
    int   gyroFlags;
    int   zeroOffset;
    int   deadBand;
    long  timestamp;
    int   turnRate;
    float heading;
} GYRO;

/**
 *  This function calibrates the gyro for zero offset and deadband.
 *
 *  @param gyro Points to the GYRO structure to be initialized.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
GyroCal(
    __out GYRO &gyro,
    __in  int numSamples,
    __in  int calInterval
    )
{
    int i;
    int turnRate;
    int min, max;

    TFuncName("GyroCal");
    TLevel(API);

    gyro.zeroOffset = 0;
    gyro.deadBand = 0;
    min = 1023;
    max = 0;

    for (i = 0; i < numSamples; i++)
    {
#ifdef HTSMUX_STATUS
        turnRate = (gyro.gyroFlags & GYROF_HTSMUX)?
                        HTGYROreadRot((tMUXSensor)gyro.sensorID):
                        HTGYROreadRot((tSensors)gyro.sensorID);
#else
        turnRate = HTGYROreadRot((tSensors)gyro.sensorID);
#endif
        gyro.zeroOffset += turnRate;

        if (turnRate < min)
        {
            min = turnRate;
        }
        else if (turnRate > max)
        {
            max = turnRate;
        }

        wait1Msec(calInterval);
    }

    gyro.zeroOffset /= numSamples;
    gyro.deadBand = max - min;

    TExit();
    return;
}   //GyroCal

/**
 *  This function performs the gyro task where it integrates the turn rate
 *  into a heading value.
 *
 *  @param gyro Points to the GYRO structure.
 */
void
GyroTask(
    __inout GYRO &gyro
    )
{
    long currTime;

    TFuncName("GyroTask");
    TLevel(TASK);
    TEnter();

    currTime = nPgmTime;
#ifdef HTSMUX_STATUS
    gyro.turnRate = (gyro.gyroFlags & GYROF_HTSMUX)?
                        HTGYROreadRot((tMUXSensor)gyro.sensorID):
                        HTGYROreadRot((tSensors)gyro.sensorID);
#else
    gyro.turnRate = HTGYROreadRot((tSensors)gyro.sensorID);
#endif
    gyro.turnRate -= gyro.zeroOffset;
    gyro.turnRate = DEADBAND(gyro.turnRate, gyro.deadBand);
    if (gyro.gyroFlags & GYROF_INVERSE)
    {
        gyro.turnRate = -gyro.turnRate;
    }
    gyro.heading += (float)gyro.turnRate*(currTime - gyro.timestamp)/1000;
    gyro.timestamp = currTime;

    TExit();
    return;
}   //GyroTask

/**
 *  This function resets the gyro heading.
 *
 *  @param gyro Points to the GYRO structure to be reset.
 */
void
GyroReset(
    __out GYRO &gyro
    )
{
    TFuncName("GyroReset");
    TLevel(API);
    TEnter();

    GyroTask(gyro);
    gyro.heading = 0;

    TExit();
    return;
}   //GyroReset

/**
 *  This function initializes the gyro sensor.
 *
 *  @param gyro Points to the GYRO structure to be initialized.
 *  @param sensorID Specifies the ID of the gyro sensor.
 *  @param gyroFlags Specifies the gyro flags.
 */
void
GyroInit(
    __out GYRO &gyro,
    __in  int sensorID,
    __in  int gyroFlags
    )
{
    TFuncName("GyroInit");
    TLevel(INIT);
    TEnter();

    gyro.sensorID = sensorID;
    gyro.gyroFlags = gyroFlags & GYROF_USER_MASK;
#ifdef HTSMUX_STATUS
    if (gyro.gyroFlags & GYROF_HTSMUX)
    {
        HTGYROstartCal((tMUXSensor)sensorID);
    }
    else
    {
        HTGYROstartCal((tSensors)sensorID);
    }
#else
    HTGYROstartCal((tSensors)sensorID);
#endif
    GyroCal(gyro, GYRO_NUM_CAL_SAMPLES, GYRO_CAL_INTERVAL);
    gyro.timestamp = nPgmTime;
    GyroReset(gyro);

    TExit();
    return;
}   //GyroInit

#endif  //ifndef _GYRO_H
