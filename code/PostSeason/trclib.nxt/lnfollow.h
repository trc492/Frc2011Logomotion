#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="lnfollow.h" />
///
/// <summary>
///     This module contains the library functions to follow the line.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LNFOLLOW_H
#define _LNFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif

#define MOD_ID                  MOD_LNFOLLOW

//
// Constants.
//
#define LNF_STARTED             0x0001
#define LnFollowStarted(l)      (l.lnfollowFlags & LNF_STARTED)

//
// Type definitions.
//
typedef struct {
    int     numSensors;
    SESNOR  Sensors[MAX_NUM_SENSORS];
} SENSORARRAY;

int
SensorArrayGetRawValue(
    __in SENSORARRAY &sensorArray
    );

int
SensorArrayGetMappedValue(
    __in SENSORARRAY &sensorArray
    );
    
typedef struct
{
//    DRIVE   &drive;
//    PIDCTRL &pidCtrl;
    int      sensorArray;
    int      drive;
    int      pidCtrl;
    int      maxDrivePower;
    int      stopSensorValue;
    int      findLineDrivePower;
    int      findLineTurnPower;
} LNFOLLOW;

/**
 *  This function initializes the line follower object.
 *
 *  @param lnfollow Points to the LNFOLLOW structure to be initialized.
 *  @param drive Points to the DRIVE structure.
 *  @param pidCtrl Points to the PIDCTRL structure.
 *  @param numLightSensors Specifies the number of light sensors used.
 *  @param maxDrivePower Specifies the maximum drive power.
 */
void
LnFollowInit(
    __out LNFOLLOW &lnfollow,
//    __in  DRIVE &drive,
//    __in  PIDCTRL &pidCtrl,
    __in  int sensorArray,
    __in  int drive,
    __in  int pidCtrl,
    __in  int maxDrivePower
    )
{
    int i;

    TFuncName("LnFollowInit");
    TLevel(INIT);
    TEnter();

    lnfollow.sensorArray = sensorArray;
    lnfollow.drive = drive;
    lnfollow.pidCtrl = pidCtrl;
    lnfollow.maxDrivePower = maxDrivePower;
    lnfollow.lnfollowFlags = 0;
    lnfollow.findLineDrivePower = 0;
    lnfollow.findLineTurnPower = 0;

    TExit();
    return;
}   //LnFollowInit

#if 0
/**
 *  This function calibrates the light sensors of the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param fStart Specifies TRUE to start calibration, FALSE to stop.
 */
void
LnFollowCal(
    __inout LNFOLLOW &lnfollow,
    __in    bool fStart
    )
{
    TFuncName("LnFollowCal");
    TLevel(API);
    TEnterMsg(("fStart=%d", (byte)fStart));

    if (fStart)
    {
        lnfollow.lnfollowFlags |= LNF_CALIBRATING;
    }
    else
    {
        lnfollow.lnfollowFlags &= ~LNF_CALIBRATING;
    }

    for (int i = 0; i < lnfollow.numLightSensors; i++)
    {
        SensorCal(lnfollow.LightSensors[i], fStart);
    }

    TExit();
    return;
}   //LnFollowCal
#endif

/**
 *  This function starts the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param centerValue Specifies the line center sensor value.
 *  @param tolerance Specifies the PID control tolerance.
 *  @param stopSensorValue Specifies the stop sensor value.
 */
void
LnFollowStart(
    __inout LNFOLLOW &lnfollow,
    __in    float centerValue,
    __in    int stopSensorValue
    __in    int maxDrivePower,
    __in    int findLineDrivePower,
    __in    int findLineTurnPower,
    __in    int timeout
    )
{
    TFuncName("LnFollowStart");
    TLevel(API);
    TEnterMsg(("target=%5.1f,tol=%5.1f", centerValue, tolerance));

    lnfollow.stopSensorValue = stopSensorValue;
    PIDCtrlSetTarget(g_PIDCtrl[lnfollow.pidCtrl], centerValue, tolerance);
    lnfollow.lnfollowFlags |= LNF_STARTED;

    TExit();
    return;
}   //LnFollowStart

/**
 *  This function stops the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowStop(
    __inout LNFOLLOW &lnfollow,
    )
{
    TFuncName("LnFollowStop");
    TLevel(API);

    DriveReset(g_Drive[lnfollow.drive]);
    PIDCtrlReset(g_PIDCtrl[lnfollow.pidCtrl]);
    lnfollow.stopSensorValue = INVALID_SENSOR_VALUE;
    lnfollow.lnfollowFlags &= ~LNF_STARTED;

    TExit();
    return;
}   //LnFollowStop

/**
 *  This function sets the find line drive and turn powers.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param drivePower Specifies the drive power for finding the line.
 *  @param turnPower Specifies the turn power for finding the line.
 */
void
LnFollowSetFindLinePower(
    __inout LNFOLLOW &lnfollow,
    __in int drivePower,
    __in int turnPower
    )
{
    TFuncName("SetFineLine");
    TLevel(TASK);
    TEnter();

    lnfollow.findLineDrivePower = drivePower;
    lnfollow.findLineTurnPower = turnPower;

    TExit();
    return;
}   //LnFollowSetFindLinePower

/**
 *  This function processes the sensor data for line following.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowTask(
    __inout LNFOLLOW &lnfollow
    )
{
    TFuncName("LnFollowTask");
    TLevel(TASK);
    TEnter();

    if (LnFollowStarted(lnfollow))
    {
        int sensorRawValue = SensorArrayGetRawValue(lnfollow.sensorArray);

        if ((lnfollow.stopSensorValue != INVALID_SENSOR_VALUE) &&
            (sensorRawValue == lnfollow.stopSensorValue))
        {
            LnFollowStop(lnfollow);
            //
            // notify caller if necessary.
            //
        }
        else if (sensorRawValue != 0)
        {
            float mappedSensorValue = SensorArrayGetMappedValue(
                                        lnfollow.sensorArray);
            int turnPower, drivePower;

            turnPower = BOUND((int)PIDCtrlOutput(g_PIDCtrl[lnfollow.pidCtrl],
                                                 mappedSensorValue),
                              MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
            drivePower = lnfollow.maxDrivePower*
                         (MOTOR_MAX_VALUE - abs(turnPower))/MOTOR_MAX_VALUE;
            DriveArcade(g_Drive[lnfollow.drive], drivePower, turnPower);
        }
        else
        {
            //
            // We lost the line. Find it again.
            //
            DriveArcade(g_Drive[lnfollow.drive],
                        lnfollow.findLineDrivePower,
                        lnfollow.findLineTurnPower);
        }
    }

    TExit();
    return;
}   //LnFollowTask

#endif  //ifndef _LNFOLLOW_H
