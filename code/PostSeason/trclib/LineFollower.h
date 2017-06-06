#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="LineFollower.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     LineFollower class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _LINEFOLLOWER_H
#define _LINEFOLLOWER_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_LNFOLLOWER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "LnFollower"

//
// LnFollow options.
//
#define LNFOLLOWO_MECANUM_DRIVE 0x00000001

/**
 * This abstract class defines the SensorArray object. The object
 * is a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who will provide the
 * sensor array functions.
 */
class SensorArray
{
public:
    /**
     * This function is provided by the subclass to read the sensors and
     * return the combined raw value.
     */
    virtual
    UINT32
    GetRawValue(
        void
        ) = 0;

    /**
     * This function is provided by the subclass to return the mapped value
     * from the given raw sensor value.
     */
    virtual
    float
    GetMappedValue(
        __in UINT32 rawValue
        ) = 0;
};  //class SensorArray

/**
 * This class defines and implements the LineFollower object. It consists
 * of a number of light sensors, a drive object and a PID controller that
 * drives the robot using the feedback of the light sensors.
 */
class LineFollower: public CoopTask
{
private:
    #define LNFOLLOWF_STARTED           0x00000001
    #define INVALID_SENSOR_VALUE        0xffffffff

    RobotDrive  *m_drive;
    TrcPIDCtrl  *m_pidCtrl;
    SensorArray *m_sensorArray;
    UINT32       m_lnFollowOptions;
    UINT32       m_lnFollowFlags;
    UINT32       m_stopSensorValue;
    float        m_maxDrivePower;
    float        m_findLineDrivePower;
    float        m_findLineTurnPower;
    Event       *m_notifyEvent;
    UINT32       m_expiredTime;

public:
    /**
     * Constructor: Create an instance of the LineFollower object that consists
     * of a sensor array object, a RobotDrive object, a TrcPIDCtrl object
     * for PID turn and optionally a notification object for the callback.
     *
     * @param drive Points to the RobotDrive object.
     * @param pidCtrl Points to the TrcPIDCtrl object for line following.
     * @param sensorArray Points to the sensor array object.
     * @param lnFollowOptions Specifies option flags
     */
    LineFollower(
        __in RobotDrive    *drive,
        __in TrcPIDCtrl    *pidCtrl,
        __in SensorArray   *sensorArray,
        __in UINT32         lnFollowOptions = 0
        ): m_drive(drive),
           m_pidCtrl(pidCtrl),
           m_sensorArray(sensorArray),
           m_lnFollowOptions(lnFollowOptions),
           m_stopSensorValue(INVALID_SENSOR_VALUE),
           m_maxDrivePower(1.0),
           m_findLineDrivePower(0.5),
           m_findLineTurnPower(0.5),
           m_notifyEvent(NULL),
           m_expiredTime(0)
    {
        TLevel(INIT);
        TEnterMsg(("drive=%p,pidCtrl=%p,sensorArray=%p,options=%x",
                   drive, pidCtrl, sensorArray, lnFollowOptions));

        RegisterTask(TASK_STOP | TASK_PROCESS_ACTION);

        TExit();
    }   //LineFollower

    /**
     * Destructor: Destroy an instance of the LineFollower object.
     */
    virtual
    ~LineFollower(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~LineFollower

    /**
     * This function stops the line follower.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
        {
            m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
        }
        else
        {
            m_drive->Drive(0.0, 0.0);
        }
        m_pidCtrl->Reset();
        m_lnFollowFlags = 0;

        TExit();
        return;
    }   //Stop

    /**
     * This function is called by TaskMgr to stop the LineFollower.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    StopTask(
        __in UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        Stop();

        TExit();
        return;
    }   //StopTask

    /**
     * This function starts the line follower.
     *
     * @param lineCenterValue Specifies the sensor mapped value of the line
     *        center.
     * @param stopSensorValue Specifies the raw sensor value for the stopping
     *        condition.
     * @param maxDrivePower Specifies the maximum line following drive power.
     * @param findLineDrivePower Specifies the drive power when finding the
     *        line.
     * @param findLineTurnPower Specifies the turn power when finding the line.
     * @param notifyEvent Specifies the optional notification event.
     * @param timeout Specific the timeout in msec. No timeout if zero.
     */
    void
    LineFollowStart(
        __in float  lineCenterValue,
        __in UINT32 stopSensorValue = INVALID_SENSOR_VALUE,
        __in float  maxDrivePower = 1.0,
        __in float  findLineDrivePower = 0.5,
        __in float  findLineTurnPower = 0.5,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        UINT32 sensorRawValue = m_sensorArray->GetRawValue();

        TLevel(API);
        TEnterMsg(("lineCenter=%f,stopValue=%x,maxDrive=%f,findDrive=%f,findTurn=%f,notifyEvent=%p,timeout=%d",
                   lineCenterValue, stopSensorValue, maxDrivePower,
                   findLineDrivePower, findLineTurnPower, notifyEvent,
                   timeout));

        m_stopSensorValue = stopSensorValue;
        m_maxDrivePower = maxDrivePower;
        m_findLineDrivePower = findLineDrivePower;
        m_findLineTurnPower = findLineTurnPower;
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

        m_pidCtrl->SetTarget(lineCenterValue,
                             m_sensorArray->GetMappedValue(sensorRawValue));
        m_lnFollowFlags |= LNFOLLOWF_STARTED;

        TExit();
        return;
    }   //LineFollowStart

    /**
     * This function is called by the TaskMgr to update the LineFollower state
     * and check for completion.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    ProcessAction(
        __in UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if (m_lnFollowFlags & LNFOLLOWF_STARTED)
        {
            UINT32 sensorRawValue = m_sensorArray->GetRawValue();

            if ((m_stopSensorValue != INVALID_SENSOR_VALUE) &&
                (sensorRawValue == m_stopSensorValue) ||
                (m_expiredTime != 0) && (GetMsecTime() >= m_expiredTime))
            {
                //
                // Detected stop condition.
                //
                Stop();
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                }
            }
            else if (sensorRawValue != 0)
            {
                float sensorMappedValue = m_sensorArray->GetMappedValue(
                                                sensorRawValue);
                float turnPower = m_pidCtrl->CalcPIDOutput(sensorMappedValue);
                float drivePower = m_maxDrivePower*(1.0 - fabs(turnPower));

                if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(drivePower, 0.0, turnPower);
                }
                else
                {
                    m_drive->ArcadeDrive(drivePower, turnPower);
                }
            }
            else
            {
                //
                // We lost the line. Find it again.
                //
                if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(m_findLineDrivePower,
                                                0.0,
                                                m_findLineTurnPower);
                }
                else
                {
                    m_drive->ArcadeDrive(m_findLineDrivePower,
                                         m_findLineTurnPower);
                }
            }
        }

        TExit();
        return;
    }   //ProcessAction
};  //class LineFollower

#endif  //ifndef _LINEFOLLOWER_H
