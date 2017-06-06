#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDMotor.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDMotor class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDMOTOR_H
#define _TRCPIDMOTOR_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDMOTOR
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDMotor"

/**
 * This class defines and implements the TrcPIDMotor object. It supports the
 * capability of PID controlled movement by a motor. It drives the motor to
 * the set target position and optionally signals the notification event for
 * notifying completion.
 */
class TrcPIDMotor
{
private:
    //
    // Flags
    //
    #define PIDMOTORF_MOTOR_ON          0x00000001
    #define PIDMOTORF_STOP_ONTARGET     0x00000002

    SpeedController *m_motor;
    TrcPIDCtrl      *m_pidCtrl;
    PIDInput        *m_pidInput;
    UINT32           m_pidMotorOptions;
    UINT32           m_pidMotorFlags;
    Event           *m_notifyEvent;
    UINT32           m_expiredTime;

public:
    /**
     * Constructor: Create an instance of the TrcPIDMotor object that consists
     * of a SpeedController object, a TrcPIDCtrl object and optionally a
     * notification event for signaling completion.
     *
     * @param motor Points to the SpeedController object.
     * @param pidCtrl Points to the TrcPIDCtrl object.
     * @param pidInput Specifies the PIDInput object.
     */
    TrcPIDMotor(
        __in SpeedController *motor,
        __in TrcPIDCtrl      *pidCtrl,
        __in PIDInput        *pidInput,
        __in UINT32           pidMotorOptions = 0
        ): m_motor(motor),
           m_pidCtrl(pidCtrl),
           m_pidInput(pidInput),
           m_pidMotorOptions(pidMotorOptions),
           m_pidMotorFlags(0),
           m_notifyEvent(NULL),
           m_expiredTime(0)
    {
        TLevel(INIT);
        TEnterMsg(("motor=%p,pidCtrl=%p,pidInput=%p,options=%x",
                   motor, pidCtrl, pidInput, pidMotorOptions));
        TExit();
    }   //TrcPIDMotor

    /**
     * Destructor: Destroy an instance of the TrcPIDMotor object.
     */
    ~TrcPIDMotor(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TrcPIDMotor

    /**
     * This function resets the PID Motor object.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_motor->Set(0.0);
        m_pidCtrl->Reset();
        m_pidMotorFlags = 0;

        TExit();
        return;
    }   //Stop

    /**
     * This function sets PID motor target with the given setpoint.
     *
     * @param setPoint Specifies the target setPoint.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    SetTarget(
        __in float  setPoint,
        __in bool   fStopOnTarget = true,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("setPoint=%f,fStopOnTarget=%x,event=%p,timeout=%d",
                   setPoint, fStopOnTarget, notifyEvent, timeout));

        m_pidCtrl->SetTarget(setPoint, m_pidInput->GetInput(m_pidCtrl));
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;
        m_pidMotorFlags = PIDMOTORF_MOTOR_ON;
        if (fStopOnTarget)
        {
            m_pidMotorFlags |= PIDMOTORF_STOP_ONTARGET;
        }

        TExit();
        return;
    }   //SetTarget

    /**
     * This function is called by the robot loop periodically to update the
     * PIDMotor state and check for completion.
     */
    void
    Task(
        void
        )
    {
        TLevel(TASK);
        TEnter();

        if (m_pidMotorFlags & PIDMOTORF_MOTOR_ON)
        {
            if ((m_pidMotorFlags & PIDMOTORF_STOP_ONTARGET) &&
                m_pidCtrl->OnTarget() ||
                (m_expiredTime != 0) && (GetMsecTime() >= m_expiredTime))
            {
                Stop();
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                }
            }
            else
            {
                float output = m_pidCtrl->CalcPIDOutput(
                                m_pidInput->GetInput(m_pidCtrl));
                m_motor->Set(output);
                TSampling(("MotorOutput: %f", output));
            }
        }

        TExit();
        return;
    }   //Task
};  //class TrcPIDMotor

#endif  //ifndef _TRCPIDMOTOR_H
