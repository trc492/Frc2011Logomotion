#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDDrive.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDDrive class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDDRIVE_H
#define _TRCPIDDRIVE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDDRIVE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDDrive"

//
// PIDDrive options
//
#define PIDDRIVEO_MECANUM_DRIVE 0x00000001

/**
 * This class defines and implements the TrcPIDDrive object. It supports the
 * capability of PID controlled drive by distance and/or PID controlled turn
 * by angle. After the operation is completed, it has an option of notifying
 * the caller by setting an event. Therefore, it can work with the state
 * machine to do autonomous drive.
 */
class TrcPIDDrive
{
private:
    //
    // Flags
    //
    #define PIDDRIVEF_PIDDRIVE_ON       0x00000001
    #define PIDDRIVEF_STOP_ONTARGET     0x00000002
    #define PIDDRIVEF_TURN_ONLY         0x00000004

    RobotDrive *m_drive;
    TrcPIDCtrl *m_pidCtrlDrive;
    TrcPIDCtrl *m_pidCtrlTurn;
    PIDInput   *m_pidInput;
    UINT32      m_pidDriveOptions;
    UINT32      m_pidDriveFlags;
    Event      *m_notifyEvent;
    UINT32      m_expiredTime;

public:
    /**
     * Constructor: Create an instance of the TrcPIDDrive object that consists
     * of a RobotDrive object, a TrcPIDCtrl object for drive, a TrcPIDCtrl
     * object for turn and optionally a notification object for the callback.
     *
     * @param drive Points to the RobotDrive object.
     * @param pidCtrlDrive Points to the TrcPIDCtrl object for drive straight.
     * @param pidCtrlTurn Points to the TrcPIDCtrl object for turning.
     * @param pidInput Specifies the PIDInput object.
     * @param pidDriveOptions Specifies PIDDrive options.
     */
    TrcPIDDrive(
        __in RobotDrive *drive,
        __in TrcPIDCtrl *pidCtrlDrive,
        __in TrcPIDCtrl *pidCtrlTurn,
        __in PIDInput   *pidInput,
        __in UINT32      pidDriveOptions = 0
        ): m_drive(drive),
           m_pidCtrlDrive(pidCtrlDrive),
           m_pidCtrlTurn(pidCtrlTurn),
           m_pidInput(pidInput),
           m_pidDriveOptions(pidDriveOptions),
           m_pidDriveFlags(0),
           m_notifyEvent(NULL),
           m_expiredTime(0)
    {
        TLevel(INIT);
        TEnterMsg(("drive=%p,pidCtrlDrive=%p,pidCtrlTurn=%p,pidInput=%p,options=%x",
                   drive, pidCtrlDrive, pidCtrlTurn, pidInput,
                   pidDriveOptions));
        TExit();
    }   //TrcPIDDrive

    /**
     * Destructor: Destroy an instance of the TrcPIDDrive object.
     */
    ~TrcPIDDrive(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TrcPIDDrive

    /**
     * This function resets the PID Drive object.
     */
    void
    Reset(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE)
        {
            m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
        }
        else
        {
            m_drive->Drive(0.0, 0.0);
        }
        m_pidCtrlDrive->Reset();
        m_pidCtrlTurn->Reset();
        m_pidDriveFlags = 0;

        TExit();
        return;
    }   //Reset

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distSetPoint Specifies the target distance relative to current
     *        distance.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    SetTarget(
        __in float  distSetPoint,
        __in float  angleSetPoint,
        __in bool   fStopOnTarget = true,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distSetPt=%f,angleSetPt=%f,fStopOnTarget=%x,event=%p,timeout=%d",
                   distSetPoint, angleSetPoint, fStopOnTarget, notifyEvent, timeout));

        m_pidCtrlDrive->SetTarget(distSetPoint,
                                  m_pidInput->GetInput(m_pidCtrlDrive));
        m_pidCtrlTurn->SetTarget(angleSetPoint,
                                 m_pidInput->GetInput(m_pidCtrlTurn));
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

        m_pidDriveFlags = PIDDRIVEF_PIDDRIVE_ON;
        if (fStopOnTarget)
        {
            m_pidDriveFlags |= PIDDRIVEF_STOP_ONTARGET;
        }

        if ((distSetPoint == 0.0) && (angleSetPoint != 0.0))
        {
            m_pidDriveFlags |= PIDDRIVEF_TURN_ONLY;
        }
        else
        {
            m_pidDriveFlags &= ~PIDDRIVEF_TURN_ONLY;
        }

        TExit();
        return;
    }   //SetTarget

    /**
     * This function is called by the robot loop periodically to update the
     * PIDDrive state and check for completion.
     */
    void
    Task(
        void
        )
    {
        TLevel(TASK);
        TEnter();

        if (m_pidDriveFlags & PIDDRIVEF_PIDDRIVE_ON)
        {
            if (m_pidCtrlTurn->OnTarget() &&
                ((m_pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
                 m_pidCtrlDrive->OnTarget()) ||
                (m_expiredTime != 0) && (GetMsecTime() >= m_expiredTime))
            {
                if (m_pidDriveFlags & PIDDRIVEF_STOP_ONTARGET)
                {
                    Reset();
                    if (m_notifyEvent != NULL)
                    {
                        m_notifyEvent->SetEvent();
                    }
                }
                else if (m_pidDriveFlags & PIDDRIVEO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
                }
                else
                {
                    m_drive->Drive(0.0, 0.0);
                }
            }
            else if (m_pidDriveFlags & PIDDRIVEO_MECANUM_DRIVE)
            {
                m_drive->MecanumDrive_Polar(
                        m_pidCtrlDrive->CalcPIDOutput(
                                m_pidInput->GetInput(m_pidCtrlDrive)),
                        0.0,
                        m_pidCtrlTurn->CalcPIDOutput(
                                m_pidInput->GetInput(m_pidCtrlTurn)));
            }
            else
            {
                m_drive->ArcadeDrive(
                        m_pidCtrlDrive->CalcPIDOutput(
                                m_pidInput->GetInput(m_pidCtrlDrive)),
                        m_pidCtrlTurn->CalcPIDOutput(
                                m_pidInput->GetInput(m_pidCtrlTurn)));
            }
        }

        TExit();
        return;
    }   //Task
};  //class TrcPIDDrive

#endif  //ifndef _TRCPIDDRIVE_H
