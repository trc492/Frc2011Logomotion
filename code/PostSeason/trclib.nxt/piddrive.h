#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="piddrive.h" />
///
/// <summary>
///     This module contains the library functions for the PID drive
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDDRIVE_H
#define _PIDDRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDDRIVE

//
// Constants.
//
#define PIDDRIVEF_PIDMODE_ON    0x0100
#define PIDDRIVEF_STOP_ONTARGET 0x0200
#define PIDDRIVEF_USER_MASK     0x00ff
#define PIDDRIVEF_ENABLE_EVENTS 0x0001

//
// Type definitions.
//
typedef struct
{
//    DRIVE   &drive;
//    PIDCTRL &drivePIDCtrl;
//    PIDCTRL &turnPIDCtrl;
    int      drive;
    int      drivePIDCtrl;
    int      turnPIDCtrl;
    int      pidDriveFlags;
} PIDDRIVE;

//
// Import function prototypes.
//
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    );

/**
 *  This function resets the PID Drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure to be reset.
 */
void
PIDDriveReset(
    __out PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveReset");
    TLevel(API);
    TEnter();

    pidDrive.pidDriveFlags &= ~PIDDRIVEF_PIDMODE_ON;
//    DriveReset(pidDrive.drive);
//    PIDCtrlReset(pidDrive.drivePIDCtrl);
//    PIDCtrlReset(pidDrive.turnPIDCtrl);
    DriveReset(g_Drive[pidDrive.drive]);
    PIDCtrlReset(g_PIDCtrl[pidDrive.drivePIDCtrl]);
    PIDCtrlReset(g_PIDCtrl[pidDrive.turnPIDCtrl]);
    DriveStallProtect(g_Drive[pidDrive.drive], false);

    TExit();
    return;
}   //PIDDriveReset

/**
 *  This function initializes the drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure to be initialized.
 *  @param drive Points to the DRIVE structure.
 *  @param drivePIDCtrl Points to the PIDCTRL structure for drive.
 *  @param turnPIDCtrl Points to the PIDCTRL structure for turn.
 *  @param pidDriveFlags Specifies the drive flags.
 */
void
PIDDriveInit(
    __out PIDDRIVE &pidDrive,
//    __in  DRIVE &drive,
//    __in  PIDCTRL &drivePIDCtrl,
//    __in  PIDCTRL &turnPIDCtrl,
    __in  int drive,
    __in  int drivePIDCtrl,
    __in  int turnPIDCtrl,
    __in  int pidDriveFlags
    )
{
    TFuncName("PIDDriveInit");
    TLevel(INIT);
    TEnter();

    pidDrive.drive = drive;
    pidDrive.drivePIDCtrl = drivePIDCtrl;
    pidDrive.turnPIDCtrl = turnPIDCtrl;
    pidDrive.pidDriveFlags = pidDriveFlags & PIDDRIVEF_USER_MASK;

    TExit();
    return;
}   //PIDDriveInit

/**
 *  This function sets PID drive target with the given drive distance and
 *  turn angle setpoints.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param distSetPoint Specifies the target distance to travel.
 *  @param distTolerance Specifies the distance tolerance when determining
 *         OnTarget.
 *  @param angleSetPoint Specifies the target angle to turn.
 *  @param angleTolerance Specifies the angle tolerance when determining
 *         OnTarget.
 *  @param fStopOnTarget If true, stop PIDDrive when target is reached.
 *         Otherwise, continue to monitor the target and readjust if necessary.
 */
void
PIDDriveSetTarget(
    __out PIDDRIVE &pidDrive,
    __in  float distSetPoint,
    __in  float distTolerance,
    __in  float angleSetPoint,
    __in  float angleTolerance,
    __in  bool fStopOnTarget
    )
{
    TFuncName("PIDDriveSetTarget");
    TLevel(API);
    TEnterMsg(("D=%5.1f,A=%5.1f", distSetPoint, angleSetPoint));

//    PIDCtrlSetTarget(pidDrive.drivePIDCtrl, distSetPoint);
//    PIDCtrlSetTarget(pidDrive.turnPIDCtrl, angleSetPoint);
    PIDCtrlSetTarget(g_PIDCtrl[pidDrive.drivePIDCtrl],
                     distSetPoint,
                     distTolerance);
    PIDCtrlSetTarget(g_PIDCtrl[pidDrive.turnPIDCtrl],
                     angleSetPoint,
                     angleTolerance);
    if (fStopOnTarget)
    {
        pidDrive.pidDriveFlags |= PIDDRIVEF_STOP_ONTARGET;
    }
    else
    {
        pidDrive.pidDriveFlags &= ~PIDDRIVEF_STOP_ONTARGET;
    }
    pidDrive.pidDriveFlags |= PIDDRIVEF_PIDMODE_ON;
    DriveStallProtect(g_Drive[pidDrive.drive], true);

    TExit();
    return;
}   //PIDDriveSetTarget

/**
 *  This function performs the PID drive.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 */
void
PIDDriveTask(
    __inout PIDDRIVE &pidDrive
    )
{
    int output;

    TFuncName("PIDDriveTask");
    TLevel(TASK);
    TEnter();

    if (pidDrive.pidDriveFlags & PIDDRIVEF_PIDMODE_ON)
    {
//        int drivePower = (int)PIDCtrlOutput(pidDrive.drivePIDCtrl);
//        int turnPower = (int)PIDCtrlOutput(pidDrive.turnPIDCtrl);
        int drivePower = (int)PIDCtrlOutput(g_PIDCtrl[pidDrive.drivePIDCtrl]);
        int turnPower = (int)PIDCtrlOutput(g_PIDCtrl[pidDrive.turnPIDCtrl]);
//        if (PIDCtrlIsOnTarget(pidDrive.drivePIDCtrl,
//                              pidDrive.driveTolerance) &&
//            PIDCtrlIsOnTarget(pidDrive.turnPIDCtrl,
//                              pidDrive.turnTolerance))
        if (PIDCtrlIsOnTarget(g_PIDCtrl[pidDrive.drivePIDCtrl]) &&
            PIDCtrlIsOnTarget(g_PIDCtrl[pidDrive.turnPIDCtrl]))
        {
            if (pidDrive.pidDriveFlags & PIDDRIVEF_STOP_ONTARGET)
            {
                PIDDriveReset(pidDrive);
                if (pidDrive.pidDriveFlags & PIDDRIVEF_ENABLE_EVENTS)
                {
                    PIDDriveEvent(pidDrive);
//                    nxtDisplayTextLine(
//                        1, "DE=%3.1f,TE=%3.1f",
//                        g_PIDCtrl[pidDrive.drivePIDCtrl].prevError,
//                        g_PIDCtrl[pidDrive.turnPIDCtrl].prevError);
                }
            }
            else
            {
                DriveStop(g_Drive[pidDrive.drive]);
            }
        }
        else
        {
            DriveArcade(g_Drive[pidDrive.drive], drivePower, turnPower);
//            nxtDisplayTextLine(1, "D=%d,T=%d", drivePower, turnPower);
        }
    }

    TExit();
    return;
}   //PIDDriveTask

#endif  //ifndef _PIDDRIVE_H
