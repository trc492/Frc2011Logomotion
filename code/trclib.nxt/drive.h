#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="drive.h" />
///
/// <summary>
///     This module contains the library functions for the drive subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DRIVE_H
#define _DRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVE

//
// Constants.
//
#define DRIVEF_ON               0x0100
#define DRIVEF_STALL_PROTECT_ON 0x0200
#define DRIVEF_STALLED          0x0400

#define MIN_STALL_POWER         20
#define STALL_TIME              2000    //2 seconds

//
// Macros.
//
#define NORMALIZE_DRIVE(x,m,n)  NORMALIZE(x, m, n, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)

//
// Type definitions.
//
typedef struct
{
    int   leftMotor;
    int   rightMotor;
    float distPerClick;
    float degreesPerClick;
    int   driveFlags;
    int   leftPower;
    int   rightPower;
    int   leftEncoder;
    int   rightEncoder;
    long  stallTimer;
} DRIVE;

/**
 *  This function stops the motors in the drive system.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveStop(
    __out DRIVE &drive
    )
{
    TFuncName("DriveStop");
    TLevel(API);
    TEnter();

    drive.driveFlags &= ~DRIVEF_ON;
    drive.leftPower = 0;
    drive.rightPower = 0;
    //
    // Stop the motors.
    //
    motor[drive.leftMotor] = 0;
    motor[drive.rightMotor] = 0;
    drive.leftEncoder = 0;
    drive.rightEncoder = 0;
    drive.stallTimer = 0;

    TExit();
    return;
}   //DriveStop

/**
 *  This function resets the drive system.
 *
 *  @param drive Points to the DRIVE structure to be reset.
 */
void
DriveReset(
    __out DRIVE &drive
    )
{
    TFuncName("DriveReset");
    TLevel(API);
    TEnter();

    DriveStop(drive);
//    nMotorEncoder[drive.leftMotor] = 0;
//    nMotorEncoder[drive.rightMotor] = 0;
    drive.driveFlags &= ~DRIVEF_STALLED;

    TExit();
    return;
}   //DriveReset

/**
 *  This function initializes the drive system.
 *
 *  @param drive Points to the DRIVE structure to be initialized.
 *  @param leftMotor Specifies the left motor.
 *  @param rightMotor Specifies the right motor.
 */
void
DriveInit(
    __out DRIVE &drive,
    __in  int leftMotor,
    __in  int rightMotor,
    __in  float distPerClick,
    __in  float degreesPerClick
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    drive.leftMotor = leftMotor;
    drive.rightMotor = rightMotor;
    drive.distPerClick = distPerClick;
    drive.degreesPerClick = degreesPerClick;
    drive.driveFlags = 0;
    DriveReset(drive);

    TExit();
    return;
}   //DriveInit

/**
 *  This function enables or disables stall protection.
 *
 *  @param drive Points to the DRIVE structure to be initialized.
 *  @param fOn If true, enables stall protection.
 */
void
DriveStallProtect(
    __inout DRIVE &drive,
    __in    bool fOn
    )
{
    TFuncName("DriveStallProtect");
    TLevel(API);
    TEnterMsg(("fOn=%d", (byte)fOn));

    if (fOn)
    {
        drive.driveFlags |= DRIVEF_STALL_PROTECT_ON;
    }
    else
    {
        drive.driveFlags &= ~DRIVEF_STALL_PROTECT_ON;
    }

    TExit();
    return;
}   //DriveStallProtect

/**
 *  This function sets power of the motors for tank drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftPower Specifies the left motor power.
 *  @param rightPower Specifies the right motor power.
 */
void
DriveTank(
    __out DRIVE &drive,
    __in  int leftPower,
    __in  int rightPower
    )
{
    TFuncName("DriveTank");
    TLevel(API);
    TEnterMsg(("Left=%d,Right=%d", leftPower, rightPower));

    drive.leftPower = BOUND(leftPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    drive.rightPower = BOUND(rightPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveTank

/**
 *  This function sets power of the motors for arcade drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param drivePower Specifies the drive power.
 *  @param turnPower Specifies the turn power.
 */
void
DriveArcade(
    __out DRIVE &drive,
    __in  int drivePower,
    __in  int turnPower
    )
{
    TFuncName("DriveArcade");
    TLevel(API);
    TEnterMsg(("Drive=%d,Turn=%d", drivePower, turnPower));

    drivePower = BOUND(drivePower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    turnPower = BOUND(turnPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drivePower + turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward right:
        //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
        //
        drive.leftPower = MOTOR_MAX_VALUE;
        drive.rightPower = -2*turnPower + MOTOR_MAX_VALUE;
    }
    else if (drivePower - turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward left:
        //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
        //
        drive.leftPower = 2*turnPower + MOTOR_MAX_VALUE;
        drive.rightPower = MOTOR_MAX_VALUE;
    }
    else if (drivePower + turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward left:
        //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
        //
        drive.leftPower = MOTOR_MIN_VALUE;
        drive.rightPower = -2*turnPower + MOTOR_MIN_VALUE;
    }
    else if (drivePower - turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward right:
        //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
        //
        drive.leftPower = 2*turnPower + MOTOR_MIN_VALUE;
        drive.rightPower = MOTOR_MIN_VALUE;
    }
    else
    {
        drive.leftPower = drivePower + turnPower;
        drive.rightPower = drivePower - turnPower;
    }
    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveArcade

/**
 *  This function performs the driving task according to the drive state.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveTask(
    __inout DRIVE &drive
    )
{
    TFuncName("DriveTask");
    TLevel(TASK);
    TEnter();

    if (drive.driveFlags & DRIVEF_ON)
    {
        if ((drive.driveFlags & DRIVEF_STALLED) == 0)
        {
            motor[drive.leftMotor] = drive.leftPower;
            motor[drive.rightMotor] = drive.rightPower;
            if (drive.driveFlags & DRIVEF_STALL_PROTECT_ON)
            {
                long currTime = nPgmTime;
                if ((drive.stallTimer == 0) ||
                    (abs(drive.leftPower) <= MIN_STALL_POWER) &&
                    (abs(drive.rightPower) <= MIN_STALL_POWER) ||
                    (nMotorEncoder[drive.leftMotor] != drive.leftEncoder) ||
                    (nMotorEncoder[drive.rightMotor] != drive.rightEncoder))
                {
                    drive.leftEncoder = nMotorEncoder[drive.leftMotor];
                    drive.rightEncoder = nMotorEncoder[drive.rightMotor];
                    drive.stallTimer = currTime;
                }

                if (currTime - drive.stallTimer >= STALL_TIME)
                {
                    motor[drive.leftMotor] = 0;
                    motor[drive.rightMotor] = 0;
                    drive.driveFlags |= DRIVEF_STALLED;
                    PlayImmediateTone(1000, 100);
                }
            }
        }
    }
    else
    {
        motor[drive.leftMotor] = 0;
        motor[drive.rightMotor] = 0;
    }

    TExit();
    return;
}   //DriveTask

#endif  //ifndef _DRIVE_H
