#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the physical definitions of the robot.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// Input Subsystem.
//
#define JSPORT_DRIVER_LEFT              1
#define JSPORT_DRIVER_RIGHT             2
#define JSPORT_KICKER                   3

//
// DriveBase Subsystem.
//
#define PWM_FRONTLEFT_MOTOR             1
#define PWM_FRONTRIGHT_MOTOR            2
#define PWM_REARLEFT_MOTOR              3
#define PWM_REARRIGHT_MOTOR             4

#define MOTOR_FRONTLEFT_REVERSE         false
#define MOTOR_FRONTRIGHT_REVERSE        true
#define MOTOR_REARLEFT_REVERSE          false
#define MOTOR_REARRIGHT_REVERSE         true

#define AIN_GYRO_DRIVE                  1

#define CAMERA_KP                       0.1
#define CAMERA_KI                       0.00001
#define CAMERA_KD                       0.0
#define CAMERA_TURN_TOLERANCE           0.5
#define CAMERA_TURN_SETTLING            200
#define MIN_TARGET_SCORE                0.01

#define ACCEL_KP                        0.25
#define ACCEL_KI                        0.0
#define ACCEL_KD                        1.04
#define ACCEL_DRIVE_TOLERANCE           0.5
#define ACCEL_DRIVE_SETTLING            200

#define GYRO_KP                         0.11
#define GYRO_KI                         0.0
#define GYRO_KD                         1.04
#define GYRO_TURN_TOLERANCE             0.5
#define GYRO_TURN_SETTLING              200

#define DRIVE_RANGE_MIN                 -0.5
#define DRIVE_RANGE_MAX                 0.5

//
// Kicker Subsystem.
//
#define SOL_KICKER_LEFT_OUT             1
#define SOL_KICKER_LEFT_IN              2
#define SOL_KICKER_RIGHT_OUT            3
#define SOL_KICKER_RIGHT_IN             4

#define DIN_FEELER_LEFT                 1
#define DIN_FEELER_RIGHT                2

#define KICKER_EXTEND_PERIOD            0.5
#define KICKER_RETRACT_PERIOD           0.5

//
// Lid Subsystem.
//
#define SOL_LID_UP                      5
#define SOL_LID_DOWN                    6

#define LID_HOLD_PERIOD                 0.5

//
// Miscellaneous.
//
#define RELAY_COMPRESSOR                1
#define DIN_CMP_PRESSURE_SW             3
#define DIN_NUMBALLS_SW1                14
#define DIN_NUMBALLS_SW2                13
#define DIPSwitchValue(x)               (((~(x)) >> 2) & 0x03)

#endif  //ifndef _ROBOTINFO_H

