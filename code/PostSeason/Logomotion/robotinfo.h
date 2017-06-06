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

//#define DRIVE_PWM_ARRANGEMENT   2010
#define DRIVE_PWM_ARRANGEMENT   2011

//
// DriveBase Subsystem.
//
#define DIN_LEFT_LIGHTSENSOR    8
#define DIN_CENTER_LIGHTSENSOR  9
#define DIN_RIGHT_LIGHTSENSOR   10
#define AIN_GYRO                1
#define SOL_LIGHT_SENSOR_POWER  1

#if DRIVE_PWM_ARRANGEMENT == 2011
    #define PWM_FRONTLEFT_MOTOR         1
    #define PWM_REARLEFT_MOTOR          2
    #define PWM_FRONTRIGHT_MOTOR        3
    #define PWM_REARRIGHT_MOTOR         4

    #define MOTOR_FRONTLEFT_REVERSE     false
    #define MOTOR_FRONTRIGHT_REVERSE    true
    #define MOTOR_REARLEFT_REVERSE      false
    #define MOTOR_REARRIGHT_REVERSE     true
#elif DRIVE_PWM_ARRANGEMENT == 2010
    #define PWM_FRONTLEFT_MOTOR         4
    #define PWM_REARLEFT_MOTOR          2
    #define PWM_FRONTRIGHT_MOTOR        3
    #define PWM_REARRIGHT_MOTOR         1

    #define MOTOR_FRONTLEFT_REVERSE     false
    #define MOTOR_FRONTRIGHT_REVERSE    true
    #define MOTOR_REARLEFT_REVERSE      false
    #define MOTOR_REARRIGHT_REVERSE     true
#endif

#define ACCEL_KP                0.25
#define ACCEL_KI                0.0     //0.001
#define ACCEL_KD                1.04    //0.5
#define ACCEL_DRIVE_TOLERANCE   0.005
#define ACCEL_DRIVE_SETTLING    200

#define GYRO_KP                 0.11
#define GYRO_KI                 0.0     //0.001
#define GYRO_KD                 1.04    //0.5
#define GYRO_TURN_TOLERANCE     0.005
#define GYRO_TURN_SETTLING      200

#define LNFOLLOW_KP             0.15
#define LNFOLLOW_KI             0.0
#define LNFOLLOW_KD             0.0
#define LNFOLLOW_TOLERANCE      0.0
#define LNFOLLOW_SETTLING       200

#define LNFOLLOW_LS_NOLINE      0x0
#define LNFOLLOW_LS_Y           0x5
#define LNFOLLOW_LS_T           0x7

#define FIND_LINE_DRIVE         0.3
#define FIND_LINE_TURN          0.2
#define FOLLOW_LINE_MAXIMUM_SPEED 0.3
#define FOLLOW_LINE_MEDIUM_SPEED 0.2
#define FOLLOW_LINE_SLOW_SPEED 0.15

#define LNFOLLOW_DIR_STRAIGHT   0
#define LNFOLLOW_DIR_FORKLEFT   1
#define LNFOLLOW_DIR_FORKRIGHT  2

#define DRIVE_RANGE_MIN         -0.5
#define DRIVE_RANGE_MAX         0.5

#define DRIVE_SLOW_SPEED_CONSTANT 0.2

//
// Ladder Subsystem
//
#define LADDER_UP_POWER         -1.0
#define LADDER_DOWN_POWER       0.5


#define DIN_LADDER_ENCODER_A    1
#define DIN_LADDER_ENCODER_B    2
#if DRIVE_PWM_ARRANGEMENT == 2011
    #define DIN_LADDER_SWITCH   4
#elif DRIVE_PWM_ARRANGEMENT == 2010
    #define DIN_LADDER_SWITCH   13
#endif

#define PWM_LADDERHEIGHT_MOTOR  6

#define LADDER_KP               0.25
#define LADDER_KI               0.0000
#define LADDER_KD               1.04
#define LADDER_TOLERANCE        0.0
#define LADDER_SETTLING         200


//#define CLICKS_PER_REVOLUTION   1440.0          //in clicks
//#define PULLEY_CIRCUMFERENCE    (PI*1.0)
//#define HEIGHT_PER_CLICK        (PULLEY_CIRCUMFERENCE/CLICKS_PER_REVOLUTION)
#define HEIGHT_PER_CLICK        (0.0019740276690159*4)

#define PEG_ZERO_HEIGHT         10

#define LADDER_INITIAL_HEIGHT   47.0

#define PICK_UP                 0
#define LOW_PEG                 30.0
#define MID_PEG                 67.0
#define TOP_PEG                 104.0
#define LOW_PEG_OFFSET          38.0
#define MID_PEG_OFFSET          75.0
#define TOP_PEG_OFFSET          112.0
#define FEEDING_SLOT            45.0
#define GROUND                  0.0

//
// Hanger Subsystem
//

#if DRIVE_PWM_ARRANGEMENT == 2011
    #define DIN_HANGER_SWITCH   7
#elif DRIVE_PWM_ARRANGEMENT == 2010
    #define DIN_HANGER_SWITCH   12
#endif

#define PWM_HANGERTOP_MOTOR     7
#define PWM_HANGERBOTTOM_MOTOR  8

#define HANGER_TOPOUT_SPEED     -1.0
#define HANGER_TOPIN_SPEED      1.0
#define HANGER_BOTTOMOUT_SPEED  1.0
#define HANGER_BOTTOMIN_SPEED   -1.0

//
// Deployer Subsystem
//
#define PWM_DEPLOYER_MOTOR      9
#define DEPLOY_ANGLE_LATCHED    0.0
#define DEPLOY_ANGLE_UNLATCHED  170.0

//
// Input Subsystems
//
#define JSPORT_DRIVER_LEFT      1
#define JSPORT_DRIVER_RIGHT     2
#define JSPORT_HANGER           3

#define DIN_BCDSW_BIT_3         11
#define DIN_BCDSW_BIT_2         12
#define DIN_BCDSW_BIT_1         13
#define DIN_BCDSW_BIT_0         14
#define BCDSwitchValue(x)       (((~(x)) >> 2) & 0x0f)

#define BCDSW_UNDEFINED         0
#define BCDSW_STRAIGHT_LOW      1
#define BCDSW_STRAIGHT_MID      2
#define BCDSW_STRAIGHT_HIGH     3
#define BCDSW_FORKLEFT_LOW      4
#define BCDSW_FORKLEFT_MID      5
#define BCDSW_FORKLEFT_HIGH     6
#define BCDSW_FORKRIGHT_LOW     7
#define BCDSW_FORKRIGHT_MID     8
#define BCDSW_FORKRIGHT_HIGH    9

#endif  //ifndef _ROBOTINFO_H

