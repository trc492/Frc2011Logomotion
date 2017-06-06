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
// DriveBase Subsystem.
//
#if defined(_USE_WHEEL_ENCODERS)

#define DIN_FRONT_LEFT_ENCODER_A        5
#define DIN_FRONT_LEFT_ENCODER_B        6

#define DIN_REAR_LEFT_ENCODER_A         7
#define DIN_REAR_LEFT_ENCODER_B         8

#define DIN_FRONT_RIGHT_ENCODER_A       9
#define DIN_FRONT_RIGHT_ENCODER_B       10

#define DIN_REAR_RIGHT_ENCODER_A        11
#define DIN_REAR_RIGHT_ENCODER_B        12

#define DISTANCE_PER_CLICK              1.0

#elif defined(_USE_TRACKING_WHEELS)

#define DIN_X_TRACKING_ENCODER_A        1
#define DIN_X_TRACKING_ENCODER_B        2

#define DIN_Y_TRACKING_ENCODER_A        3
#define DIN_Y_TRACKING_ENCODER_B        4

#define DISTANCE_PER_CLICK              1.0

#endif

//
// We are not going to use both the left light sensor and the right rear
// encoder at the same time.
//
#ifdef _USE_LINE_FOLLOWER
#define DIN_LEFT_LIGHTSENSOR            12	
#define DIN_CENTER_LIGHTSENSOR          13
#define DIN_RIGHT_LIGHTSENSOR           14
#endif

#define AIN_GYRO                        1

#ifdef _USE_LINE_FOLLOWER
#define SOL_LIGHT_SENSOR_POWER          1
#endif

#define PWM_FRONT_LEFT_MOTOR            1
#define PWM_REAR_LEFT_MOTOR             2
#define PWM_FRONT_RIGHT_MOTOR           3
#define PWM_REAR_RIGHT_MOTOR            4

#define MOTOR_FRONT_LEFT_REVERSE        false //purple
#define MOTOR_REAR_LEFT_REVERSE         false //orange
#define MOTOR_FRONT_RIGHT_REVERSE       true  //red
#define MOTOR_REAR_RIGHT_REVERSE        true  //black

#if defined(_USE_WHEEL_ENCODERS)
#define DRIVE_KP                        0.25
#define DRIVE_KI                        0.0     //0.001
#define DRIVE_KD                        1.04    //0.5
#define DRIVE_TOLERANCE                 0.005
#define DRIVE_SETTLING                  200
#elif defined(_USE_TRACKING_WHEELS)
#define DRIVE_KP                        0.25
#define DRIVE_KI                        0.0     //0.001
#define DRIVE_KD                        1.04    //0.5
#define DRIVE_TOLERANCE                 0.005
#define DRIVE_SETTLING                  200
#elif defined(_USE_ACCELEROMETER)
#define DRIVE_KP                        0.25
#define DRIVE_KI                        0.0     //0.001
#define DRIVE_KD                        1.04    //0.5
#define DRIVE_TOLERANCE                 0.005
#define DRIVE_SETTLING                  200
#endif

#define TURN_KP                         0.11
#define TURN_KI                         0.0     //0.001
#define TURN_KD                         1.04    //0.5
#define TURN_TOLERANCE                  0.005
#define TURN_SETTLING                   200

#ifdef _USE_LINE_FOLLOWER
#define LNFOLLOW_KP                     0.15
#define LNFOLLOW_KI                     0.0
#define LNFOLLOW_KD                     0.0
#define LNFOLLOW_TOLERANCE              0.0
#define LNFOLLOW_SETTLING               200

#define LNFOLLOW_LS_NOLINE              0x0
#define LNFOLLOW_LS_Y                   0x5
#define LNFOLLOW_LS_T                   0x7

#define FIND_LINE_DRIVE                 0.3
#define FIND_LINE_TURN                  0.2
#define FOLLOW_LINE_MAXIMUM_SPEED       0.3
#define FOLLOW_LINE_MEDIUM_SPEED        0.2
#define FOLLOW_LINE_SLOW_SPEED          0.15

#define LNFOLLOW_DIR_STRAIGHT           0
#define LNFOLLOW_DIR_FORKLEFT           1
#define LNFOLLOW_DIR_FORKRIGHT          2
#endif

#define DRIVE_RANGE_MIN                 -0.5
#define DRIVE_RANGE_MAX                 0.5

//
// Input Subsystems
//
#define JSPORT_DRIVER_LEFT              1
#define JSPORT_DRIVER_RIGHT             2

#endif  //ifndef _ROBOTINFO_H

