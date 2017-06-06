#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="main.cpp" />
///
/// <summary>
///     This main module contains the entry point of the program.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#define _DBGTRACE_ENABLED
#define _USE_TANK_DRIVE
#define _USE_WHEEL_ENCODERS
//#define _USE_TRACKING_WHEELS
//#define _USE_ACCELEROMETER
//#define _USE_LINE_FOLLOWER
//#define _USE_CAMERA

//
// Library includes.
//
#include "WPILib.h"
#include "../trclib/TrcLib.h"

//
// Tracing info.
//
#define MOD_DRIVEBASE           TGenModId(1)

#define TRACE_MODULES           (MOD_MAIN | \
                                 MOD_DRIVEBASE)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// Project includes.
//
#include "DashboardDataFormat.h"
#include "RobotInfo.h"          //Robot configurations

#include "DriveBase.h"          //DriveBase subsystem

#include "TrcRobot.h"           //Main Robot module
#include "Auto.h"               //Autonomous mode
#include "TeleOp.h"             //TeleOp mode

START_ROBOT_CLASS(TrcRobot);
