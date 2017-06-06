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

//
// Library includes.
//
#include "WPILib.h"
#include "../trclib/TrcLib.h"

//
// Tracing info.
//
#define MOD_DRIVEBASE           TGenModId(1)
#define MOD_LADDER              TGenModId(2)
#define MOD_HANGER              TGenModId(3)
#define MOD_DEPLOYER            TGenModId(4)

#define TRACE_MODULES           (MOD_DRIVEBASE | \
                                 MOD_LADDER)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// Project includes.
//
#include "DashboardDataFormat.h"
#include "RobotInfo.h"          //Robot configurations

#include "Deployer.h"           //Deployer subsystem
#include "Hanger.h"             //Hanger subsystem
#include "Ladder.h"             //Ladder subsystem
#include "DriveBase.h"          //DriveBase subsystem

#include "TrcRobot.h"           //Main Robot module
#include "Auto.h"               //Autonomous mode
#include "TeleOp.h"             //TeleOp mode

START_ROBOT_CLASS(TrcRobot);
