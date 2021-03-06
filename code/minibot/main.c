#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Sensor, S2,     startButton,         sensorTouch)
#pragma config(Sensor, S3,     topButton,           sensorTouch)
#pragma config(Sensor, S4,     accel,               sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     motor1,        tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     motor2,        tmotorNormal, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="main.c" />
///
/// <summary>
///     This module contains the entry point for the program.
///     This is template file and should not be modified. The main competition
///     code should live in MiniBot.h.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "JoystickDriver.c"
#include "..\trclib.nxt\trcdefs.h"
#include "..\trclib.nxt\dbgtrace.h"
#include "MiniBot.h"

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    //
    // The RobotInit function is provided by student code in main.h.
    //
    RobotInit();

    nextTime = nPgmTime;
    while (true)
    {
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;

            //
            // The following functions are provided by student code in main.h.
            //
            InputTasks();
            MainTasks();
            OutputTasks();
        }

        wait1Msec(1);
    }
}   //main
