#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="touch.h" />
///
/// <summary>
///     This module contains the library functions for the touch sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TOUCH_H
#define _TOUCH_H

#include "..\HTDriversV1.6\drivers\LEGOTS-driver.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TOUCH

//
// Constants.
//
#define TOUCHF_USER_MASK        0x00ff
#ifdef HTSMUX_STATUS
  #define TOUCHF_HTSMUX         0x0080
#endif
#define TOUCHF_ENABLE_EVENTS    0x0001

//
// Macros.
//
#ifdef HTSMUX_STATUS
    #define TouchGetState(p)    (p.touchFlags & TOUCHF_HTSMUX)? \
                                    TSreadState((tMUXSensor)p.sensorID): \
                                    TSreadState((tSensors)p.sensorID)
#else
    #define TouchGetState(p)    TSreadState((tSensors)p.sensorID)
#endif

//
// Type definitions.
//
typedef struct
{
    int  sensorID;
    int  touchFlags;
    bool fActive;
} TOUCH;

//
// Import function prototypes.
//
void
TouchEvent(
    __in TOUCH &touch,
    __in bool fActive
    );

/**
 *  This function initializes the touch sensor.
 *
 *  @param touch Points to the TOUCH structure to be initialized.
 *  @param sensorID Specifies the ID of the gyro sensor.
 *  @param touchFlags Specifies the touch flags.
 */
void
TouchInit(
    __out TOUCH &touch,
    __in  int sensorID,
    __in  int touchFlags
    )
{
    TFuncName("TouchInit");
    TLevel(INIT);
    TEnter();

    touch.sensorID = sensorID;
    touch.touchFlags = touchFlags & TOUCHF_USER_MASK;
    touch.fActive = false;

    TExit();
    return;
}   //TouchInit

/**
 *  This function performs the touch task where it monitors the touch sensor
 *  state and send a notification if necessary.
 *
 *  @param touch Points to the TOUCH structure.
 */
void
TouchTask(
    __inout TOUCH &touch
    )
{
    bool fActive;

    TFuncName("TouchTask");
    TLevel(TASK);
    TEnter();

    if (touch.touchFlags & TOUCHF_ENABLE_EVENTS)
    {
        fActive = TouchGetState(touch);
        if (fActive != touch.fActive)
        {
            //
            // Touch sensor has changed state.
            //
            TouchEvent(touch, fActive);
            touch.fActive = fActive;
        }
    }

    TExit();
    return;
}   //TouchTask

#endif  //ifndef _TOUCH_H
