#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="joybtn.h" />
///
/// <summary>
///     This module contains the library functions for the joystick buttons.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _JOYBTN_H
#define _JOYBTN_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_JOYBTN

//
// Constants.
//
#ifndef NUM_BTNS
  #define NUM_BTNS              12
#endif
#define JOYBTNF_USER_MASK       0x00ff
#define JOYBTNF_ENABLE_EVENTS   0x0001

//
// Macros.
//
#define Btn(n)                  (1 << (n))

#define Logitech_Btn1           Btn(0)
#define Logitech_Btn2           Btn(1)
#define Logitech_Btn3           Btn(2)
#define Logitech_Btn4           Btn(3)
#define Logitech_LB5            Btn(4)
#define Logitech_RB6            Btn(5)
#define Logitech_LB7            Btn(6)
#define Logitech_RB8            Btn(7)
#define Logitech_Btn9           Btn(8)
#define Logitech_Btn10          Btn(9)
#define Logitech_LStick         Btn(10)
#define Logitech_RStick         Btn(11)

#define Xbox_A                  Btn(0)
#define Xbox_B                  Btn(1)
#define Xbox_X                  Btn(2)
#define Xbox_Y                  Btn(3)
#define Xbox_LB                 Btn(4)
#define Xbox_RB                 Btn(5)
#define Xbox_Back               Btn(6)
#define Xbox_Start              Btn(7)
#define Xbox_LStick             Btn(8)
#define Xbox_RStick             Btn(9)

//
// Type definitions.
//
typedef struct
{
    int  joystickID;
    int  buttonFlags;
    int  prevButtons;
    int  buttonMask;
    bool fPressed;
} JOYBTN;

//
// Import function prototypes.
//
void
JoyBtnEvent(
    __in JOYBTN &joybtn
    );

/**
 *  This function initializes the joystick button system.
 *
 *  @param joybtn Points to the JOYBTN structure to be initialized.
 *  @param joystickID Specifies the joystick ID.
 *  @param buttonFlags Specifies the button flags.
 */
void
JoyBtnInit(
    __out JOYBTN &joybtn,
    __in  int joystickID,
    __in  int buttonFlags
    )
{
    TFuncName("JoyBtnInit");
    TLevel(INIT);
    TEnter();

    joybtn.joystickID = joystickID;
    joybtn.buttonFlags = buttonFlags & JOYBTNF_USER_MASK;
    joybtn.prevButtons = 0;
    joybtn.buttonMask = 0;
    joybtn.fPressed = false;

    TExit();
    return;
}   //JoyBtnInit

/**
 *  This function processes the changed buttons and sends button event
 *  notifications.
 *
 *  @param joybtn Points to the JOYBTN structure.
 */
void
JoyBtnTask(
    __inout JOYBTN &joybtn
    )
{
    int currButtons = (joybtn.joystickID == 1)? joystick.joy1_Buttons:
                                                joystick.joy2_Buttons;

    TFuncName("JoyBtnTask");
    TLevel(TASK);
    TEnterMsg(("Prev=%x,Curr=%x", joybtn.prevButtons, currButtons));

    if (joybtn.buttonFlags & JOYBTNF_ENABLE_EVENTS)
    {
        int changedButtons = joybtn.prevButtons^currButtons;

        while (changedButtons != 0)
        {
            //
            // maskButton contains the least significant set bit.
            //
            joybtn.buttonMask = changedButtons &
                                ~(changedButtons^-changedButtons);
            if ((currButtons & joybtn.buttonMask) != 0)
            {
                //
                // Button is pressed.
                //
                joybtn.fPressed = true;
                JoyBtnEvent(joybtn);
            }
            else
            {
                //
                // Button is released.
                //
                joybtn.fPressed = false;
                JoyBtnEvent(joybtn);
            }
            changedButtons &= ~joybtn.buttonMask;
        }
    }
    joybtn.prevButtons = currButtons;

    TExit();
    return;
}   //JoyBtnTask

#endif  //ifndef _JOYBTN_H
