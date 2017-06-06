#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcJoystick.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcJoystick class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCJOYSTICK_H
#define _TRCJOYSTICK_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_JOYSTICK
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcJoystick"

//
// Joystick input macros.
//
#ifndef DEADBAND_INPUT_THRESHOLD
  #define DEADBAND_INPUT_THRESHOLD  0.10
#endif
#define DEADBAND_INPUT(n)       DEADBAND(n, DEADBAND_INPUT_THRESHOLD)
//
// Macros.
//
#define Btn(n)                  (1 << (n))
//
// Logitech Joystick:
// UsagePage=0x01, Usage=0x04
//
#define Logitech_Trigger        Btn(0)
#define Logitech_Btn2           Btn(1)
#define Logitech_Btn3           Btn(2)
#define Logitech_Btn4           Btn(3)
#define Logitech_Btn5           Btn(4)
#define Logitech_Btn6           Btn(5)
#define Logitech_Btn7           Btn(6)
#define Logitech_Btn8           Btn(7)
#define Logitech_Btn9           Btn(8)
#define Logitech_Btn10          Btn(9)
#define Logitech_Btn11          Btn(10)
#define Logitech_Btn12          Btn(11)
//
// Microsoft SideWinder Joystick:
// UsagePage=0x01, Usage=0x04
//
#define SideWinder_Trigger      Btn(0)
#define SideWinder_Btn2         Btn(1)
#define SideWinder_Btn3         Btn(2)
#define SideWinder_Btn4         Btn(3)
#define SideWinder_BtnA         Btn(4)
#define SideWinder_BtnB         Btn(5)
#define SideWinder_BtnC         Btn(6)
#define SideWinder_BtnD         Btn(7)
#define SideWinder_Btn9         Btn(8)
//
// Microsoft Xbox Game Controller:
// UsagePage=0x01, Usage=0x05
//
#define Xbox_BtnA               Btn(0)
#define Xbox_BtnB               Btn(1)
#define Xbox_BtnX               Btn(2)
#define Xbox_BtnY               Btn(3)
#define Xbox_LB                 Btn(4)
#define Xbox_RB                 Btn(5)
#define Xbox_Back               Btn(6)
#define Xbox_Start              Btn(7)

/**
 * This abstract class defines the ButtonNotify object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the joystick button events.
 */
class ButtonNotify
{
public:
    /**
     * This function is provided by the subclass to handle a joystick
     * button event notification.
     *
     * @param port Specifies the joystick port.
     * @param btnMask Specifies the joystick button bit mask.
     * @param fPressed If true, specifies the button is pressed,
     *        false otherwise.
     */
    virtual
    void
    NotifyButton(
        __in UINT32 port,
        __in UINT16 btnMask,
        __in bool   fPressed
        ) = 0;
};  //class ButtonNotify

/**
 * This class defines and implements the TrcJoystick object. This object
 * inherits the Joystick object from the WPI library. It added the support
 * of detecting joystick button events and calling the notification object.
 * It also added the deadband support for reading the analog joystick axes.
 */
class TrcJoystick: public Joystick
{
private:
    UINT32          m_port;
    ButtonNotify   *m_notify;
    DriverStation  *m_ds;
    UINT16          m_prevBtn;

public:
    /**
     * Constructor: Create an instance of the TrcJoystick object.
     * 
     * @param port Specifies the joystick port.
     * @param notify Points to the ButtonNotify object for button event
     *        notification callback.
     */
    TrcJoystick(
        __in UINT32        port,
        __in ButtonNotify *notify = NULL
        ): Joystick(port),
           m_port(port),
           m_notify(notify)
    {
        TLevel(INIT);
        TEnterMsg(("Joystick=%d,notify=%p", port, notify));

        m_ds = DriverStation::GetInstance();
        m_prevBtn = m_ds->GetStickButtons(port);

        TExit();
    }   //TrcJoystick

    /**
     * Destructor: Destroy an instance of the TrcJoystick object.
     */
    ~TrcJoystick(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TrcJoystick

    /**
     * This function gets the X value of the joystick with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     * @param hand Specifies the handedness of the joystick (default to right
     *        hand).
     */
    float
    GetXWithDeadband(
        __in float        threshold = DEADBAND_INPUT_THRESHOLD,
        __in JoystickHand hand = kRightHand
        )
    {
        float value;

        TLevel(HIFREQ);
        TEnterMsg(("threshold=%f,hand=%d", threshold, hand));

        value = DEADBAND(GetX(hand), threshold);

        TExitMsg(("=%f", value));
        return value;
    }   //GetXWithDeadband

    /**
     * This function gets the Y value of the joystick with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     * @param hand Specifies the handedness of the joystick (default to right
     *        hand).
     */
    float
    GetYWithDeadband(
        __in float        threshold = DEADBAND_INPUT_THRESHOLD,
        __in JoystickHand hand = kRightHand
        )
    {
        float value;

        TLevel(HIFREQ);
        TEnterMsg(("threshold=%f,hand=%d", threshold, hand));

        value = DEADBAND(GetY(hand), threshold);

        TExitMsg(("=%f", value));
        return value;
    }   //GetYWithDeadband

    /**
     * This function gets the Z value of the joystick with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     */
    float
    GetZWithDeadband(
        __in float threshold = DEADBAND_INPUT_THRESHOLD
        )
    {
        float value;

        TLevel(HIFREQ);
        TEnterMsg(("threshold=%f", threshold));

        value = DEADBAND(GetZ(), threshold);

        TExitMsg(("=%f", value));
        return value;
    }   //GetZWithDeadband

    /**
     * This function gets the twist value of the joystick with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     */
    float
    GetTwistWithDeadband(
        __in float threshold = DEADBAND_INPUT_THRESHOLD
        )
    {
        float value;

        TLevel(API);
        TEnterMsg(("threshold=%f", threshold));

        value = DEADBAND(GetTwist(), threshold);

        TExitMsg(("=%f", value));
        return value;
    }   //GetTwistWithDeadband

    /**
     * This function gets the magnitude value of the joystick vector between
     * the X and Y axes with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     */
    float
    GetMagnitudeWithDeadband(
        __in float threshold = DEADBAND_INPUT_THRESHOLD
        )
    {
        float value;

        TLevel(API);
        TEnterMsg(("threshold=%f", threshold));

        value = MAGNITUDE(GetXWithDeadband(threshold),
                          GetYWithDeadband(threshold));

        TExitMsg(("=%f", value));
        return value;
    }   //GetMagnitudeWithDeadband

    /**
     * This function gets the direction value in radians of the joystick
     * vector between the X and Y axes with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     */
    float
    GetRadiansWithDeadband(
        __in float threshold = DEADBAND_INPUT_THRESHOLD
        )
    {
        float value;

        TLevel(API);
        TEnterMsg(("threshold=%f", threshold));

        value = DIR_RADIANS(GetXWithDeadband(threshold),
                            -GetYWithDeadband(threshold));

        TExitMsg(("=%f", value));
        return value;
    }   //GetRadiansWithDeadband

    /**
     * This function gets the direction value in degrees of the joystick
     * vector with deadband.
     *
     * @param threshold Specifies the deadband threshold.
     */
    float
    GetDegreesWithDeadband(
        __in float threshold = DEADBAND_INPUT_THRESHOLD
        )
    {
        float value;

        TLevel(API);
        TEnterMsg(("threshold=%f", threshold));

        value = RADIANS_TO_DEGREES(GetRadiansWithDeadband(threshold));

        TExitMsg(("=%f", value));
        return value;
    }   //GetDegreesWithDeadband

    /**
     * This function is called periodically by the robot loop to check and
     * process joystick button events.
     */
    void
    ButtonTask(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        UINT16 currBtn = m_ds->GetStickButtons(m_port);
        if (m_notify != NULL)
        {
            UINT16 changedBtn = m_prevBtn^currBtn;
            UINT16 btnMask;
            while (changedBtn != 0)
            {
                //
                // maskButton contains the least significant set bit.
                //
                btnMask = changedBtn & ~(changedBtn^-changedBtn);
                if ((currBtn & btnMask) != 0)
                {
                    //
                    // Button is pressed.
                    //
                    m_notify->NotifyButton(m_port, btnMask, true);
                }
                else
                {
                    //
                    // Button is released.
                    //
                    m_notify->NotifyButton(m_port, btnMask, false);
                }
                //
                // Clear the least significant set bit.
                //
                changedBtn &= ~btnMask;
            }
        }
        m_prevBtn = currBtn;

        TExit();
    }   //ButtonTask
};  //class TrcJoystick

#endif  //ifndef _TRCJOYSTICK_H
