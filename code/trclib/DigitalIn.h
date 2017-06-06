#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DigitalIn.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DigitalIn class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DIGITALIN_H
#define _DIGITALIN_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DIGITALIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DigitalIn"

//
// Macros.
//
#define MAX_DIGITAL_CHANNEL     14
#define DInMask(n)              (1 << (16 - (n)))

/**
 * This abstract class defines the DigitalInNotify object. The object
 * is a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the digital input events.
 */
class DigitalInNotify
{
public:
    /**
     * This function is provided by the subclass to handle a DigitalIn event
     * notification.
     *
     * @param slot Specifies the Digital Module slot.
     * @param channel Specifies the DigitalIn channel that changed state.
     * @param fActive If true, specifies the DigitalIn channel is active,
     *        false otherwise.
     */
    virtual
    void
    NotifyDIn(
        __in UINT32 slot,
        __in UINT32 channel,
        __in bool   fActive
        ) = 0;
};  //class DigitalInNotify

/**
 * This class defines and implements the DigitalIn object. It inherited the
 * DigitalModule object from the WPI library. It also added the capability of
 * detecting the changes of a digital input channel and call the notification
 * object for the change.
 */
class DigitalIn: public DigitalModule
{
private:
    UINT32           m_slot;
    UINT32           m_notifyMask;
    DigitalInNotify *m_notify;
    UINT16           m_prevDIn;

public:
    /**
     * Constructor: Create an instance of the DigitalIn object.
     * 
     * @param notifyMask Specifies the channel mask that will generate
     *        notifications.
     * @param notify Points to the DigitalInNotify object for DigitalIn
     *        event notification callback.
     */
    DigitalIn(
        __in UINT32 notifyMask,
        __in DigitalInNotify *notify = NULL
        ): DigitalModule(GetDefaultDigitalModule()),
           m_slot(GetDefaultDigitalModule()),
           m_notifyMask(notifyMask),
           m_notify(notify)
    {
        TLevel(INIT);
        TEnterMsg(("Slot=%d,mask=%x,notify=%p", m_slot, notifyMask, notify));

        m_prevDIn = GetDIO();

        TExit();
    }   //DigitalIn

    /**
     * Constructor: Create an instance of the DigitalIn object.
     * 
     * @param slot Specifies the slot for the Digital Module.
     * @param notifyMask Specifies the channel mask that will generate
     *        notifications.
     * @param notify Points to the DigitalInNotify object for DigitalIn
     *        event notification callback.
     */
    DigitalIn(
        __in UINT32           slot,
        __in UINT32           notifyMask,
        __in DigitalInNotify *notify = NULL
        ): DigitalModule(slot),
           m_slot(slot),
           m_notifyMask(notifyMask),
           m_notify(notify)
    {
        TLevel(INIT);
        TEnterMsg(("Slot=%d,mask=%x,notify=%p", slot, notifyMask, notify));

        m_prevDIn = GetDIO();

        TExit();
    }   //DigitalIn

    /**
     * Destructor: Destroy an instance of the DigitalIn object.
     */
    ~DigitalIn(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~DigitalIn

    /**
     * This function returns the state of the a digital input channel.
     *
     * @param channel Specifies the digital channel to get its state.
     *
     * @return Returns the state of the digital input channel.
     */
    UINT32
    GetChannelState(
        __in UINT32 channel
        )
    {
        UINT32 state;

        TLevel(API);
        TEnterMsg(("channel=%d", channel));

        state = GetDIO(channel);

        TExitMsg(("=%x", state));
        return state;
    }   //GetChannelState

    /**
     * This function is called by the robot loop to check and process
     * DigitalIn events. It reads all digital input channels and compares
     * them to their previous states. If any channel has changed state,
     * the notification object is called.
     */
    void
    DigitalInTask(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        UINT16 currDIn = GetDIO();
        if (m_notify != NULL)
        {
            UINT16 changedDIn = (m_prevDIn^currDIn) & m_notifyMask;
            UINT16 maskDIn;
            UINT32 channel;

            while (changedDIn != 0)
            {
                //
                // maskButton contains the least significant set bit.
                //
                maskDIn = changedDIn & ~(changedDIn^-changedDIn);
                for (channel = 1; channel <= MAX_DIGITAL_CHANNEL; channel++)
                {
                    if (maskDIn == DInMask(channel))
                    {
                        break;
                    }
                }

                if ((currDIn & maskDIn) != 0)
                {
                    //
                    // DigitalIn is active.
                    //
                    m_notify->NotifyDIn(m_slot, channel, true);
                }
                else
                {
                    //
                    // DigitalIn is inactive.
                    //
                    m_notify->NotifyDIn(m_slot, channel, false);
                }
                //
                // Clear the least significant set bit.
                //
                changedDIn &= ~maskDIn;
            }
        }
        m_prevDIn = currDIn;

        TExit();
    }   //DigitalInTask
};  //class DigitalIn

#endif  //ifndef _DIGITALIN_H
