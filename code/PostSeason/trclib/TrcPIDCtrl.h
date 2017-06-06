#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDCtrl.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDCtrl class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDCTRL_H
#define _TRCPIDCTRL_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDCTRL
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDCtrl"

//
// PIDCtrl options.
//
#define PIDCTRLO_INVERSE        0x00000001
#define PIDCTRLO_ABS_SETPT      0x00000002

class TrcPIDCtrl;

/**
 * This abstract class defines the PIDInput object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to provide
 * input data to a PID controller.
 */
class PIDInput
{
public:
    /**
     * This function is provided by the subclass to provide input to the PID
     * controller object.
     *
     * @param pidCtrl Points to the PIDCtrl object that requires input.
     */
    virtual
    float
    GetInput(
        __in TrcPIDCtrl *pidCtrl
        ) = 0;
};  //class PIDInput

/**
 * This class defines and implements the TrcPIDCtrl object. This object
 * replaces the PIDController object from the WPI library. The PIDController
 * object in the WPI library is not flexible enough because it dealt with
 * PID input and PID output directly. In contrast, the TrcPIDCtrl object is
 * a simple primitive that calculates the PID output according to the given
 * PID input. So it is agnostic to the kind of input, output or even how the
 * calculated output is used. This is especially important because the
 * TrcPIDDrive object is going to combine three PID controllers to
 * calculate the drive output. One PID controller is for driving straight
 * distance in the X direction, one PID controller is for driving straight
 * distance in the Y direction, and one for rotation. In addition, the WPI
 * PIDController is creating a periodic loop per controller. In the TRC
 * library, we will do periodic loop at a higher level in the TrcPIDDrive
 * object instead.
 */
class TrcPIDCtrl
{
private:

    float     m_Kp;
    float     m_Ki;
    float     m_Kd;
    float     m_tolerance;
    UINT32    m_settlingTime;
    UINT32    m_pidCtrlOptions;
    float     m_minInput;
    float     m_maxInput;
    float     m_minOutput;
    float     m_maxOutput;
    float     m_prevError;
    double    m_totalError;
    UINT32    m_startSettling;
    float     m_setPoint;

public:
    /**
     * Constructor: Create an instance of the TrcPIDCtrl object.
     *
     * @param Kp Specifies the proportional coefficient.
     * @param Ki Specifies the integral coefficient.
     * @param Kd Specifies the derivative coefficient.
     * @param tolerance Specifies the on-target tolerance.
     * @param settlingTime Specifes the on-target settling time in msec.
     * @param pidCtrlOptions Specifies the option flags.
     */
    TrcPIDCtrl(
        __in float  Kp,
        __in float  Ki,
        __in float  Kd,
        __in float  tolerance = 0.0,
        __in UINT32 settlingTime = 0,
        __in UINT32 pidCtrlOptions = 0
        ): m_Kp(Kp),
           m_Ki(Ki),
           m_Kd(Kd),
           m_tolerance(tolerance),
           m_settlingTime(settlingTime),
           m_pidCtrlOptions(pidCtrlOptions)
    {
        TLevel(INIT);
        TEnterMsg(("Kp=%f,Ki=%f,Kd=%f,tolerance=%f,settlingTime=%d,options=%x",
                   Kp, Ki, Kd, tolerance, settlingTime, pidCtrlOptions));

        m_minInput = 0.0;
        m_maxInput = 0.0;
        m_minOutput = -1.0;
        m_maxOutput = 1.0;

        m_prevError = 0.0;
        m_totalError = 0.0;
        m_startSettling = 0;
        m_setPoint = 0.0;

        TExit();
    }   //TrcPIDCtrl

    /**
     * Destructor: Destroy an instance of the TrcPIDCtrl object.
     */
    ~TrcPIDCtrl(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TrcPIDCtrl

    /**
     * This function resets the PID controller.
     */
    void
    Reset(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_prevError = 0.0;
        m_totalError = 0.0;

        TExit();
        return;
    }   //Reset

    /**
     * This function gets the proportional constant.
     *
     * @return Proportional coefficient constant.
     */
    float
    GetKp(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%f", m_Kp));
        return m_Kp;
    }   //GetKp

    /**
     * This function gets the integral constant.
     *
     * @return Integral coefficient constant.
     */
    float
    GetKi(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%f", m_Ki));
        return m_Ki;
    }   //GetKi

    /**
     * This function gets the differential constant.
     *
     * @return Differential coefficient constant.
     */
    float
    GetKd(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%f", m_Kd));
        return m_Kd;
    }   //GetKd

    /**
     * This function gets the PID controller constants.
     *
     * @param pKp Points to the variable to hold the proportional constant.
     * @param pKi Points to the variable to hold the integral constant.
     * @param pKd Points to the variable to hold the differential constant.
     */
    void
    GetPID(
        __in float *pKp = NULL,
        __in float *pKi = NULL,
        __in float *pKd = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("pKp=%p,pKi=%p,pKd=%p", pKp, pKi, pKd));

        if (pKp != NULL)
        {
            *pKp = m_Kp;
        }

        if (pKi != NULL)
        {
            *pKi = m_Ki;
        }

        if (pKd != NULL)
        {
            *pKd = m_Kd;
        }

        TExitMsg(("Kp=%f,Ki=%f,Kd=%f", m_Kp, m_Ki, m_Kd));
        return;
    }   //GetPID

    /**
     * This function sets the PID controller constants.
     *
     * @param Kp Specifies the proportional constant.
     * @param Ki Specifies the integral constant.
     * @param Kd Specifies the differential constant.
     */
    void
    SetPID(
        __in float Kp,
        __in float Ki,
        __in float Kd
        )
    {
        TLevel(API);
        TEnterMsg(("Kp=%f,Ki=%f,Kd=%f", Kp, Ki, Kd));

        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;

        TExit();
        return;
    }   //SetPID

    /**
     * This function gets the last error.
     *
     * @return the last error.
     */
    float
    GetError(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%f", m_prevError));
        return m_prevError;
    }   //GetError

    /**
     * This function gets the PID controller setpoint.
     *
     * @return the current setpoint.
     */
    float
    GetTarget(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%f", m_setPoint));
        return m_setPoint;
    }   //GetTarget

    /**
     * This function sets the PID controller setpoint.
     *
     * @param setPoint Specifies the target setpoint.
     * @param currInput Specifies the current input value.
     */
    void
    SetTarget(
        __in float setPoint,
        __in float currInput
        )
    {
        TLevel(API);
        TEnterMsg(("setpt=%f,currInput=%f", setPoint, currInput));

        if (!(m_pidCtrlOptions & PIDCTRLO_ABS_SETPT))
        {
            setPoint += currInput;
        }

        if (m_maxInput > m_minInput)
        {
            if (setPoint > m_maxInput)
            {
                m_setPoint = m_maxInput;
            }
            else if (setPoint < m_minInput)
            {
                m_setPoint = m_minInput;
            }
            else
            {
                m_setPoint = setPoint;
            }
        }
        else
        {
            m_setPoint = setPoint;
        }
        m_prevError = m_setPoint - currInput;
        m_totalError = 0.0;
        m_startSettling = GetMsecTime();

        TExit();
        return;
    }   //SetTarget

    /**
     * This function determines if we are on target by checking if the
     * previous error is within target tolerance and remain within tolerance
     * for at least the settling period.
     *
     * @return True if we are on target, false otherwise.
     */
    bool
    OnTarget(
        void
        )
    {
        bool fOnTarget = false;

        TLevel(API);
        TEnter();

        if (fabs(m_prevError) > m_tolerance)
        {
            m_startSettling = GetMsecTime();
        }
        else if (GetMsecTime() - m_startSettling >= m_settlingTime)
        {
            fOnTarget = true;
        }

        TExitMsg(("=%x", fOnTarget));
        return fOnTarget;
    }   //OnTarget

    /**
     * This function sets the minimum and maximum values expected from the
     * input.
     *
     * @param minInput Specifies the minimum value.
     * @param maxInput Specifies the maximum value.
     */
    void
    SetInputRange(
        __in float minInput,
        __in float maxInput
        )
    {
        TLevel(API);
        TEnterMsg(("min=%f,max=%f", minInput, maxInput));

        m_minInput = minInput;
        m_maxInput = maxInput;

        TExit();
        return;
    }   //SetInputRange

    /**
     * This function limits the minimum and maximum values of the output.
     *
     * @param minOutput Specifies the minimum value.
     * @param maxOutput Specifies the maximum value.
     */
    void
    SetOutputRange(
        __in float minOutput,
        __in float maxOutput
        )
    {
        TLevel(API);
        TEnterMsg(("min=%f,max=%f", minOutput, maxOutput));

        m_minOutput = minOutput;
        m_maxOutput = maxOutput;

        TExit();
        return;
    }   //SetOutputRange

    /**
     * This function returns the calculated PID output according to the input
     * value from the input source.
     *
     * @param currInput Specifies the current input value.
     *
     * @return Returns the PID control source input value.
     */
    float
    CalcPIDOutput(
        __in float currInput
        )
    {
        float output;
        float error;
        float adjTotalError;

        TLevel(API);
        TEnterMsg(("currInput=%f", currInput));

        error = m_setPoint - currInput;
        if (m_pidCtrlOptions & PIDCTRLO_INVERSE)
        {
            error = -error;
        }

        adjTotalError = m_Ki*(m_totalError + error);
        if ((adjTotalError > m_minOutput) && (adjTotalError < m_maxOutput))
        {
            m_totalError += error;
        }

        output = m_Kp*error + m_Ki*m_totalError + m_Kd*(error - m_prevError);
        m_prevError = error;

        if (output < m_minOutput)
        {
            output = m_minOutput;
        }
        else if (output > m_maxOutput)
        {
            output = m_maxOutput;
        }

        TExitMsg(("=%f", output));
        return output;
    }   //CalcPIDOutput
};  //class TrcPIDCtrl

#endif  //ifndef _TRCPIDCTRL_H
