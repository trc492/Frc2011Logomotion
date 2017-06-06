#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DriveBase.h" />
///
/// <summary>
///     This module contains the definitions of the DriveBase class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DRIVEBASE_H
#define _DRIVEBASE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVEBASE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DriveBase"

/**
 * This class defines and implements the DriveBase object. This object
 * inherits the RobotDrive object. It consists of a PIDDrive object and a
 * LineFollower object. It also inherits PIDInput to provide sensor readings
 * to the PID controllers. It inherits the LightSensorArray to provide the
 * light sensor mapping for line following PID control.
 */
class DriveBase: public CoopTask, 
                 public RobotDrive,
                 public PIDInput,
                 public SensorArray
{
private:
    TrcAccel        *m_accel;
    Gyro            *m_gyro;
    DigitalInput    *m_leftLightSensor;
    DigitalInput    *m_centerLightSensor;
    DigitalInput    *m_rightLightSensor;
    TrcPIDCtrl      *m_pidCtrlXAccel;
    TrcPIDCtrl      *m_pidCtrlYAccel;
    TrcPIDCtrl      *m_pidCtrlGyro;
    TrcPIDCtrl      *m_pidCtrlLightSensor;
    TrcPIDDrive     *m_pidDrive;
    LineFollower    *m_lineFollower;

public:
    /**
     * Constructor for the BaseDrive class.
     *
     * @param leftFrontMotor Specifies the speed controller for the left
     *        front motor.
     * @param leftRearMotor Specifies the speed controller for the left
     *        rear motor.
     * @param rightFrontMotor Specifies the speed controller for the right
     *        front motor.
     * @param rightRearMotor Specifies the speed controller for the right
     *        rear motor.
     */
    DriveBase(
        __in SpeedController    *leftFrontMotor,
        __in SpeedController    *leftRearMotor,
        __in SpeedController    *rightFrontMotor,
        __in SpeedController    *rightRearMotor
        ): RobotDrive(leftFrontMotor, leftRearMotor,
                      rightFrontMotor, rightRearMotor)
    {
        TLevel(INIT);
        TEnterMsg(("leftFront=%p,leftRear=%p,rightFront=%p,rightRear=%p",
                   leftFrontMotor, leftRearMotor, rightFrontMotor,
                   rightRearMotor));

        m_accel = new TrcAccel(SensorBase::GetDefaultDigitalModule());
        m_gyro = new Gyro(AIN_GYRO);
        m_leftLightSensor = new DigitalInput(DIN_LEFT_LIGHTSENSOR);
        m_centerLightSensor = new DigitalInput(DIN_CENTER_LIGHTSENSOR);
        m_rightLightSensor = new DigitalInput(DIN_RIGHT_LIGHTSENSOR);
        m_pidCtrlXAccel = new TrcPIDCtrl(ACCEL_KP, ACCEL_KI, ACCEL_KD,
                                         ACCEL_DRIVE_TOLERANCE,
                                         ACCEL_DRIVE_SETTLING);
        m_pidCtrlYAccel = new TrcPIDCtrl(ACCEL_KP, ACCEL_KI, ACCEL_KD,
                                         ACCEL_DRIVE_TOLERANCE,
                                         ACCEL_DRIVE_SETTLING);
        m_pidCtrlGyro = new TrcPIDCtrl(GYRO_KP, GYRO_KI, GYRO_KD,
                                       GYRO_TURN_TOLERANCE,
                                       GYRO_TURN_SETTLING);
        m_pidCtrlLightSensor = new TrcPIDCtrl(LNFOLLOW_KP,
                                              LNFOLLOW_KI,
                                              LNFOLLOW_KD,
                                              LNFOLLOW_TOLERANCE,
                                              LNFOLLOW_SETTLING,
                                              PIDCTRLO_ABS_SETPT);
        m_pidDrive = new TrcPIDDrive(this,
                                     m_pidCtrlXAccel,
                                     m_pidCtrlYAccel,
                                     m_pidCtrlGyro,
                                     this,
                                     PIDDRIVEO_MECANUM_DRIVE);
        m_lineFollower = new LineFollower(this,
                                          m_pidCtrlLightSensor,
                                          this,
                                          LNFOLLOWO_MECANUM_DRIVE);
        RegisterTask(TASK_INIT);

        TExit();
    }   //DriveBase

    /**
     * Destructor for the BaseDrive class.
     */
    virtual
    ~DriveBase(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SAFE_DELETE(m_lineFollower);
        SAFE_DELETE(m_pidDrive);
        SAFE_DELETE(m_pidCtrlLightSensor);
        SAFE_DELETE(m_pidCtrlGyro);
        SAFE_DELETE(m_pidCtrlYAccel);
        SAFE_DELETE(m_pidCtrlXAccel);
        SAFE_DELETE(m_rightLightSensor);
        SAFE_DELETE(m_centerLightSensor);
        SAFE_DELETE(m_leftLightSensor);
        SAFE_DELETE(m_gyro);
        SAFE_DELETE(m_accel);

        TExit();
    }   //~DriveBase

    /**
     * This function initializes the DriveBase subsystem.
     */
    void
    InitTask(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        //
        // Initialize RobotDrive.
        //
        SetInvertedMotor(RobotDrive::kFrontLeftMotor, MOTOR_FRONTLEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kFrontRightMotor,MOTOR_FRONTRIGHT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearLeftMotor,MOTOR_REARLEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor, MOTOR_REARRIGHT_REVERSE);
        m_pidCtrlXAccel->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlYAccel->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlGyro->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlLightSensor->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);

        TExit();
    }   //InitTask

    /**
     * This function stops the DriveBase subsystem.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_lineFollower->Stop();
        m_pidDrive->Stop();
        m_accel->SetEnabled(false);

        TExit();
    }   //Stop

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distXSetPoint Specifies the lateral target distance relative
     *        to current position.
     * @param distYSetPoint Specifies the forward target distance relative
     *        to current position.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    DriveSetTarget(
        __in float  distXSetPoint,
        __in float  distYSetPoint,
        __in float  angleSetPoint,
        __in bool   fStopOnTarget = true,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distXSetPt=%f,distYSetPt=%f,angleSetPt=%f,"
                   "fStopOnTarget=%x,event=%p,timeout=%d",
                   distXSetPoint, distYSetPoint, angleSetPoint, fStopOnTarget,
                   notifyEvent, timeout));

        m_pidDrive->SetTarget(distXSetPoint,
                              distYSetPoint,
                              angleSetPoint,
                              fStopOnTarget,
                              notifyEvent,
                              timeout);

        TExit();
        return;
    }   //DriveSetTarget

    /**
     * This function tells the line follower to follow the line.
     *
     * @param maxDrivePower Specifies the maximum driving power when
     *        following line.
     * @param flags Specifies the stopping condition. If 0, it will not
     *        stop.
     */
    void
    DriveFollowLine(
        __in float  lineCenterValue,
        __in UINT32 stopSensorValue,
        __in float  maxDrivePower,
        __in float  findLineDrivePower,
        __in float  findLineTurnPower,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("lineCenter=%f,stopValue=%x,maxDrive=%f,findDrive=%f,findTurn=%f,timeout=%d,notifyEvent=%p",
                   lineCenterValue, stopSensorValue, maxDrivePower,
                   findLineDrivePower, findLineTurnPower, timeout,
                   notifyEvent));

        m_lineFollower->LineFollowStart(lineCenterValue,
                                        stopSensorValue,
                                        maxDrivePower,
                                        findLineDrivePower,
                                        findLineTurnPower,
                                        notifyEvent,
                                        timeout);

        TExit();
    }   //DriveFollowLine

    /**
     * This function enables or disables the accelerometer integrator.
     *
     * @param fEnable If true, enables the accelerometer integrator.
     */
    void
    EnableAccel(
        __in bool fEnable
        )
    {
        TLevel(API);
        TEnterMsg(("fEnable=%d", fEnable));

        m_accel->SetEnabled(fEnable);

        TExit();
        return;
    }   //EnableAccel

    /**
     * This function is called from the LineFollower to get the raw light
     * sensors value. It reads the three light sensors and combine them into
     * a raw bit value.
     *
     * @return Returns the combined bit value of the light sensors.
     */
    UINT32
    GetRawValue(
        void
        )
    {
        UINT32 value = 0;

        TLevel(HIFREQ);
        TEnter();

        value = m_leftLightSensor->Get() << 2;
        value |= m_centerLightSensor->Get() << 1;
        value |= m_rightLightSensor->Get();

        TExitMsg(("=%x", value));
        return value;
    }   //GetRawValue

    /**
     * This function is called to map the raw light sensor value to a value
     * can be used by the PID controller.
     * 
     * @param rawValue Specifies the raw value to lookup.
     *
     * @return Returns the mapped value.
     */
    float
    GetMappedValue(
        __in UINT32 rawValue
        )
    {
        float value;
        static float inputMap[8] =
        {
            10.0,   //000: n/a (No light)
            -2.0,   //001: Extreme right
            0.0,    //010: Center
            -1.0,   //011: Slight right
            2.0,    //100: Extreme left
            10.0,   //101: n/a (At Y)
            1.0,    //110: Slight left
            10.0    //111: n/a (At T)
        };

        TLevel(HIFREQ);
        TEnterMsg(("rawValue=%x", rawValue));

        value = inputMap[rawValue];

        TExitMsg(("=%f", value));
        return value;
    }   //GetMappedValue

    /**
     * This function is called from the PID controllers to get the PID input
     * value.
     *
     * @param pidCtrl Specifies the PID controller that needs to read its input
     *        sensor.
     *
     * @return Read and return the sensor value corresponding to the PID
     *         controller.
     */
    float
    GetInput(
        __in TrcPIDCtrl *pidCtrl
        )
    {
        static float prevInput = 0.0;
        float input = prevInput;

        TLevel(CALLBK);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        if (pidCtrl == m_pidCtrlXAccel)
        {
            input = m_accel->GetDistX();
        }
        else if (pidCtrl == m_pidCtrlYAccel)
        {
            input = m_accel->GetDistY();
        }
        else if (pidCtrl == m_pidCtrlGyro)
        {
            input = m_gyro->GetAngle();
        }
        else if (pidCtrl == m_pidCtrlLightSensor)
        {
            input = GetMappedValue(GetRawValue());
            if (input == 10.0)
            {
                input = prevInput; 
            }
            prevInput = input;
        }

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput
};  //class BaseDrive

#endif  //ifndef _DRIVEBASE_H
