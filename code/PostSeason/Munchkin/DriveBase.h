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
#ifdef _USE_LINE_FOLLOWER
                 public PIDInput,
                 public SensorArray
#else
                 public PIDInput
#endif
{
private:
#if defined(_USE_WHEEL_ENCODERS)
    Encoder         *m_frontLeftEncoder;
    Encoder         *m_rearLeftEncoder;
    Encoder         *m_frontRightEncoder;
    Encoder         *m_rearRightEncoder;
#elif defined(_USE_TRACKING_WHEELS)
    Encoder         *m_xTrackingEncoder;
    Encoder         *m_yTrackingEncoder;
#elif defined(_USE_ACCELEROMETER)
    TrcAccel        *m_accel;
#endif
    Gyro            *m_gyro;
#ifdef _USE_LINE_FOLLOWER
    DigitalInput    *m_leftLightSensor;
    DigitalInput    *m_centerLightSensor;
    DigitalInput    *m_rightLightSensor;
#endif
    TrcPIDCtrl      *m_pidCtrlXDrive;
    TrcPIDCtrl      *m_pidCtrlYDrive;
    TrcPIDCtrl      *m_pidCtrlTurn;
#ifdef _USE_LINE_FOLLOWER
    TrcPIDCtrl      *m_pidCtrlLightSensor;
#endif
    TrcPIDDrive     *m_pidDrive;
#ifdef _USE_LINE_FOLLOWER
    LineFollower    *m_lineFollower;
#endif

public:
    /**
     * Constructor for the BaseDrive class.
     *
     * @param frontLeftMotor Specifies the speed controller for the front
     *        left motor.
     * @param rearLeftMotor Specifies the speed controller for the rear
     *        left motor.
     * @param frontRightMotor Specifies the speed controller for the front
     *        right motor.
     * @param rearRightMotor Specifies the speed controller for the rear
     *        right motor.
     */
    DriveBase(
        __in SpeedController    *frontLeftMotor,
        __in SpeedController    *rearLeftMotor,
        __in SpeedController    *frontRightMotor,
        __in SpeedController    *rearRightMotor
        ): RobotDrive(frontLeftMotor, rearLeftMotor,
                      frontRightMotor, rearRightMotor)
    {
        TLevel(INIT);
        TEnterMsg(("frontLeft=%p,rearLeft=%p,frontRight=%p,rearRight=%p",
                   frontLeftMotor, rearLeftMotor, frontRightMotor,
                   rearRightMotor));

#if defined(_USE_WHEEL_ENCODERS)
        m_frontLeftEncoder = new Encoder(DIN_FRONT_LEFT_ENCODER_A,
                                         DIN_FRONT_LEFT_ENCODER_B);
        m_rearLeftEncoder = new Encoder(DIN_REAR_LEFT_ENCODER_A,
                                        DIN_REAR_LEFT_ENCODER_B);
        m_frontRightEncoder = new Encoder(DIN_FRONT_RIGHT_ENCODER_A,
                                          DIN_FRONT_RIGHT_ENCODER_B);
        m_rearRightEncoder = new Encoder(DIN_REAR_RIGHT_ENCODER_A,
                                         DIN_REAR_RIGHT_ENCODER_B);
        m_frontLeftEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
        m_rearLeftEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
        m_frontRightEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
        m_rearRightEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
#elif defined(_USE_TRACKING_WHEELS)
        m_xTrackingEncoder = new Encoder(DIN_X_TRACKING_ENCODER_A,
                                         DIN_X_TRACKING_ENCODER_B);
        m_yTrackingEncoder = new Encoder(DIN_Y_TRACKING_ENCODER_A,
                                         DIN_Y_TRACKING_ENCODER_B);
        m_xTrackingEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
        m_yTrackingEncoder->SetDistancePerPulse(DISTANCE_PER_CLICK);
#elif defined(_USE_ACCELEROMETER)
        m_accel = new TrcAccel(SensorBase::GetDefaultDigitalModule());
#endif

        m_gyro = new Gyro(AIN_GYRO);

#ifdef _USE_LINE_FOLLOWER
        m_leftLightSensor = new DigitalInput(DIN_LEFT_LIGHTSENSOR);
        m_centerLightSensor = new DigitalInput(DIN_CENTER_LIGHTSENSOR);
        m_rightLightSensor = new DigitalInput(DIN_RIGHT_LIGHTSENSOR);
#endif

        m_pidCtrlXDrive = new TrcPIDCtrl(DRIVE_KP, DRIVE_KI, DRIVE_KD,
                                         DRIVE_TOLERANCE, DRIVE_SETTLING);
        m_pidCtrlYDrive = new TrcPIDCtrl(DRIVE_KP, DRIVE_KI, DRIVE_KD,
                                         DRIVE_TOLERANCE, DRIVE_SETTLING);
        m_pidCtrlTurn = new TrcPIDCtrl(TURN_KP, TURN_KI, TURN_KD,
                                       TURN_TOLERANCE, TURN_SETTLING);

#ifdef _USE_LINE_FOLLOWER
        m_pidCtrlLightSensor = new TrcPIDCtrl(LNFOLLOW_KP,
                                              LNFOLLOW_KI,
                                              LNFOLLOW_KD,
                                              LNFOLLOW_TOLERANCE,
                                              LNFOLLOW_SETTLING,
                                              PIDCTRLO_ABS_SETPT);
#endif
        m_pidDrive = new TrcPIDDrive(this,
                                     m_pidCtrlXDrive,
                                     m_pidCtrlYDrive,
                                     m_pidCtrlTurn,
                                     this,
                                     PIDDRIVEO_MECANUM_DRIVE);
#ifdef _USE_LINE_FOLLOWER
        m_lineFollower = new LineFollower(this,
                                          m_pidCtrlLightSensor,
                                          this,
                                          LNFOLLOWO_MECANUM_DRIVE);
#endif
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

#ifdef _USE_LINE_FOLLOWER
        SAFE_DELETE(m_lineFollower);
#endif
        SAFE_DELETE(m_pidDrive);
#ifdef _USE_LINE_FOLLOWER
        SAFE_DELETE(m_pidCtrlLightSensor);
#endif
        SAFE_DELETE(m_pidCtrlTurn);
        SAFE_DELETE(m_pidCtrlYDrive);
        SAFE_DELETE(m_pidCtrlXDrive);
#ifdef _USE_LINE_FOLLOWER
        SAFE_DELETE(m_rightLightSensor);
        SAFE_DELETE(m_centerLightSensor);
        SAFE_DELETE(m_leftLightSensor);
#endif
        SAFE_DELETE(m_gyro);
#if defined(_USE_WHEEL_ENCODERS)
        SAFE_DELETE(m_rearRightEncoder);
        SAFE_DELETE(m_frontRightEncoder);
        SAFE_DELETE(m_rearLeftEncoder);
        SAFE_DELETE(m_frontLeftEncoder);
#elif defined(_USE_TRACKING_WHEELS)
        SAFE_DELETE(m_yTrackingEncoder);
        SAFE_DELETE(m_xTrackingEncoder);
#elif defined(_USE_ACCELEROMETER)
        SAFE_DELETE(m_accel);
#endif

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
        SetInvertedMotor(RobotDrive::kFrontLeftMotor,
                         MOTOR_FRONT_LEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearLeftMotor,
                         MOTOR_REAR_LEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kFrontRightMotor,
                         MOTOR_FRONT_RIGHT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor,
                         MOTOR_REAR_RIGHT_REVERSE);
        m_pidCtrlXDrive->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlYDrive->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlTurn->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
#ifdef _USE_LINE_FOLLOWER
        m_pidCtrlLightSensor->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
#endif

#if defined(_USE_WHEEL_ENCODERS)
        m_frontLeftEncoder->Reset();
        m_frontLeftEncoder->Start();
        m_rearLeftEncoder->Reset();
        m_rearLeftEncoder->Start();
        m_frontRightEncoder->Reset();
        m_frontRightEncoder->Start();
        m_rearRightEncoder->Reset();
        m_rearRightEncoder->Start();
#elif defined(_USE_TRACKING_WHEELS)
        m_xTrackingEncoder->Reset();
        m_xTrackingEncoder->Start();
        m_yTrackingEncoder->Reset();
        m_yTrackingEncoder->Start();
#endif

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

#ifdef _USE_LINE_FOLLOWER
        m_lineFollower->Stop();
#endif
        m_pidDrive->Stop();
#ifdef _USE_ACCELEROMETER
        m_accel->SetEnabled(false);
#endif

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

#ifdef _USE_LINE_FOLLOWER
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
#endif

#ifdef _USE_ACCELEROMETER
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
#endif

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
#ifdef _USE_LINE_FOLLOWER
        static float prevInput = 0.0;
        float input = prevInput;
#else
        float input = 0.0;
#endif

        TLevel(CALLBK);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        if (pidCtrl == m_pidCtrlXDrive)
        {
#if defined(_USE_WHEEL_ENCODERS)
            input = (float)(m_frontLeftEncoder->GetDistance() +
                            m_rearRightEncoder->GetDistance() -
                            m_frontRightEncoder->GetDistance() -
                            m_rearLeftEncoder->GetDistance())/sqrt(2.0)/2.0;
#elif defined(_USE_TRACKING_WHEELS)
            input = (float)m_xTrackingEncoder->GetDistance();
#elif defined(_USE_ACCELEROMETER)
            input = m_accel->GetDistX();
#endif
        }
        else if (pidCtrl == m_pidCtrlYDrive)
        {
#if defined(_USE_WHEEL_ENCODERS)
            input = (float)(m_frontLeftEncoder->GetDistance() +
                            m_rearRightEncoder->GetDistance() -
                            m_frontRightEncoder->GetDistance() -
                            m_rearLeftEncoder->GetDistance())/sqrt(2.0)/2.0;
#elif defined(_USE_TRACKING_WHEELS)
            input = (float)m_yTrackingEncoder->GetDistance();
#elif defined(_USE_ACCELEROMETER)
            input = m_accel->GetDistY();
#endif
        }
        else if (pidCtrl == m_pidCtrlTurn)
        {
            input = m_gyro->GetAngle();
        }
#ifdef _USE_LINE_FOLLOWER
        else if (pidCtrl == m_pidCtrlLightSensor)
        {
            input = GetMappedValue(GetRawValue());
            if (input == 10.0)
            {
                input = prevInput; 
            }
            prevInput = input;
        }
#endif

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput
};  //class BaseDrive

#endif  //ifndef _DRIVEBASE_H
