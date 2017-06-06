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
                 public PIDInput
{
private:
    DriverStationLCD    *m_dsLCD;
    AxisCamera          &m_camera;
    TrcAccel            *m_accel;
    Gyro                *m_gyro;
    TrcPIDCtrl          *m_pidCtrlCamera;
    TrcPIDCtrl          *m_pidCtrlXAccel;
    TrcPIDCtrl          *m_pidCtrlYAccel;
    TrcPIDCtrl          *m_pidCtrlGyro;
    TrcPIDDrive         *m_pidDrive;
    TrcPIDDrive         *m_pidVisionDrive;

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
        __in SpeedController *leftFrontMotor,
        __in SpeedController *leftRearMotor,
        __in SpeedController *rightFrontMotor,
        __in SpeedController *rightRearMotor
        ): RobotDrive(leftFrontMotor, leftRearMotor,
                      rightFrontMotor, rightRearMotor),
           m_camera(AxisCamera::GetInstance())
    {
        TLevel(INIT);
        TEnterMsg(("leftFront=%p,leftRear=%p,rightFront=%p,rightRear=%p",
                   leftFrontMotor, leftRearMotor, rightFrontMotor,
                   rightRearMotor));

        m_dsLCD = DriverStationLCD::GetInstance();
        m_accel = new TrcAccel(SensorBase::GetDefaultDigitalModule());
        m_gyro = new Gyro(AIN_GYRO_DRIVE);
        m_pidCtrlCamera = new TrcPIDCtrl(CAMERA_KP, CAMERA_KI, CAMERA_KD,
                                         CAMERA_TURN_TOLERANCE,
                                         CAMERA_TURN_SETTLING,
                                         (PIDCTRLO_ABS_SETPT |
                                          PIDCTRLO_INVERSE));
        m_pidCtrlXAccel = new TrcPIDCtrl(ACCEL_KP, ACCEL_KI, ACCEL_KD,
                                         ACCEL_DRIVE_TOLERANCE,
                                         ACCEL_DRIVE_SETTLING);
        m_pidCtrlYAccel = new TrcPIDCtrl(ACCEL_KP, ACCEL_KI, ACCEL_KD,
                                         ACCEL_DRIVE_TOLERANCE,
                                         ACCEL_DRIVE_SETTLING);
        m_pidCtrlGyro = new TrcPIDCtrl(GYRO_KP, GYRO_KI, GYRO_KD,
                                       GYRO_TURN_TOLERANCE,
                                       GYRO_TURN_SETTLING);
        m_pidDrive = new TrcPIDDrive(this,
                                     m_pidCtrlXAccel,
                                     m_pidCtrlYAccel,
                                     m_pidCtrlGyro,
                                     this,
                                     PIDDRIVEO_MECANUM_DRIVE);
        m_pidVisionDrive = new TrcPIDDrive(this,
                                           m_pidCtrlXAccel,
                                           m_pidCtrlYAccel,
                                           m_pidCtrlCamera,
                                           this,
                                           PIDDRIVEO_MECANUM_DRIVE);
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

        SAFE_DELETE(m_pidVisionDrive);
        SAFE_DELETE(m_pidDrive);
        SAFE_DELETE(m_pidCtrlGyro);
        SAFE_DELETE(m_pidCtrlYAccel);
        SAFE_DELETE(m_pidCtrlXAccel);
        SAFE_DELETE(m_pidCtrlCamera);
        SAFE_DELETE(m_gyro);
        SAFE_DELETE(m_accel);
        AxisCamera::GetInstance().DeleteInstance();

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
        // Initialize camera.
        //
        m_camera.WriteResolution(AxisCamera::kResolution_160x120);
        m_camera.WriteCompression(10);
        m_camera.WriteBrightness(25);

        //
        // Initialize RobotDrive.
        //
        SetInvertedMotor(RobotDrive::kFrontLeftMotor, MOTOR_FRONTLEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kFrontRightMotor,MOTOR_FRONTRIGHT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearLeftMotor,MOTOR_REARLEFT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor, MOTOR_REARRIGHT_REVERSE);
        m_pidCtrlCamera->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlXAccel->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlYAccel->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlGyro->SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);

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

        m_pidVisionDrive->Stop();
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
     * This function sets PID drive target with the given drive X and Y
     * distance. The orientation is control by the camera vision targetting
     * system.
     *
     * @param distXSetPoint Specifies the lateral target distance relative
     *        to current position.
     * @param distYSetPoint Specifies the forward target distance relative
     *        to current position.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    VisionDriveSetTarget(
        __in float  distXSetPoint,
        __in float  distYSetPoint,
        __in bool   fStopOnTarget = true,
        __in Event *notifyEvent = NULL,
        __in UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distXSetPt=%f,distYSetPt=%f,fStopOnTarget=%x,event=%p,"
                   "timeout=%d",
                   distXSetPoint, distYSetPoint, fStopOnTarget, notifyEvent,
                   timeout));

        m_pidVisionDrive->SetTarget(distXSetPoint,
                                    distYSetPoint,
                                    0.0,        //target center
                                    fStopOnTarget,
                                    notifyEvent,
                                    timeout);

        TExit();
        return;
    }   //VisionDriveSetTarget

    /**
     * This function drives the robot with the given x and y power but use
     * the camera to control the rotation of the robot.
     *
     * @param xPower Specifies the drive power for the X direction.
     * @param yPower Specifies the drive power for the Y direction.
     */
    void
    VisionDrive(
        __in float xPower,
        __in float yPower
        )
    {
        TLevel(API);
        TEnterMsg(("xPower=%f,yPower=%f", xPower, yPower));

        m_pidVisionDrive->SetAngleTarget(xPower, yPower, 0.0);

        TExit();
        return;
    }   //VisionDrive

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

        if (pidCtrl == m_pidCtrlCamera)
        {
            if (m_camera.IsFreshImage())
            {
                UINT32 timeStart = GetMsecTime();
                // find FRC targets in the image
                HSLImage *image = m_camera.GetImage();
                m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,
                                    "GetImageTime: %d",
                                    GetMsecTime() - timeStart);
                timeStart = GetMsecTime();
                vector<Target> targets = Target::FindCircularTargets(image);
                m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,
                                    "GetImageTime: %d",
                                    GetMsecTime() - timeStart);
                delete image;
                if ((targets.size() > 0) &&
                    (targets[0].m_score >= MIN_TARGET_SCORE))
                {
                    input = targets[0].GetHorizontalAngle();
                    prevInput = input;
                    m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
                                        "Target found at %f", input);
                }
                m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,
                                    "Time: %d", GetMsecTime() - timeStart);
            }
        }
        else if (pidCtrl == m_pidCtrlXAccel)
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

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput
};  //class BaseDrive

#endif  //ifndef _DRIVEBASE_H
