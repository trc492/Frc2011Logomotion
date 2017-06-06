#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Deployer.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Deployer class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DEPLOYER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Deployer"

/**
 * This module defines and implements the Minibot Deployer subsytem. The
 * subsystem consists of a servo motor that pulls the pin to release the
 * spring mechanism that extends the deployer outward.
 */
class Deployer: public SubSystem
{
private:
    Servo *m_deployMotor;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Deployer(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_deployMotor = new Servo(PWM_DEPLOYER_MOTOR);
        RegisterSubSystem(SUBSYS_INIT);

        TExit();
    }   //Deployer

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Deployer(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SAFE_DELETE(m_deployMotor);

        TExit();
    }   //~Deployer

    /**
     * This function is called one time to do subsystem initialization.
     */
    void
    Init(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_deployMotor->SetAngle(DEPLOY_ANGLE_LATCHED);

        TExit();
    }   //Init

#if 0
    /**
     * This function is called by the Disabled or Initialize code to stop the
     * subsystem.
     */
    void
    Stop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //Stop
#endif

    /**
     * This function sets the deployer to extend or retract.
     *
     * @param extend If true, it extends the deployment platform, retract
     *        otherwise.
     */
    void
    SetDeploy(
        __in bool extend
        )
    {
        TLevel(API);
        TEnter();

        m_deployMotor->SetAngle(extend? DEPLOY_ANGLE_UNLATCHED:
                                        DEPLOY_ANGLE_LATCHED);

        TExit();
    }   //SetDeploy
};  //class Deployer
