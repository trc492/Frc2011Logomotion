#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="SubSystem.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     SubSystem and SubSystemMgr class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _SUBSYSTEM_H
#define _SUBSYSTEM_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SUBSYS
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "SubSys"

//
// Constants.
//
#define MAX_NUM_SUBSYSTEMS      16

#define SUBSYS_INIT             0x00000001
#define SUBSYS_START            0x00000002
#define SUBSYS_STOP             0x00000004
#define SUBSYS_INPUT_TASKS      0x00000008
#define SUBSYS_ACTION_TASKS     0x00000010

#define CONTEXT_NONE            0
#define CONTEXT_DISABLED        1
#define CONTEXT_AUTONOMOUS      2
#define CONTEXT_TELEOP          3

/**
 * This abstract class defines the SubSystem object. The object is a callback
 * interface. It is not meant to be created as an object. Instead, it should
 * be inherited by a subclass who needs to be called as a subsystem.
 */
class SubSystem
{
public:
    /**
     * This function registers a subsystem object.
     *
     * @param flags Specifies the subsystem callback types.
     *
     * @return Returns true if the subsystem is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterSubSystem(
        __in UINT32 flags
        );

    /**
     * This function is called to initialize a subsystem.
     */
    virtual
    void
    Init(
        void
        )
    {
        TLevel(CALLBK);
        TEnter();
        TExit();
        return;
    }   //Init

    /**
     * This function is called to start a subsystem.
     *
     * @param context Specifies the caller context.
     */
    virtual
    void
    Start(
        __in UINT32 context
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("context=%d", context));
        TExit();
        return;
    }   //Start

    /**
     * This function is called to stop a subsystem.
     *
     * @param context Specifies the caller context.
     */
    virtual
    void
    Stop(
        __in UINT32 context
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("context=%d", context));
        TExit();
        return;
    }   //Stop

    /**
     * This function is called to perform subsystem input tasks.
     *
     * @param context Specifies the caller context.
     */
    virtual
    void
    InputTasks(
        __in UINT32 context
        )
    {
        TLevel(TASK);
        TEnterMsg(("context=%d", context));
        TExit();
        return;
    }   //InputTasks

    /**
     * This function is called to perform subsystem action tasks.
     *
     * @param context Specifies the caller context.
     */
    virtual
    void
    ActionTasks(
        __in UINT32 context
        )
    {
        TLevel(TASK);
        TEnterMsg(("context=%d", context));
        TExit();
        return;
    }   //ActionTasks
};  //class SubSystem

/**
 * This class defines and implements the SubSystemMgr object.
 */
class SubSystemMgr
{
private:
    static SubSystemMgr *m_instance;
    int                  m_numSubSystems;
    SubSystem           *m_subsystems[MAX_NUM_SUBSYSTEMS];
    UINT32               m_subsysFlags[MAX_NUM_SUBSYSTEMS];

protected:
    /**
     * Constructor: Create an instance of the SubSystemMgr object.
     */
    SubSystemMgr(
        void
        ): m_numSubSystems(0)
    {
        TLevel(INIT);
        TEnter();

        for (int idx = 0; idx < MAX_NUM_SUBSYSTEMS; idx++)
        {
            m_subsystems[idx] = NULL;
            m_subsysFlags[idx] = 0;
        }

        TExit();
    }   //SubSystemMgr

public:
    /**
     * Destructor: Destroy an instance of the SubSystemMgr object.
     */
    ~SubSystemMgr(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~SubSystemMgr

    /**
     * This function returns the global instance of the subsystem manager,
     * create it if it has not be created yet.
     */
    static
    SubSystemMgr *
    GetInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_instance == NULL)
        {
            m_instance = new SubSystemMgr();
        }

        TExitMsg(("=%p", m_instance));
        return m_instance;
    }   //GetInstance

    /**
     * This function deletes the global instance of the SubSystemMgr if it
     * exists.
     */
    static
    void
    DeleteInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        SAFE_DELETE(m_instance);

        TExit();
        return;
    }   //DeleteInstance

    /**
     * This function registers a subsystem object.
     *
     * @param subsystem Specifies the subsystem to be registered with
     *        subsystem manager.
     * @param flags Specifies the subsystem callback types.
     *
     * @return Returns true if the subsystem is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterSubSystem(
        __in SubSystem *subsystem,
        __in UINT32     flags
        )
    {
        bool rc = false;

        TLevel(API);
        TEnterMsg(("subsys=%p,flags=%x", subsystem, flags));

        if (m_numSubSystems < MAX_NUM_SUBSYSTEMS)
        {
            m_subsystems[m_numSubSystems] = subsystem;
            m_subsysFlags[m_numSubSystems] = flags;
            m_numSubSystems++;
            rc = true;
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //RegisterSubSystem

    /**
     * This function initializes all the registered subsystems.
     */
    void
    InitSubSystems(
        void
        )
    {
        TLevel(CALLBK);
        TEnter();

        for (int idx = 0; idx < m_numSubSystems; idx++)
        {
            if (m_subsysFlags[idx] & SUBSYS_INIT)
            {
                m_subsystems[idx]->Init();
            }
        }

        TExit();
    }   //InitSubSystems

    /**
     * This function starts all the registered subsystems.
     *
     * @param context Specifies the caller context.
     */
    void
    StartSubSystems(
        __in UINT32 context
        )
    {
        TLevel(API);
        TEnterMsg(("context=%d", context));

        TInfo(("Total subsystems=%d", m_numSubSystems));
        for (int idx = 0; idx < m_numSubSystems; idx++)
        {
            TInfo(("Starting subsys: %d, flags=%x", idx, m_subsysFlags[idx]));
            if (m_subsysFlags[idx] & SUBSYS_START)
            {
                m_subsystems[idx]->Start(context);
            }
        }

        TExit();
        return;
    }   //StartSubSystems

    /**
     * This function stops all the registered subsystems.
     *
     * @param context Specifies the caller context.
     */
    void
    StopSubSystems(
        __in UINT32 context
        )
    {
        TLevel(API);
        TEnterMsg(("context=%d", context));

        for (int idx = m_numSubSystems - 1; idx >= 0; idx--)
        {
            if (m_subsysFlags[idx] & SUBSYS_STOP)
            {
                m_subsystems[idx]->Stop(context);
            }
        }

        TExit();
        return;
    }   //StopSubSystems

    /**
     * This function performs the input tasks of all the registered
     * subsystems.
     *
     * @param context Specifies the caller context.
     */
    void
    SubSystemInputTasks(
        __in UINT32 context
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("context=%d", context));

        for (int idx = 0; idx < m_numSubSystems; idx++)
        {
            if (m_subsysFlags[idx] & SUBSYS_INPUT_TASKS)
            {
                m_subsystems[idx]->InputTasks(context);
            }
        }

        TExit();
        return;
    }   //SubSystemInputTasks

    /**
     * This function performs the action tasks of all the registered
     * subsystems.
     *
     * @param context Specifies the caller context.
     */
    void
    SubSystemActionTasks(
        __in UINT32 context
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("context=%d", context));

        for (int idx = m_numSubSystems - 1; idx >= 0; idx--)
        {
            if (m_subsysFlags[idx] & SUBSYS_ACTION_TASKS)
            {
                m_subsystems[idx]->ActionTasks(context);
            }
        }

        TExit();
        return;
    }   //SubSystemActionTasks
};  //class SubSystemMgr

SubSystemMgr* SubSystemMgr::m_instance = NULL;

/**
 * This function registers a subsystem object.
 *
 * @param flags Specifies the subsystem callback types.
 *
 * @return Returns true if the subsystem is successfully registered, false
 *         otherwise.
 */
bool
SubSystem::RegisterSubSystem(
    __in UINT32 flags
    )
{
    bool rc = false;
    SubSystemMgr *subsysMgr = SubSystemMgr::GetInstance();

    TLevel(API);
    TEnterMsg(("flags=%x", flags));

    if (subsysMgr != NULL)
    {
        rc = subsysMgr->RegisterSubSystem(this, flags);
    }

    TExitMsg(("=%x", rc));
    return rc;
}   //RegisterSubSystem

#endif  //ifndef _SUBSYSTEM_H
