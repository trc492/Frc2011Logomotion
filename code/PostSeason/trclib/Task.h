#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Task.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     CoopTask and TaskMgr classes.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TASK_H
#define _TASK_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TASK
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Task"

//
// Constants.
//
#define MAX_NUM_TASKS           32

#define TASK_INIT               0x00000001
#define TASK_START              0x00000002
#define TASK_STOP               0x00000004
#define TASK_PROCESS_INPUT      0x00000008
#define TASK_PROCESS_ACTION     0x00000010

#define MODE_DISABLED           0
#define MODE_AUTONOMOUS         1
#define MODE_TELEOP             2

/**
 * This abstract class defines the CoopTask object. The object is a callback
 * interface. It is not meant to be created as an object. Instead, it should
 * be inherited by a subclass who needs to be called as a cooperative task.
 */
class CoopTask
{
public:
    /**
     * This function registers a CoopTask object.
     *
     * @param flags Specifies the CoopTask callback types.
     *
     * @return Returns true if the CoopTask is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterTask(
        __in UINT32 flags
        );

    /**
     * This function is called to initialize a CoopTask.
     */
    virtual
    void
    InitTask(
        void
        )
    {
        TLevel(CALLBK);
        TEnter();
        TExit();
        return;
    }   //InitTask

    /**
     * This function is called to start a CoopTask.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    StartTask(
        __in UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //StartTask

    /**
     * This function is called to stop a CoopTask.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    StopTask(
        __in UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //StopTask

    /**
     * This function is called to process inputs of a CoopTask.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    ProcessInput(
        __in UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //ProcessInput

    /**
     * This function is called to process actions of a CoopTask.
     *
     * @param mode Specifies the caller mode.
     */
    virtual
    void
    ProcessAction(
        __in UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        TExit();
        return;
    }   //ProcessAction
};  //class CoopTask

/**
 * This class defines and implements the TaskMgr object.
 */
class TaskMgr
{
private:
    static TaskMgr *m_instance;
    int             m_numTasks;
    CoopTask       *m_tasks[MAX_NUM_TASKS];
    UINT32          m_taskFlags[MAX_NUM_TASKS];

protected:
    /**
     * Constructor: Create an instance of the TaskMgr object.
     */
    TaskMgr(
        void
        ): m_numTasks(0)
    {
        TLevel(INIT);
        TEnter();

        for (int idx = 0; idx < MAX_NUM_TASKS; idx++)
        {
            m_tasks[idx] = NULL;
            m_taskFlags[idx] = 0;
        }

        TExit();
    }   //TaskMgr

public:
    /**
     * Destructor: Destroy an instance of the TaskMgr object.
     */
    ~TaskMgr(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TaskMgr

    /**
     * This function returns the global instance of the TaskMgr, create it
     * if necessary.
     */
    static
    TaskMgr *
    GetInstance(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_instance == NULL)
        {
            m_instance = new TaskMgr();
        }

        TExitMsg(("=%p", m_instance));
        return m_instance;
    }   //GetInstance

    /**
     * This function deletes the global instance of the TaskMgr if it exists.
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
     * This function registers a CoopTask object.
     *
     * @param task Specifies the CoopTask to be registered with TaskMgr.
     * @param flags Specifies the CoopTask callback types.
     *
     * @return Returns true if the CoopTask is successfully registered, false
     *         otherwise.
     */
    bool
    RegisterTask(
        __in CoopTask *task,
        __in UINT32    flags
        )
    {
        bool rc = false;

        TLevel(API);
        TEnterMsg(("task=%p,flags=%x", task, flags));

        if (m_numTasks < MAX_NUM_TASKS)
        {
            m_tasks[m_numTasks] = task;
            m_taskFlags[m_numTasks] = flags;
            m_numTasks++;
            rc = true;
        }

        TExitMsg(("=%x", rc));
        return rc;
    }   //RegisterTask

    /**
     * This function initializes all the registered tasks.
     */
    void
    InitAllTasks(
        void
        )
    {
        TLevel(CALLBK);
        TEnter();

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_INIT)
            {
                m_tasks[idx]->InitTask();
            }
        }

        TExit();
    }   //InitAllTasks

    /**
     * This function starts all the registered tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    StartAllTasks(
        __in UINT32 mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_START)
            {
                m_tasks[idx]->StartTask(mode);
            }
        }

        TExit();
        return;
    }   //StartAllTasks

    /**
     * This function stops all the registered tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    StopAllTasks(
        __in UINT32 mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        for (int idx = m_numTasks - 1; idx >= 0; idx--)
        {
            if (m_taskFlags[idx] & TASK_STOP)
            {
                m_tasks[idx]->StopTask(mode);
            }
        }

        TExit();
        return;
    }   //StopAllTasks

    /**
     * This function processes the input of all the registered tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    ProcessAllInputs(
        __in UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = 0; idx < m_numTasks; idx++)
        {
            if (m_taskFlags[idx] & TASK_PROCESS_INPUT)
            {
                m_tasks[idx]->ProcessInput(mode);
            }
        }

        TExit();
        return;
    }   //ProcessAllInputs

    /**
     * This function processes the action of all the registered tasks.
     *
     * @param mode Specifies the caller mode.
     */
    void
    ProcessAllActions(
        __in UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        for (int idx = m_numTasks - 1; idx >= 0; idx--)
        {
            if (m_taskFlags[idx] & TASK_PROCESS_ACTION)
            {
                m_tasks[idx]->ProcessAction(mode);
            }
        }

        TExit();
        return;
    }   //ProcessAllActions
};  //class TaskMgr

TaskMgr* TaskMgr::m_instance = NULL;

/**
 * This function registers a CoopTask object.
 *
 * @param flags Specifies the CoopTask callback types.
 *
 * @return Returns true if the CoopTask is successfully registered, false
 *         otherwise.
 */
bool
CoopTask::RegisterTask(
    __in UINT32 flags
    )
{
    bool rc = false;
    TaskMgr *taskMgr = TaskMgr::GetInstance();

    TLevel(API);
    TEnterMsg(("flags=%x", flags));

    if (taskMgr != NULL)
    {
        rc = taskMgr->RegisterTask(this, flags);
    }

    TExitMsg(("=%x", rc));
    return rc;
}   //RegisterTask

#endif  //ifndef _TASK_H
