#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="DbgTrace.h" />
///
/// <summary>
///     This module contains the definitions of the DbgTrace class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DBGTRACE_H
#define _DBGTRACE_H

//
// Module ID.
//
#define MOD_LIB                 0xffffff00
#define MOD_TRCROBOT            0x00000100
#define MOD_JOYSTICK            0x00000200
#define MOD_DIGITALIN           0x00000400
#define MOD_ANALOGIN            0x00000800
#define MOD_GYRO                0x00001000
#define MOD_ACCEL               0x00002000
#define MOD_PIDCTRL             0x00004000
#define MOD_PIDMOTOR            0x00008000
#define MOD_PIDDRIVE            0x00010000
#define MOD_LNFOLLOWER          0x00020000
#define MOD_SM                  0x00040000
#define MOD_TIMER               0x00080000
#define MOD_EVENT               0x00100000
#define MOD_SUBSYS              0x00200000
#define MOD_COOPMTROBOT         0x00400000

#define MOD_MAIN                0x00000001
#define TGenModId(n)            ((MOD_MAIN << (n)) & 0xff)

#define INIT                    0
#define API                     1
#define CALLBK                  2
#define EVENT                   3
#define FUNC                    4
#define TASK                    5
#define UTIL                    6
#define HIFREQ                  7

#define FATAL                   0
#define ERR                     1
#define WARN                    2
#define INFO                    3
#define VERBOSE                 4

#ifndef TRACE_PERIOD
  #define TRACE_PERIOD          1000    //in msec
#endif

#ifndef SAMPLING_PERIOD
  #define SAMPLING_PERIOD       500     //in msec
#endif

#define TPrintf                 printf

//
// Trace macros.
//
#ifdef _DBGTRACE_ENABLED
    #define TModEnterMsg(m,p)   if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       true, \
                                                       false); \
                                    TPrintf p; \
                                    TPrintf(")\n"); \
                                }
    #define TModEnter(m)        if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       true, \
                                                       true); \
                                }
    #define TModExitMsg(m,p)    if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       false, \
                                                       false); \
                                    TPrintf p; \
                                    TPrintf("\n"); \
                                }
    #define TModExit(m)         if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       false, \
                                                       true); \
                                }
    #define TModMsg(m,e,p)      if (((g_Trace.m_traceModules & (m)) != 0) && \
                                    ((e) <= g_Trace.m_msgLevel)) \
                                { \
                                    g_Trace.MsgPrefix(MOD_NAME, \
                                                      __FUNCTION__, \
                                                      e); \
                                    TPrintf p; \
                                    TPrintf("\n"); \
                                }
    #define TEnable(b)          g_Trace.m_fTraceEnabled = (b)
    #define TraceInit(m,l,e)    g_Trace.Initialize(m, l, e)
    #define TLevel(l)           UINT32 _traceLevel = l
    #define TEnterMsg(p)        TModEnterMsg(MOD_ID, p)
    #define TEnter()            TModEnter(MOD_ID)
    #define TExitMsg(p)         TModExitMsg(MOD_ID, p)
    #define TExit()             TModExit(MOD_ID)
    #define TMsg(e,p)           TModMsg(MOD_ID, e, p)
    #define TFatal(p)           TModMsg(MOD_ID, FATAL, p)
    #define TErr(p)             TModMsg(MOD_ID, ERR, p)
    #define TWarn(p)            TModMsg(MOD_ID, WARN, p)
    #define TInfo(p)            TModMsg(MOD_ID, INFO, p)
    #define TVerbose(p)         TModMsg(MOD_ID, VERBOSE, p)
    #define TMsgPeriod(t,p)     { \
                                    static UINT32 _usecNextTime = 0; \
                                    if (GetFPGATime() >= _usecNextTime) \
                                    { \
                                        _usecNextTime = GetFPGATime() + \
                                                        (UINT32)(t)*1000; \
                                        TModMsg(MOD_ID, INFO, p); \
                                    } \
                                }
    #define TSampling(p)        TMsgPeriod(SAMPLING_PERIOD, p)
    #define TAssertPeriod(t,p,r) { \
                                    UINT32 _time = GetMsecTime(); \
                                    float _period = (float)(_time - (t))/ \
                                                    1000.0; \
                                    t = _time; \
                                    if (fabs(_period - (p)) > (r)) \
                                    { \
                                        TWarn(("Period variance exceeding " \
                                               "tolerance (period=%f)", \
                                               _period)); \
                                    } \
                                }
    #define TPeriodStart()      if (GetFPGATime() >= g_Trace.m_traceTime) \
                                { \
                                    g_Trace.m_traceTime = GetFPGATime() + \
                                                          TRACE_PERIOD; \
                                    TEnable(true); \
                                }
    #define TPeriodEnd()        TEnable(false)
#else
    #define TEnable(b)
    #define TraceInit(m,l,e)
    #define TLevel(l)
    #define TEnterMsg(p)
    #define TEnter()
    #define TExitMsg(p)
    #define TExit()
    #define TMsg(e,p)
    #define TFatal(p)
    #define TErr(p)
    #define TWarn(p)
    #define TInfo(p)
    #define TVerbose(p)
    #define TMsgPeriod(t,p)
    #define TSampling(p)
    #define TAssertPeriod(t,p,r)
    #define TPeriodStart()
    #define TPeriodEnd()
#endif  //ifdef _DBGTRACE_ENABLED

/**
 * This class implements the debug tracing object. It provides two facilities.
 * One allows the functions to trace the enter and exit conditions of the call
 * by dumping the calling parameters of function entry and the return value
 * of function exit. The other one allows the function to print out different
 * level of messages such as fatal message, error message, warning message,
 * info message and verbose message etc.
 */
class DbgTrace
{
public:
    bool   m_fTraceEnabled;
    UINT32 m_traceModules;
    UINT32 m_traceLevel;
    UINT32 m_msgLevel;
    UINT32 m_traceTime;

private:
    INT32  m_indentLevel;

public:
    /**
     * Constructor for the DbgTrace object.
     */
    DbgTrace(
        void
        )
    {
        m_fTraceEnabled = false;
        m_traceModules = 0;
        m_traceLevel = 0;
        m_msgLevel = 0;
        m_traceTime = 0;
        m_indentLevel = 0;
    }   //DbgTrace

    /**
     * Destructor for the DbgTrace object.
     */
    virtual
    ~DbgTrace(
        void
        )
    {
    }   //~DbgTrace

    /**
     * This function initializes the tracing module with the specified
     * module IDs and trace levels.
     *
     * @param traceModules Bit mask specifying which modules to enable tracing
     *        with. Each module is assigned a bit ID in the bit mask.
     * @param traceLevel Specifies the function trace level at or below which
     *        function tracing is enabled.
     * @param msgLevel Specifies the message trace level at or below which
     *        message tracing is enabled.
     */
    void
    Initialize(
        __in UINT32 traceModules,
        __in UINT32 traceLevel,
        __in UINT32 msgLevel
        )
    {
        m_traceModules = traceModules;
        m_traceLevel = traceLevel;
        m_msgLevel = msgLevel;
        m_indentLevel = 0;
    }   //Initialize

    /**
     * This method generates the function trace prefix string. The prefix
     * contains the indentation, the module name and the function name.
     *
     * @param pszMod Specifies the name of the module.
     * @param pszFunc Specifies the name of the function.
     * @param fEnter Specifies whether we are entering or exiting the
     *        function.
     * @param fNewLine Specifies whether we will print a new line.
     */
    void
    FuncPrefix(
        __in const char *pszMod,
        __in const char *pszFunc,
        __in bool        fEnter,
        __in bool        fNewLine
        )
    {
        if (fEnter)
        {
            m_indentLevel++;
        }

        for (INT32 i = 0; i < m_indentLevel; i++)
        {
            TPrintf("| ");
        }

        TPrintf("%s.%s", pszMod, pszFunc);

        if (fEnter)
        {
            TPrintf("%s", fNewLine? "()\n": "(");
        }
        else
        {
            TPrintf("%s", fNewLine? "!\n": "!");
            m_indentLevel--;
        }
    }   //FuncPrefix

    /**
     * This method generates the message trace prefix string. The prefix
     * contains the module and function names as well as message level info
     * in which the message is printed.
     *
     * @param pszMod Specifies the name of the module.
     * @param pszFunc Specifies the name of the function.
     * @param msgLevel Specifies message level.
     */
    void
    MsgPrefix(
        __in const char *pszMod,
        __in const char *pszFunc,
        __in UINT32      msgLevel
        )
    {
        char *pszPrefix = "_Unk: ";

        switch (msgLevel)
        {
        case FATAL:
            pszPrefix = "_Fatal: ";
            break;

        case ERR:
            pszPrefix = "_Err: ";
            break;

        case WARN:
            pszPrefix = "_Warn: ";
            break;

        case INFO:
            pszPrefix = "_Info: ";
            break;

        case VERBOSE:
            pszPrefix = "_Verbose: ";
            break;
        }

        TPrintf("%s.%s%s", pszMod, pszFunc, pszPrefix);
    }   //MsgPrefix
};	//class DbgTrace

#ifdef _DBGTRACE_ENABLED
    DbgTrace g_Trace;
#endif

#endif  //ifndef _DBGTRACE_H
