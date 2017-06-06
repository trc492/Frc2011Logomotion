#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="batt.h" />
///
/// <summary>
///     This module contains the library functions for dealing with the NXT
///     battery and the external battery.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _BATT_H
#define _BATT_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_BATT

//
// Type definitions.
//
typedef struct
{
    int  summaryLineNum;
    bool fDetails;
    int  minIntBatt;
    int  maxIntBatt;
    int  minExtBatt;
    int  maxExtBatt;
} BATT;

BATT l_Batt;

/**
 *  This function display the battery info on the NXT LCD screen.
 *
 *  @param batt Points to the BATT structure.
 *  @param lineNum Specifies the line number to display the info.
 *  @param fDetails If true, also display the min and max voltages of the
 *         external battery
 */
void
BattShowInfo(
    __in BATT &batt,
    __in int lineNum,
    __in bool fDetails
    )
{
    nxtDisplayTextLine(lineNum, "Int:%3.1f Ext:%4.1f",
                       (float)nAvgBatteryLevel/1000.0,
                       (externalBatteryAvg < 0)?
                            0.0: (float)externalBatteryAvg/1000.0);

    if (fDetails)
    {
        nxtDisplayTextLine(lineNum + 1, "%4.1f<=Int<=%4.1f",
                           l_Batt.minIntBatt/1000.0, l_Batt.maxIntBatt/1000.0);
        nxtDisplayTextLine(lineNum + 2, "%4.1f<=Ext<=%4.1f",
                           (l_Batt.minExtBatt < 0)?
                                0.0: (float)l_Batt.minExtBatt/1000.0,
                           (l_Batt.maxExtBatt < 0)?
                                0.0: (float)l_Batt.maxExtBatt/1000.0);
    }

    return;
}   //BattShowInfo

/**
 *  This function initializes the BATT info.
 *
 *  @param batt Specifies the BATT structure to be initialized.
 *  @param summaryLineNum If positive, a battery summary line will be displayed
 *         at the given line number. If -1, no battery summary is displayed.
 *  @param fDetails If true, show detail voltage range of the batteries.
 */
void
BattInit(
    __out BATT &batt,
    __in  int summaryLineNum,
    __in  bool fDetails
    )
{
    TFuncName("BattInit");
    TLevel(INIT);
    TEnter();

    l_Batt.summaryLineNum = summaryLineNum;
    l_Batt.fDetails = fDetails;
    l_Batt.minIntBatt = nAvgBatteryLevel;
    l_Batt.maxIntBatt = l_Batt.minIntBatt;
    l_Batt.minExtBatt = externalBatteryAvg;
    l_Batt.maxExtBatt = l_Batt.minExtBatt;
    StopTask(displayDiagnostics);
    eraseDisplay();

    TExit();
    return;
}   //BattInit

/**
 *  This function checks the battery voltages against the min and max values
 *  for determining the operating voltage range. This is helpful to determine
 *  if the batteries are running low under load.
 *
 *  @param batt Points to the BATT structure.
 */
void
BattTask(
    __inout BATT &batt
    )
{
    int currIntBatt;
    int currExtBatt;

    TFuncName("BattTask");
    TLevel(TASK);
    TEnter();

    currIntBatt = nAvgBatteryLevel;
    currExtBatt = externalBatteryAvg;

    if (currIntBatt < l_Batt.minIntBatt)
    {
        l_Batt.minIntBatt = currIntBatt;
    }
    else if (currIntBatt > l_Batt.maxIntBatt)
    {
        l_Batt.maxIntBatt = currIntBatt;
    }

    if (currExtBatt < l_Batt.minExtBatt)
    {
        l_Batt.minExtBatt = currExtBatt;
    }
    else if (currExtBatt > l_Batt.maxExtBatt)
    {
        l_Batt.maxExtBatt = currExtBatt;
    }

    if (l_Batt.summaryLineNum >= 0)
    {
        BattShowInfo(batt, l_Batt.summaryLineNum, l_Batt.fDetails);
    }

    TExit();
    return;
}   //BattTask

#endif  //ifndef _BATT_H
