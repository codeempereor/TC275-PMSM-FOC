
/**********************************************************************************************************************
  COPYRIGHT
-----------------------------------------------------------------------------------------------------------------------
  \par      copyright
  \verbatim
  Copyright (c) 2019 by Vector Informatik GmbH.                                                  All rights reserved.

                This software is copyright protected and proprietary to Vector Informatik GmbH.
                Vector Informatik GmbH grants to you only those rights as set out in the license conditions.
                All other rights remain with Vector Informatik GmbH.
  \endverbatim
-----------------------------------------------------------------------------------------------------------------------
  FILE DESCRIPTION
-----------------------------------------------------------------------------------------------------------------------
  \file  File:  BrsMain.c
      Project:  Vector Basic Runtime System
       Module:  BrsMain

  \brief Description:  Main file of BRS contains
                       - Main function (called from StartUpCode and calls stack entry EcuM_Init())
                       - Call of BrsHw HW initialization routines
                       - 1ms handler, with support of cycle functions with several cycle times
                       - Exception handling
                       - Default_Init_Task, Main and Background Task which can be used by the operating system

  \attention Please note:
    The demo and example programs only show special aspects of the software. With regard to the fact
    that these programs are meant for demonstration purposes only, Vector Informatik liability shall be
    expressly excluded in cases of ordinary negligence, to the extent admissible by law or statute.
**********************************************************************************************************************/

/**********************************************************************************************************************
  AUTHOR IDENTITY
 ----------------------------------------------------------------------------------------------------------------------
  Name                          Initials      Company
  ----------------------------  ------------  -------------------------------------------------------------------------
  Benjamin Walter               visbwa        Vector Informatik GmbH
-----------------------------------------------------------------------------------------------------------------------
  REVISION HISTORY
 ----------------------------------------------------------------------------------------------------------------------
  Version   Date        Author  Description
  --------  ----------  ------  ---------------------------------------------------------------------------------------
  01.00.00  2018-03-20  visbwa  Initial creation of new BRS code basis
            2018-05-08  visbwa  Introduced support for Generic Testsuite (BRS_ENABLE_TESTSUITE_SUPPORT),
                                removed support for all other testsuites,
                                introduced BRS_ENABLE_1MS_HANDLER and brsMain_CallCounter1sec
  01.00.01  2018-06-22  visbwa  Added CAN stack exclusive area workaround from former BswInit_Callout_Stubs.c
  01.00.02  2018-06-25  visbwa  Added configuration check for OS_MS2TICKS_SystemTimer into BrsMain.h
  01.00.03  2018-07-18  visbwa  Moved BrsHwDisableInterruptAtPowerOn() from BrsMainInit() into main()
                                (OS specifies, this has to be called before Os_Init() and for every Core)
  01.00.04  2018-07-30  visbwa  Encapsuled call of BrsHwDisableInterruptAtPowerOn() with usecase != VTT
  01.01.00  2018-10-08  visbwa  Changed include of BrsCfg.h into vBrsCfg.h in BrsMain.h (vBaseEnv 1.1.0)
  01.01.01  2018-10-10  visbwa  Moved BrsTestsuiteInit() from BrsMainInit() into Default_Init_Task(),
                                removed usage of BRS_ENABLE_HLPTEST_SUPPORT (generic Testsuite concept!)
  01.01.02  2018-11-05  visbwa  Fixed include order (Can.h previous to BrsTestsuite.h)
  01.01.03  2018-11-13  visbwa  Implemented Default_Init_Tasks for projects with up to 6 Cores,
                                fixed VTT support within IdleTasks (CANoeAPI_ConsumeTicks)
  01.02.00  2018-11-21  visbwa  Added testsuite callouts into every function
  01.02.01  2018-11-30  visbwa  Removed include of Rte_Main.h (Rte_Start() not called any more)
  01.03.00  2018-12-21  visbwa  Added call of new API Os_InitialEnableInterruptSources() within every Default_Init_Task
  01.03.01  2019-03-06  visbwa  Added hint for DrvCan using infix, added wrapper for Can_30_Mcan
  01.03.02  2019-06-13  vismaa  Added additional Default_Init_Task_Core6
  01.03.03  2019-06-26  visbwa  Moved call of BrsMainInit() previous to call of Os_Init()
  01.04.00  2019-07-02  visbwa  Enhanced FBL support (encapsulations, call of fblmain())
  01.04.01  2019-07-29  visbwa  Added implementation of BrsMainExceptionStartup() for calls from assembler StartUpCode
  01.05.00  2019-08-08  visbwa  Implemented support for different FBL usecases Legacy, WithoutOSScheduling and
                                OSScheduling, added BrsMainExceptionHandler module code for BrsMainStartup, changed FBL
                                call from fblmain() to FblMain() for legacy mode, removed include of fbl_main.h
**********************************************************************************************************************/

/**********************************************************************************************************************
*  EXAMPLE CODE ONLY
*  -------------------------------------------------------------------------------------------------------------------
*  This Example Code is only intended for illustrating an example of a possible BSW integration and BSW configuration.
*  The Example Code has not passed any quality control measures and may be incomplete. The Example Code is neither
*  intended nor qualified for use in series production. The Example Code as well as any of its modifications and/or
*  implementations must be tested with diligent care and must comply with all quality requirements which are necessary
*  according to the state of the art before their use.
*********************************************************************************************************************/

/**********************************************************************************************************************
  INCLUDES
**********************************************************************************************************************/
#include "BrsMain.h"

#if !defined (_MICROSOFT_C_VTT_)
  #include "BrsHw.h"
#endif

#include "LedTest/led_test.h"
#include "LedTest/uart_test.h"

//  #include "Can.h"
  /* if DrvCan with infix is used, change the header file accordingly (e.g. Can_30_Mcan.h) */

#if defined (BRS_ENABLE_TESTSUITE_SUPPORT)
  #include "BrsTestsuite.h"
#endif

/**********************************************************************************************************************
  VERSION CHECK
**********************************************************************************************************************/
#if (BRSMAIN_VERSION != 0x0105u)
  #error "Header and source file are inconsistent!"
#endif
#if (BRSMAIN_BUGFIX_VERSION != 0x00u)
  #error "Different versions of bugfix in Header and Source used!"
#endif

/**********************************************************************************************************************
  CONFIGURATION CHECK
**********************************************************************************************************************/

/**********************************************************************************************************************
  DEFINITION + MACROS
**********************************************************************************************************************/

/**********************************************************************************************************************
  GLOBAL VARIABLES
**********************************************************************************************************************/
#define BRSMAIN_CYCLIC_MAX_CALLBACKS 2u

typedef struct
{
  void (*FunctionPointer[BRSMAIN_CYCLIC_MAX_CALLBACKS]) (void);
  uint8 FunctionCounter;
}brsMain_Cyclic_Callbacks;

  /* UserDefined ExclusiveArea handling for CAN channels, according to AN-ISC-8-1149_ErrorHook_E_OS_DISABLED_INT.pdf */
  #define CAN_START_SEC_VAR_NOINIT_8BIT
  #include "MemMap.h"
  static uint8 BrsAsrApplCanInterruptLockCtr;
  #define CAN_STOP_SEC_VAR_NOINIT_8BIT
  #include "MemMap.h"

/**********************************************************************************************************************
  GLOBAL CONST VARIABLES
**********************************************************************************************************************/

/**********************************************************************************************************************
  LOCAL VARIABLES
**********************************************************************************************************************/
#if defined (BRS_ENABLE_1MS_HANDLER)
  #define START_SEC_VAR_ZERO_INIT_16BIT
  #include "MemMap.h"
  /**
   * \var brsMain_CallCounter1ms
   *      Counter for calls of the function BrsMainCyclic1ms
   */
  static volatile uint16 brsMain_CallCounter1ms;

  /**
   * \var brsMain_CallCounter1sec
   *      Counter for 1000 calls of the function BrsMainCyclic1ms
   */
  static volatile uint16 brsMain_CallCounter1sec;
  #define STOP_SEC_VAR 
  #include "MemMap.h"

  #define START_SEC_VAR_NOINIT_UNSPECIFIED
  #include "MemMap.h"
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_Background;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_1ms;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_10ms;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_100ms;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_250ms;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_500ms;
  static brsMain_Cyclic_Callbacks brsMain_Cyclic_Callbacks_1000ms;
  #define STOP_SEC_VAR 
  #include "MemMap.h"
#endif /*BRS_ENABLE_1MS_HANDLER*/

/**********************************************************************************************************************
  LOCAL CONST VARIABLES
**********************************************************************************************************************/
# if defined (_MICROSOFT_C_VTT_)
  #define BRSMAIN_ERRORBUFFERSIZE  120
#endif

/**********************************************************************************************************************
  PROTOTYPES OF GLOBAL FUNCTIONS
**********************************************************************************************************************/
#if defined (BRS_FBL_LEGACY)
extern void FblMain(void);
#endif

/**********************************************************************************************************************
  PROTOTYPES OF LOCAL FUNCTIONS
**********************************************************************************************************************/

/**********************************************************************************************************************
  FUNCTION DEFINITIONS
**********************************************************************************************************************/
#if defined (BRS_ENABLE_SUPPORT_LEDS)
void BrsMainWrapperTogglePinLED(void)
{
  BrsHwTogglePin(BRSHW_TOGGLEPIN_LED);
}
#endif

#if defined (BRS_ENABLE_SUPPORT_TOGGLE_WD_PIN)
void BrsMainWrapperTogglePinWD(void)
{
  BrsHwTogglePin(BRSHW_TOGGLEPIN_WD);
}
#endif

#if defined (BRS_ENABLE_SUPPORT_TOGGLE_CUSTOM_PIN)
void BrsMainWrapperTogglePinCustom(void)
{
  BrsHwTogglePin(BRSHW_TOGGLEPIN_CUSTOM);
}
#endif

/*****************************************************************************/
/**
 * @brief      Main initialization routine.
 *             Contains initialisation of BRSModules and BrsMain specific initialization
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    Function is called from main
 */
/*****************************************************************************/
void BrsMainInit(void)
{
#if defined (BRSMAIN_BRSMAININIT_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainInit();
#endif

  /* UserDefined ExclusiveArea handling for CAN channels, according to AN-ISC-8-1149_ErrorHook_E_OS_DISABLED_INT.pdf */
  BrsAsrApplCanInterruptLockCtr = 0u;

#if !defined (BRS_FBL_LEGACY)
# if defined (BRSHW_PREINIT_AVAILABLE)
  BrsHwPreInitPowerOn();
# endif

# if defined (BRS_ENABLE_WATCHDOG)
  BrsHwWatchdogInitPowerOn();
# endif

# if defined (BRS_ENABLE_PLLCLOCKS)
  BrsHwPllInitPowerOn();
# endif

# if defined (BRS_ENABLE_PORT)
  BrsHwPortInitPowerOn();
# endif
#endif /*!BRS_FBL_LEGACY*/

#if defined (BRS_ENABLE_1MS_HANDLER)
  brsMain_CallCounter1ms = 0u;
  brsMain_CallCounter1sec = 0u;

  brsMain_Cyclic_Callbacks_Background.FunctionCounter = 0u;
  brsMain_Cyclic_Callbacks_1ms.FunctionCounter        = 0u;
  brsMain_Cyclic_Callbacks_10ms.FunctionCounter       = 0u;
  brsMain_Cyclic_Callbacks_100ms.FunctionCounter      = 0u;
  brsMain_Cyclic_Callbacks_250ms.FunctionCounter      = 0u;
  brsMain_Cyclic_Callbacks_500ms.FunctionCounter      = 0u;
  brsMain_Cyclic_Callbacks_1000ms.FunctionCounter     = 0u;
#endif /*BRS_ENABLE_1MS_HANDLER*/

#if defined (BRS_ENABLE_SUPPORT_LEDS)
  BrsMainRegisterCyclic(BrsMainWrapperTogglePinLED, BRSMAIN_CYCLETIME_500MS);
#endif

#if defined (BRS_ENABLE_SUPPORT_TOGGLE_WD_PIN)
  BrsMainRegisterCyclic(BrsMainWrapperTogglePinWD, BRSMAIN_CYCLETIME_250MS);
#endif

#if defined (BRS_ENABLE_SUPPORT_TOGGLE_CUSTOM_PIN)
  BrsMainRegisterCyclic(BrsMainWrapperTogglePinCustom, BRSMAIN_CYCLETIME_1000MS);
#endif

  LedTest_Init();
}

#if defined (BRS_ENABLE_1MS_HANDLER)
/*****************************************************************************/
/**
 * @brief      Routine to register cyclic callbacks.
 * @pre        Initialization of BrsMain was done threw call of BrsMainInit().
 * @param[in]  FunctionPointer has to be a pointer to a function of type
 *             void function(void).
 * @param[in]  Cycletime described the cycletime, the callback should be triggered.
 * @param[out] -
 * @return     -
 * @context    Function is called from modules that need cyclic callbacks.
 */
/*****************************************************************************/
void BrsMainRegisterCyclic(void (*FunctionPointer)(void), brsMain_Cyclic_Cycletime Cycletime)
{
#if defined (BRSMAIN_BRSMAINREGISTERCYCLIC_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainRegisterCyclic(FunctionPointer, Cycletime);
#endif

  switch (Cycletime)
  {
    case BRSMAIN_CYCLETIME_BACKGROUND:
      if (brsMain_Cyclic_Callbacks_Background.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_Background.FunctionPointer[brsMain_Cyclic_Callbacks_Background.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_Background.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_1MS:
      if (brsMain_Cyclic_Callbacks_1ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_1ms.FunctionPointer[brsMain_Cyclic_Callbacks_1ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_1ms.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_10MS:
      if (brsMain_Cyclic_Callbacks_10ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_10ms.FunctionPointer[brsMain_Cyclic_Callbacks_10ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_10ms.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_100MS:
      if (brsMain_Cyclic_Callbacks_100ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_100ms.FunctionPointer[brsMain_Cyclic_Callbacks_100ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_100ms.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_250MS:
      if (brsMain_Cyclic_Callbacks_250ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_250ms.FunctionPointer[brsMain_Cyclic_Callbacks_250ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_250ms.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_500MS:
      if (brsMain_Cyclic_Callbacks_500ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_500ms.FunctionPointer[brsMain_Cyclic_Callbacks_500ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_500ms.FunctionCounter++;
      break;

    case BRSMAIN_CYCLETIME_1000MS:
      if (brsMain_Cyclic_Callbacks_1000ms.FunctionCounter >= BRSMAIN_CYCLIC_MAX_CALLBACKS)
        BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      brsMain_Cyclic_Callbacks_1000ms.FunctionPointer[brsMain_Cyclic_Callbacks_1000ms.FunctionCounter] = FunctionPointer;
      brsMain_Cyclic_Callbacks_1000ms.FunctionCounter++;
      break;

    default:
      BrsMainExceptionHandler(kBrsIllegalParameter, BRSERROR_MODULE_BRSMAIN, (uint16)(__LINE__));
      break;
  }
}
#endif /*BRS_ENABLE_1MS_HANDLER*/

/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo()
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS
 */
/*****************************************************************************/
TASK(Default_Init_Task)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task();
#endif

#if defined (BRS_ENABLE_OS_MULTICORESUPPORT)
  /* Workaround for RTE ESCAN00078832 */
  /* Use this code, if you get a Det Error at the end of Rte_Start() on MasterCore */
   //Rte_Start() ;/*on the SlaveCores has to be called first, before Rte_Start() on MasterCore */
  /* SET THIS InitTask TO FULL PREEMPTIVE (OsTaskSchedule) within OsConfig! */
  /*while(Rte_InitState_1 != RTE_STATE_INIT)
  {
    (void)Schedule();
  }*/
#endif /*BRS_ENABLE_OS_MULTICORESUPPORT*/


#if !defined (BRS_FBL_LEGACY) && !defined (BRS_FBL_NO_OSSCHEDULE)
  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);
#endif

  LedTest_Init();
  KeyTest_Init();
  UartTest_Init();
  UartTest_SendString("TC275 UART HelloWorld!\r\n");

  {
    uint32 cnt = 0;
    while(1)
    {
      if (KeyTest_Read())
        LedTest_SetLed1(1);
      else
        LedTest_SetLed1(0);

      LedTest_ToggleLed2();

      cnt++;
      UartTest_SendString("cnt=");
      UartTest_SendUint32(cnt);
      UartTest_SendString("\r\n");

      for(volatile uint32 i = 0; i < 5000000; i++);
    }
  }

  (void)TerminateTask();
}

#if defined (BRS_ENABLE_OS_MULTICORESUPPORT)
/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 2nd Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 2nd Core
 */
/*****************************************************************************/
TASK(Default_Init_Task_Core1)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE1_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core1();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}

/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 3rd Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 3rd Core
 */
/*****************************************************************************/
TASK(Default_Init_Task_Core2)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE2_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core2();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}

/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 4th Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 4th Core
 */
/*****************************************************************************/


#if defined (BRS_DERIVATIVE_GROUP_TC3xx)

TASK(Default_Init_Task_Core3)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE3_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core3();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}

#endif

/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 5th Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 5th Core
 */
/*****************************************************************************/

#if defined (BRS_DERIVATIVE_GROUP_TC3xx)
TASK(Default_Init_Task_Core4)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE4_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core4();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}
#endif
/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 6th Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 6th Core
 */
/*****************************************************************************/
#if defined (BRS_DERIVATIVE_GROUP_TC3xx)

TASK(Default_Init_Task_Core5)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE5_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core5();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}
#endif
/*****************************************************************************/
/**
 * @brief      InitTask to call EcuM_StartupTwo() on 7th Core
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS on the 7th Core
 */
/*****************************************************************************/
#if defined (BRS_DERIVATIVE_GROUP_TC4xx)

TASK(Default_Init_Task_Core6)
{
#if defined (BRSMAIN_DEFAULT_INIT_TASK_CORE6_CALLOUT)
  BrsTestsuite_BrsMain_Default_Init_Task_Core6();
#endif

  EcuM_StartupTwo();

  Os_InitialEnableInterruptSources(FALSE);

  (void)TerminateTask();
}
#endif

#endif /*BRS_ENABLE_OS_MULTICORESUPPORT*/

#if defined (BRS_ENABLE_1MS_HANDLER)
/*****************************************************************************/
/**
 * @brief      One millisecond handler for BrsMain 
 *               - Executes retransmission of BRS TCC messages
 *               - Toggling of LED (alive signal)
 *               - BRS Test code (1s cyclic negative TCC response message)
 * @pre        Initialization of BrsMain was done threw call of BrsMainInit().
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    Function is called each millisecond either from the main loop or from the BRS main task (TASK(mainTask))
 */
/*****************************************************************************/
static void BrsMainCyclic1ms(void)
{
  uint8 cycliccounter;

#if defined (BRSMAIN_BRSMAINCYCLIC1MS_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainCyclic1ms();
#endif

  brsMain_CallCounter1ms++;

  LedTest_Run1ms();

  for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_1ms.FunctionCounter; cycliccounter++)
    brsMain_Cyclic_Callbacks_1ms.FunctionPointer[cycliccounter]();

  if (brsMain_CallCounter1ms %10 == 0)
  {
    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_10ms.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_10ms.FunctionPointer[cycliccounter]();
  }

  if (brsMain_CallCounter1ms %100 == 0)
  {
    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_100ms.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_100ms.FunctionPointer[cycliccounter]();
  }

  if (brsMain_CallCounter1ms %250 == 0)
  {
    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_250ms.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_250ms.FunctionPointer[cycliccounter]();
  }

  if (brsMain_CallCounter1ms %500 == 0)
  {
    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_500ms.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_500ms.FunctionPointer[cycliccounter]();
  }

  if (brsMain_CallCounter1ms %1000 == 0)
  {
    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_1000ms.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_1000ms.FunctionPointer[cycliccounter]();
    brsMain_CallCounter1ms = 0u;
    brsMain_CallCounter1sec++;
  }
}

/*****************************************************************************/
/**
 * @brief      BrsMainTask executes the 1 millisecond handler
 *             The function initiates calls to BrsMainCyclic1ms
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    The task is started once by the OS and activated by
 *             the OS event EvCyclicAlarm_1ms
 */
/*****************************************************************************/
TASK(BrsMainTask)
{
  EventMaskType ev;

#if defined (BRSMAIN_BRSMAINTASK_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainTask();
#endif

  (void)SetRelAlarm(BrsCyclicAlarm_1ms, OS_MS2TICKS_SystemTimer(1), OS_MS2TICKS_SystemTimer(1));

  for(;;)
  {
    (void)WaitEvent(BrsEvCyclicAlarm_1ms);
    (void)GetEvent(BrsMainTask, &ev);
    (void)ClearEvent(ev);
    if(ev & BrsEvCyclicAlarm_1ms)
    {
      /* 1ms event detected, call the ms handler */
      BrsMainCyclic1ms();
    }
  }
}

TASK(BrsMainBackgroundTask)
{
  uint8 cycliccounter;

#if defined (BRSMAIN_BRSMAINBACKGROUNDTASK_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainBackgroundTask();
#endif

  for(;;)
  {
# if defined (_MICROSOFT_C_VTT_)
    CANoeAPI_ConsumeTicks(100);
# endif

    for (cycliccounter=0u; cycliccounter<brsMain_Cyclic_Callbacks_Background.FunctionCounter; cycliccounter++)
      brsMain_Cyclic_Callbacks_Background.FunctionPointer[cycliccounter]();

    (void)Schedule();
  }
}
#endif /*BRS_ENABLE_1MS_HANDLER*/


/*****************************************************************************/
/**
 * @brief      This is the central exeption handler of BRS.
 *             All BRS modules and all CANbedded specific fatal error handler will
 *             call this API in case an assertion has failed.
 * @pre        -
 * @param[in]  ErrorCode shall hold the caller specific error code (uint8)
 * @param[in]  ModuleCode shall describe the caller; please use the CI constant as parameter (uint8)
 * @param[in]  LineNumber shall be the line where the assertion has failed, or, 
 *             if not available on caller location, the line where this API is 
 *             called from
 * @param[out] -
 * @return     -
 * @context    This function implements an endless loop with locked interrupts.
 *             The recommendation is to set a breakpoint on top of this function 
 *             to see if any assertion has failed during the code execution.
 *             Due to an assertion has failed and the endless loop runs with
 *             locked global interrupts there will be no life after the call ...
 */
/*****************************************************************************/
void BrsMainExceptionHandler(uint8 ErrorCode, uint8 ModuleCode, uint16 LineNumber)
{
#if defined (BRSMAIN_BRSMAINEXCEPTIONHANDLER_CALLOUT)
  BrsTestsuite_BrsMain_BrsMainExceptionHandler();
#endif

#if defined (BRS_ENABLE_SAFECTXSUPPORT)
  while(1)
  {}

#else
# if defined (_MICROSOFT_C_VTT_)
  char error[BRSMAIN_ERRORBUFFERSIZE];

  sprintf_s(error, BRSMAIN_ERRORBUFFERSIZE, "BrsMainExceptionHandler Code: [0x%x] ModuleCode: [0x%x] LineNumber: [0x%x]", ErrorCode, ModuleCode, LineNumber);

  CANoeAPI_WriteString(error);
  ShutdownOS(0);

# else
  volatile uint8 BrsMain_Continue;
  BrsMain_Continue = 0;
  DisableAllInterrupts();
  while (BrsMain_Continue == 0)
  {
    /* Set BrsMain_Continue to 1 to continue here.
     *  If the debugger is not able to show the stack properly, this mechanism can be used to find the
     *  source of the exception. */
  }
# endif /*_MICROSOFT_C_VTT_*/
#endif /*BRS_ENABLE_SAFECTXSUPPORT*/
}

/*****************************************************************************/
/**
 * @brief      This is just a wrapper to ease calls to
 *             BrsMainExceptionHandler() from assembler StartUpCode.
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     -
 * @context    This function will call BrsMainExceptionHandler() with the
 *             error code for reaching illegal assembler in StartUpCpde.
 */
/*****************************************************************************/
void BrsMainExceptionStartup(void)
{
  BrsMainExceptionHandler(kBrsIllegalAssemblerReached, BRSERROR_MODULE_BRSSTARTUP, (uint16)0);
}

/*****************************************************************************/
/**
 * @brief      Main function
 * @pre        -
 * @param[in]  -
 * @param[out] -
 * @return     always 0 as the function is not expected to return
 * @context    Called by the startup code
 */
/*****************************************************************************/
int main(void)
{

  Os_InitMemory();
  Os_Init();

  EcuM_Init(); /* never returns */
  return 0;
}

/***********************************************************************************************************************
 *  Additional UserCode can be placed here
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * UserDefined ExclusiveArea handling for CAN channels, according to AN-ISC-8-1149_ErrorHook_E_OS_DISABLED_INT.pdf
 **********************************************************************************************************************/
# if defined (CAN_30_MCAN_SW_MAJOR_VERSION)
  #define Can_DisableControllerInterrupts Can_30_Mcan_DisableControllerInterrupts
  #define Can_EnableControllerInterrupts Can_30_Mcan_EnableControllerInterrupts
# endif

// void SchM_Enter_Can_CAN_EXCLUSIVE_AREA_0(void)
// {
//   if (++BrsAsrApplCanInterruptLockCtr == 1u)
//   {
//     Can_DisableControllerInterrupts(0u);
//   }
// }

// void SchM_Exit_Can_CAN_EXCLUSIVE_AREA_0(void)
// {
//   if (--BrsAsrApplCanInterruptLockCtr == 0u)
//   {
//     Can_EnableControllerInterrupts(0u);
//   }
// }

// void SchM_Enter_Can_CAN_EXCLUSIVE_AREA_6(void)
// {
//   if (++BrsAsrApplCanInterruptLockCtr == 1u)
//   {
//     Can_DisableControllerInterrupts(0u);
//   }
// }

// void SchM_Exit_Can_CAN_EXCLUSIVE_AREA_6(void)
// {
//   if (--BrsAsrApplCanInterruptLockCtr == 0u)
//   {
//     Can_EnableControllerInterrupts(0u);
//   }
// }

// void SchM_Enter_CanIf_CANIF_EXCLUSIVE_AREA_0(void)
// {
//   if (++BrsAsrApplCanInterruptLockCtr == 1u)
//   {
//     Can_DisableControllerInterrupts(0u);
//   }
// }

// void SchM_Exit_CanIf_CANIF_EXCLUSIVE_AREA_0(void)
// {
//   if (--BrsAsrApplCanInterruptLockCtr == 0u)
//   {
//     Can_EnableControllerInterrupts(0u);
//   }
// }

// void SchM_Enter_CanSM_CANSM_EXCLUSIVE_AREA_4(void)
// {
//   if (++BrsAsrApplCanInterruptLockCtr == 1u)
//   {
//     Can_DisableControllerInterrupts(0u);
//   }
// }

// void SchM_Exit_CanSM_CANSM_EXCLUSIVE_AREA_4(void)
// {
//   if (--BrsAsrApplCanInterruptLockCtr == 0u)
//   {
//     Can_EnableControllerInterrupts(0u);
//   }
// }

// void SchM_Enter_CanSM_CANSM_EXCLUSIVE_AREA_1(void)
// {
//   /* Protects against task interruptions -> no locking needed, because all main functions are on same task */
// }

// void SchM_Exit_CanSM_CANSM_EXCLUSIVE_AREA_1(void)
// {
//   /* Protects against task interruptions -> no locking needed, because all main functions are on same task */
// }

void SchM_Enter_ComM_COMM_EXCLUSIVE_AREA_1(void)
{
  /* Protects against task interruptions -> no locking needed, because all main functions are on same task */
}

void SchM_Exit_ComM_COMM_EXCLUSIVE_AREA_1(void)
{
  /* Protects against task interruptions -> no locking needed, because all main functions are on same task */
}


void Appl_UnlockEndinit(void)
{
	Mcal_ResetENDINIT();

}
void Appl_LockEndinit(void)
{

	Mcal_SetENDINIT();

}
static volatile uint32 dummy;
void ApplCanWaitSetMCR(void)
{
	uint32 i;
	for(i = 0 ; i < 10000 ; i++)
	{
		dummy++;
	}

}
