/*
 * Do not modify this file; it is automatically generated from the template
 * linkcmd.xdt in the ti.platforms.simplelink package and will be overwritten.
 */

"C:\Users\johnny\Desktop\UartOverAir\configPkg\package\cfg\SimpleUART_prm3.orm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/mw/display/lib/display.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/mw/lcd/lib/lcd.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/mw/grlib/lib/grlib.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/drivers/pdm/lib/pdm_cc13xxware.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/drivers/lib/drivers_cc13xxware.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/drivers/lib/power_cc13xx_tirtos.arm3"
"C:\ti\tirtos_cc13xx_cc26xx_2_16_00_08\products\tidrivers_cc13xx_cc26xx_2_16_00_08\packages\ti\mw\fatfs\lib\release\ti.mw.fatfs.arm3"
"C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages/ti/drivers/ports/lib/tirtosport.arm3"
"C:\Users\johnny\Desktop\UartOverAir\src\sysbios\rom_sysbios.arm3"
"C:\ti\tirtos_cc13xx_cc26xx_2_16_00_08\products\bios_6_45_01_29\packages\iar\targets\arm\rts\lib\release\boot.arm3"


/* Content from xdc.services.global (null): */

/* Content from xdc (null): */

/* Content from xdc.corevers (null): */

/* Content from xdc.shelf (null): */

/* Content from xdc.services.spec (null): */

/* Content from xdc.services.intern.xsr (null): */

/* Content from xdc.services.intern.gen (null): */

/* Content from xdc.services.intern.cmd (null): */

/* Content from xdc.bld (null): */

/* Content from iar.targets.arm (null): */

/* Content from xdc.rov (null): */

/* Content from xdc.runtime (null): */

/* Content from iar.targets.arm.rts (): */

/* Content from ti.sysbios.interfaces (null): */

/* Content from ti.sysbios.family (null): */

/* Content from ti.sysbios.family.arm (ti/sysbios/family/arm/linkcmd.xdt): */

/* Content from xdc.services.getset (null): */

/* Content from ti.sysbios.rom (null): */

/* Content from ti.sysbios.rts (ti/sysbios/rts/linkcmd.xdt): */

/* Content from xdc.runtime.knl (null): */

/* Content from ti.catalog.arm.cortexm3 (null): */

/* Content from ti.catalog.peripherals.hdvicp2 (null): */

/* Content from ti.catalog (null): */

/* Content from ti.catalog.arm.peripherals.timers (null): */

/* Content from xdc.platform (null): */

/* Content from xdc.cfg (null): */

/* Content from ti.catalog.arm.cortexm4 (null): */

/* Content from ti.platforms.simplelink (null): */

/* Content from ti.sysbios.hal (null): */

/* Content from ti.sysbios.family.arm.cc26xx (null): */

/* Content from ti.sysbios.family.arm.m3 (ti/sysbios/family/arm/m3/linkcmd.xdt): */
--entry __iar_program_start
--keep __vector_table
--define_symbol ti_sysbios_family_arm_m3_Hwi_nvic=0xe000e000

/* Content from ti.sysbios.knl (null): */

/* Content from ti.sysbios (null): */

/* Content from ti.drivers.ports (null): */

/* Content from ti.mw.fatfs (null): */

/* Content from ti.sysbios.gates (null): */

/* Content from ti.sysbios.xdcruntime (null): */

/* Content from ti.sysbios.heaps (null): */

/* Content from ti.sysbios.rom.cortexm.cc13xx (C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/rom/cortexm/cc13xx/golden/CC13xx/CC13xx_link_iar.xdt): */

--keep xdc_runtime_Error_policy__C
--keep xdc_runtime_IModule_Interface__BASE__C
--keep xdc_runtime_Startup_lastFxns__C
--keep ti_sysbios_gates_GateMutex_Object__DESC__C
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_initDevice__I
--keep xdc_runtime_Startup_execImpl__C
--keep ti_sysbios_gates_GateMutex_Instance_State_sem__O
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getMaxTicks__E
--keep ti_sysbios_knl_Swi_Object__count__C
--keep ti_sysbios_knl_Idle_funcList__C
--keep ti_sysbios_family_arm_m3_Hwi_Object__PARAMS__C
--keep xdc_runtime_Text_isLoaded__C
--keep ti_sysbios_knl_Clock_Object__DESC__C
--keep ti_sysbios_knl_Mailbox_Instance_State_dataQue__O
--keep ti_sysbios_gates_GateMutex_Module__FXNS__C
--keep ti_sysbios_knl_Task_Module_State_inactiveQ__O
--keep ti_sysbios_family_arm_m3_Hwi_Module__id__C
--keep ti_sysbios_family_arm_cc26xx_Timer_Module__id__C
--keep ti_sysbios_knl_Mailbox_Object__table__C
--keep ti_sysbios_family_arm_m3_Hwi_Object__table__C
--keep ti_sysbios_knl_Swi_Object__DESC__C
--keep xdc_runtime_Text_charCnt__C
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_start__E
--keep ti_sysbios_heaps_HeapMem_Object__table__C
--keep xdc_runtime_Error_policyFxn__C
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCount64__E
--keep xdc_runtime_Startup_firstFxns__C
--keep ti_sysbios_knl_Swi_Object__PARAMS__C
--keep ti_sysbios_knl_Clock_serviceMargin__C
--keep xdc_runtime_Text_charTab__C
--keep ti_sysbios_rom_ROM_AONRTCCurrentCompareValueGet
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_TimestampProvider_get32__E
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCurrentTick__E
--keep ti_sysbios_family_arm_m3_TaskSupport_stackAlignment__C
--keep ti_sysbios_family_arm_m3_Hwi_NUM_INTERRUPTS__C
--keep xdc_runtime_Main_Module__diagsMask__C
--keep ti_sysbios_knl_Swi_Object__table__C
--keep xdc_runtime_Memory_Module__id__C
--keep ti_sysbios_knl_Task_Object__PARAMS__C
--keep ti_sysbios_gates_GateMutex_Object__PARAMS__C
--keep ti_sysbios_heaps_HeapMem_Module__gateObj__C
--keep ti_sysbios_family_arm_cc26xx_Timer_startupNeeded__C
--keep ti_sysbios_knl_Queue_Object__DESC__C
--keep ti_sysbios_knl_Task_Object__DESC__C
--keep xdc_runtime_Assert_E_assertFailed__C
--keep ti_sysbios_heaps_HeapMem_Object__PARAMS__C
--keep ti_sysbios_gates_GateHwi_Module__id__C
--keep ti_sysbios_gates_GateHwi_Object__PARAMS__C
--keep xdc_runtime_IHeap_Interface__BASE__C
--keep xdc_runtime_SysCallback_exitFxn__C
--keep ti_sysbios_heaps_HeapMem_Module__id__C
--keep ti_sysbios_family_arm_m3_Hwi_excHandlerFunc__C
--keep ti_sysbios_heaps_HeapMem_Module__FXNS__C
--keep xdc_runtime_System_maxAtexitHandlers__C
--keep ti_sysbios_knl_Queue_Object__count__C
--keep ti_sysbios_knl_Task_Object__table__C
--keep ti_sysbios_knl_Mailbox_Object__DESC__C
--keep ti_sysbios_family_arm_m3_Hwi_nullIsrFunc__C
--keep ti_sysbios_knl_Clock_tickMode__C
--keep ti_sysbios_gates_GateMutex_Module__id__C
--keep ti_sysbios_knl_Swi_numPriorities__C
--keep ti_sysbios_knl_Task_numConstructedTasks__C
--keep xdc_runtime_Startup_maxPasses__C
--keep ti_sysbios_rom_ROM_AONRTCEventClear
--keep ti_sysbios_knl_Task_initStackFlag__C
--keep xdc_runtime_Main_Module__diagsEnabled__C
--keep xdc_runtime_Main_Module__diagsIncluded__C
--keep xdc_runtime_System_abortFxn__C
--keep ti_sysbios_knl_Mailbox_Instance_State_dataSem__O
--keep ti_sysbios_gates_GateHwi_Module__FXNS__C
--keep ti_sysbios_hal_Hwi_Object__DESC__C
--keep ti_sysbios_family_arm_m3_Hwi_priGroup__C
--keep xdc_runtime_Error_E_memory__C
--keep ti_sysbios_family_arm_m3_Hwi_E_alreadyDefined__C
--keep ti_sysbios_knl_Mailbox_Instance_State_freeSem__O
--keep ti_sysbios_knl_Queue_Object__table__C
--keep ti_sysbios_knl_Semaphore_Object__PARAMS__C
--keep xdc_runtime_System_exitFxn__C
--keep ti_sysbios_knl_Clock_Object__PARAMS__C
--keep ti_sysbios_rom_ROM_AONRTCCompareValueSet
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setNextTick__E
--keep ti_sysbios_heaps_HeapMem_reqAlign__C
--keep xdc_runtime_Main_Module__id__C
--keep xdc_runtime_Startup_sfxnRts__C
--keep ti_sysbios_knl_Semaphore_Object__DESC__C
--keep ti_sysbios_gates_GateHwi_Object__DESC__C
--keep ti_sysbios_heaps_HeapMem_Object__count__C
--keep ti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__C
--keep ti_sysbios_knl_Mailbox_maxTypeAlign__C
--keep ti_sysbios_family_arm_cc26xx_TimestampProvider_useClockTimer__C
--keep ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_exit__E
--keep ti_sysbios_knl_Queue_Object__PARAMS__C
--keep ti_sysbios_knl_Task_allBlockedFunc__C
--keep ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_abort__E
--keep ti_sysbios_knl_Mailbox_Object__count__C
--keep xdc_runtime_Text_nameStatic__C
--keep ti_sysbios_rom_ROM_xdc_runtime_Startup_getState__I
--keep ti_sysbios_knl_Clock_Module_State_clockQ__O
--keep ti_sysbios_knl_Task_defaultStackSize__C
--keep xdc_runtime_IGateProvider_Interface__BASE__C
--keep ti_sysbios_family_arm_m3_Hwi_E_hwiLimitExceeded__C
--keep xdc_runtime_Startup_startModsFxn__C
--keep ti_sysbios_knl_Semaphore_Instance_State_pendQ__O
--keep ti_sysbios_family_arm_m3_Hwi_Object__DESC__C
--keep xdc_runtime_Text_nameEmpty__C
--keep ti_sysbios_family_arm_m3_Hwi_Object__count__C
--keep xdc_runtime_SysCallback_abortFxn__C
--keep ti_sysbios_knl_Task_defaultStackHeap__C
--keep ti_sysbios_family_arm_m3_Hwi_ccr__C
--keep ti_sysbios_knl_Mailbox_Object__PARAMS__C
--keep ti_sysbios_hal_Hwi_Object__PARAMS__C
--keep ti_sysbios_heaps_HeapMem_E_memory__C
--keep ti_sysbios_knl_Task_Object__count__C
--keep ti_sysbios_rom_ROM_AONRTCChannelEnable
--keep ti_sysbios_heaps_HeapMem_Object__DESC__C
--keep xdc_runtime_Text_nameUnknown__C
--keep xdc_runtime_Memory_defaultHeapInstance__C
--keep ti_sysbios_knl_Mailbox_Instance_State_freeQue__O
--keep ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setThreshold__I
--keep xdc_runtime_Startup_sfxnTab__C
--keep ti_sysbios_knl_Clock_Module__state__V
--keep ti_sysbios_family_arm_cc26xx_TimestampProvider_Module__state__V
--keep xdc_runtime_Startup_Module__state__V
--keep ti_sysbios_BIOS_Module__state__V
--keep ti_sysbios_knl_Swi_Module__state__V
--keep ti_sysbios_knl_Task_Module__state__V
--keep xdc_runtime_Memory_Module__state__V
--keep xdc_runtime_System_Module__state__V
--keep ti_sysbios_family_arm_m3_Hwi_Module__state__V
--keep ti_sysbios_family_arm_cc26xx_Timer_Module__state__V

--define_symbol memcpy=0x1001ca75
--define_symbol memset=0x1001ca87
--define_symbol ti_sysbios_rom_cortexm_cc13xx_CC13xx_getRevision__E=0x1001ca97
--define_symbol ti_sysbios_knl_Queue_get__E=0x1001bf0d
--define_symbol ti_sysbios_knl_Swi_enabled__E=0x1001c0e1
--define_symbol ti_sysbios_knl_Clock_scheduleNextTick__E=0x1001beb9
--define_symbol ti_sysbios_knl_Swi_runLoop__I=0x1001b0d5
--define_symbol ti_sysbios_knl_Clock_getTicks__E=0x1001b6d5
--define_symbol ti_sysbios_gates_GateMutex_Object__destruct__S=0x1001bc45
--define_symbol ti_sysbios_knl_Queue_enqueue__E=0x1001c1b1
--define_symbol ti_sysbios_knl_Queue_put__E=0x1001bf29
--define_symbol ti_sysbios_family_arm_m3_Hwi_Object__create__S=0x1001aab1
--define_symbol ti_sysbios_gates_GateHwi_Instance_init__E=0x1001b45f
--define_symbol ti_sysbios_hal_Hwi_Instance_finalize__E=0x1001c1ed
--define_symbol ti_sysbios_BIOS_RtsGateProxy_leave__E=0x1001c249
--define_symbol ti_sysbios_heaps_HeapMem_Object__create__S=0x1001b841
--define_symbol xdc_runtime_Error_raiseX__E=0x1001c765
--define_symbol ti_sysbios_knl_Semaphore_construct=0x1001b705
--define_symbol ti_sysbios_knl_Clock_Object__destruct__S=0x1001bce5
--define_symbol ti_sysbios_knl_Clock_TimerProxy_getMaxTicks__E=0x1001c0a1
--define_symbol ti_sysbios_knl_Swi_Object__destruct__S=0x1001bf45
--define_symbol ti_sysbios_family_arm_cc26xx_TimestampProvider_getFreq__E=0x1001c12b
--define_symbol ti_sysbios_gates_GateMutex_Handle__label__S=0x1001bc25
--define_symbol ti_sysbios_knl_Mailbox_delete=0x1001c2c1
--define_symbol ti_sysbios_knl_Semaphore_destruct=0x1001c2dd
--define_symbol ti_sysbios_BIOS_RtsGateProxy_enter__E=0x1001c245
--define_symbol ti_sysbios_knl_Task_processVitalTaskFlag__I=0x1001b735
--define_symbol ti_sysbios_knl_Mailbox_create=0x1001abdd
--define_symbol xdc_runtime_Core_deleteObject__I=0x1001c4f5
--define_symbol ti_sysbios_knl_Queue_delete=0x1001c2c9
--define_symbol ti_sysbios_family_arm_m3_Hwi_doSwiRestore__I=0x1001c1f7
--define_symbol xdc_runtime_System_atexit__E=0x1001c67d
--define_symbol ti_sysbios_gates_GateMutex_Params__init__S=0x1001c175
--define_symbol ti_sysbios_knl_Clock_getTimerHandle__E=0x1001c18d
--define_symbol ti_sysbios_knl_Task_enable__E=0x1001c23f
--define_symbol ti_sysbios_knl_Clock_TimerProxy_getExpiredTicks__E=0x1001c2a9
--define_symbol ti_sysbios_knl_Queue_Object__destruct__S=0x1001bef1
--define_symbol ti_sysbios_knl_Clock_Object__delete__S=0x1001bad9
--define_symbol ti_sysbios_gates_GateMutex_delete=0x1001c269
--define_symbol ti_sysbios_heaps_HeapMem_restore__E=0x1001c101
--define_symbol ti_sysbios_knl_Swi_create=0x1001ad05
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_leave__E=0x1001c29d
--define_symbol ti_sysbios_knl_Semaphore_pend__E=0x1001a001
--define_symbol ti_sysbios_knl_Mailbox_Instance_finalize__E=0x1001a8f9
--define_symbol xdc_runtime_Startup_startMods__I=0x1001c309
--define_symbol ti_sysbios_heaps_HeapMem_init__I=0x1001b541
--define_symbol ti_sysbios_knl_Swi_Object__delete__S=0x1001bda5
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_enableInterrupt__E=0x1001c281
--define_symbol ti_sysbios_knl_Clock_removeI__E=0x1001c2b9
--define_symbol xdc_runtime_System_abort__E=0x1001c745
--define_symbol ti_sysbios_family_arm_m3_Hwi_dispatchC__I=0x1001aa49
--define_symbol ti_sysbios_knl_Swi_construct=0x1001b085
--define_symbol ti_sysbios_knl_Task_sleepTimeout__I=0x1001bf7d
--define_symbol ti_sysbios_knl_Queue_remove__E=0x1001c11d
--define_symbol ti_sysbios_knl_Semaphore_Instance_finalize__E=0x1001c0c1
--define_symbol ti_sysbios_gates_GateMutex_destruct=0x1001c26d
--define_symbol ti_sysbios_knl_Task_SupportProxy_Module__startupDone__S=0x1001c2ed
--define_symbol ti_sysbios_knl_Queue_Object__delete__S=0x1001bd45
--define_symbol ti_sysbios_knl_Mailbox_Object__get__S=0x1001b899
--define_symbol ti_sysbios_family_arm_m3_Hwi_Instance_init__E=0x1001a3c5
--define_symbol ti_sysbios_knl_Clock_delete=0x1001c2b1
--define_symbol ti_sysbios_knl_Clock_walkQueueDynamic__E=0x1001a809
--define_symbol ti_sysbios_knl_Mailbox_Object__destruct__S=0x1001bed5
--define_symbol ti_sysbios_knl_Mailbox_post__E=0x1001a58d
--define_symbol ti_sysbios_knl_Clock_Instance_init__E=0x1001b509
--define_symbol ti_sysbios_knl_Task_allBlockedFunction__I=0x1001b361
--define_symbol ti_sysbios_knl_Task_postInit__I=0x1001a611
--define_symbol ti_sysbios_knl_Task_enter__I=0x1001be05
--define_symbol ti_sysbios_hal_Hwi_switchFromBootStack__E=0x1001c295
--define_symbol ti_sysbios_knl_Semaphore_Object__destruct__S=0x1001bd85
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_Object__create__S=0x1001c081
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_postInit__I=0x1001b765
--define_symbol ti_sysbios_knl_Swi_Module_startup__E=0x1001c233
--define_symbol ti_sysbios_gates_GateMutex_Instance_finalize__E=0x1001c071
--define_symbol xdc_runtime_Core_assignParams__I=0x1001c5ed
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_switchFromBootStack__E=0x1001c28d
--define_symbol ti_sysbios_knl_Swi_post__E=0x1001b31d
--define_symbol ti_sysbios_hal_Hwi_initStack=0x1001b815
--define_symbol xdc_runtime_Memory_alloc__E=0x1001c485
--define_symbol ti_sysbios_knl_Queue_next__E=0x1001c2d5
--define_symbol ti_sysbios_knl_Clock_Instance_finalize__E=0x1001bfc9
--define_symbol ti_sysbios_knl_Queue_elemClear__E=0x1001c22d
--define_symbol ti_sysbios_knl_Clock_Params__init__S=0x1001c181
--define_symbol ti_sysbios_knl_Task_Instance_init__E=0x1001a259
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_startup__E=0x1001c289
--define_symbol ti_sysbios_knl_Task_self__E=0x1001c1e1
--define_symbol ti_sysbios_knl_Task_startup__E=0x1001c305
--define_symbol ti_sysbios_gates_GateHwi_Object__delete__S=0x1001bc05
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_startup__E=0x1001b791
--define_symbol xdc_runtime_Memory_free__E=0x1001c7fd
--define_symbol ti_sysbios_hal_Hwi_delete=0x1001c291
--define_symbol ti_sysbios_knl_Queue_Instance_init__E=0x1001c227
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_Module_startup__E=0x1001ba6d
--define_symbol xdc_runtime_Assert_raise__I=0x1001c5a5
--define_symbol ti_sysbios_hal_Hwi_create=0x1001ab79
--define_symbol ti_sysbios_knl_Task_destruct=0x1001c301
--define_symbol ti_sysbios_hal_Hwi_Module_startup__E=0x1001c01d
--define_symbol ti_sysbios_family_arm_m3_Hwi_excHandler__I=0x1001bbc5
--define_symbol xdc_runtime_Core_destructObject__I=0x1001c79d
--define_symbol ti_sysbios_knl_Swi_disable__E=0x1001c0d1
--define_symbol ti_sysbios_BIOS_setThreadType__E=0x1001c041
--define_symbol ti_sysbios_knl_Task_disable__E=0x1001c0f1
--define_symbol ti_sysbios_knl_Swi_Instance_init__E=0x1001aca5
--define_symbol ti_sysbios_knl_Semaphore_pendTimeout__I=0x1001b949
--define_symbol ti_sysbios_knl_Clock_create=0x1001b3e9
--define_symbol ti_sysbios_knl_Idle_loop__E=0x1001c207
--define_symbol ti_sysbios_gates_GateHwi_leave__E=0x1001c21b
--define_symbol ti_sysbios_family_arm_m3_Hwi_enableInterrupt__E=0x1001b295
--define_symbol ti_sysbios_knl_Semaphore_Params__init__S=0x1001c1bd
--define_symbol ti_sysbios_knl_Task_unblock__E=0x1001bfb1
--define_symbol ti_sysbios_knl_Swi_destruct=0x1001c2e5
--define_symbol ti_sysbios_BIOS_getCpuFreq__E=0x1001bff5
--define_symbol xdc_runtime_Memory_calloc__E=0x1001c849
--define_symbol ti_sysbios_family_arm_m3_Hwi_startup__E=0x1001c1ff
--define_symbol xdc_runtime_SysCallback_exit__E=0x1001c81d
--define_symbol ti_sysbios_knl_Queue_empty__E=0x1001c10f
--define_symbol ti_sysbios_knl_Clock_logTick__E=0x1001be63
--define_symbol ti_sysbios_knl_Task_yield__E=0x1001b3a5
--define_symbol ti_sysbios_knl_Task_SupportProxy_getStackAlignment__E=0x1001c2f1
--define_symbol ti_sysbios_family_arm_m3_Hwi_create=0x1001ab15
--define_symbol xdc_runtime_Timestamp_SupportProxy_get32__E=0x1001c82d
--define_symbol ti_sysbios_family_arm_m3_Hwi_destruct=0x1001c25d
--define_symbol ti_sysbios_family_arm_m3_Hwi_doTaskRestore__I=0x1001c20f
--define_symbol ti_sysbios_knl_Swi_run__I=0x1001afd9
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_Module__startupDone__S=0x1001bb45
--define_symbol xdc_runtime_Core_createObject__I=0x1001c401
--define_symbol ti_sysbios_knl_Queue_create=0x1001b91d
--define_symbol ti_sysbios_hal_Hwi_Object__delete__S=0x1001bc65
--define_symbol ti_sysbios_knl_Clock_construct=0x1001b575
--define_symbol xdc_runtime_System_abortSpin__E=0x1001c895
--define_symbol ti_sysbios_family_arm_m3_Hwi_Object__destruct__S=0x1001be65
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_delete=0x1001c279
--define_symbol ti_sysbios_gates_GateMutex_Object__create__S=0x1001b645
--define_symbol ti_sysbios_family_arm_m3_Hwi_getStackInfo__E=0x1001b461
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_enter__E=0x1001c299
--define_symbol ti_sysbios_knl_Semaphore_post__E=0x1001ac41
--define_symbol ti_sysbios_knl_Task_exit__E=0x1001b425
--define_symbol ti_sysbios_heaps_HeapMem_Instance_init__E=0x1001b4d1
--define_symbol ti_sysbios_knl_Swi_restore__E=0x1001b5dd
--define_symbol ti_sysbios_knl_Task_startCore__E=0x1001ae75
--define_symbol ti_sysbios_knl_Semaphore_create=0x1001b5a9
--define_symbol ti_sysbios_gates_GateHwi_enter__E=0x1001c169
--define_symbol ti_sysbios_knl_Task_blockI__E=0x1001b611
--define_symbol ti_sysbios_heaps_HeapMem_free__E=0x1001a791
--define_symbol ti_sysbios_knl_Task_Object__destruct__S=0x1001bf61
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_periodicStub__E=0x1001a465
--define_symbol ti_sysbios_hal_Hwi_Instance_init__E=0x1001b9f5
--define_symbol ti_sysbios_gates_GateHwi_query__E=0x1001c265
--define_symbol xdc_runtime_System_processAtExit__E=0x1001c6b5
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_setPeriod__E=0x1001c255
--define_symbol xdc_runtime_Error_init__E=0x1001c83d
--define_symbol ti_sysbios_knl_Semaphore_Instance_init__E=0x1001bafd
--define_symbol ti_sysbios_knl_Queue_head__E=0x1001c2d1
--define_symbol xdc_runtime_Error_check__E=0x1001c7b5
--define_symbol xdc_runtime_Error_policySpin__E=0x1001c483
--define_symbol ti_sysbios_gates_GateMutex_create=0x1001b675
--define_symbol xdc_runtime_Gate_leaveSystem__E=0x1001c875
--define_symbol ti_sysbios_knl_Swi_restoreHwi__E=0x1001ad61
--define_symbol ti_sysbios_knl_Task_sleep__E=0x1001a9d9
--define_symbol ti_sysbios_knl_Task_create=0x1001ae19
--define_symbol ti_sysbios_knl_Mailbox_Params__init__S=0x1001c199
--define_symbol ti_sysbios_knl_Task_restoreHwi__E=0x1001be25
--define_symbol ti_sysbios_knl_Mailbox_postInit__I=0x1001b8c5
--define_symbol ti_sysbios_knl_Task_delete=0x1001c2fd
--define_symbol ti_sysbios_heaps_HeapMem_isBlocking__E=0x1001c2a1
--define_symbol ti_sysbios_knl_Clock_startI__E=0x1001a715
--define_symbol ti_sysbios_knl_Clock_start__E=0x1001bfdf
--define_symbol ti_sysbios_family_arm_m3_Hwi_Object__delete__S=0x1001bba5
--define_symbol ti_sysbios_knl_Clock_TimerProxy_getPeriod__E=0x1001c2ad
--define_symbol ti_sysbios_knl_Task_SupportProxy_start__E=0x1001c2f5
--define_symbol ti_sysbios_heaps_HeapMem_Handle__label__S=0x1001bc85
--define_symbol ti_sysbios_family_arm_m3_Hwi_delete=0x1001c259
--define_symbol ti_sysbios_knl_Semaphore_Object__delete__S=0x1001bb21
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_getStackInfo__E=0x1001c285
--define_symbol ti_sysbios_knl_Idle_run__E=0x1001bd05
--define_symbol ti_sysbios_knl_Swi_delete=0x1001c2e1
--define_symbol xdc_runtime_Memory_valloc__E=0x1001c781
--define_symbol ti_sysbios_knl_Mailbox_Object__delete__S=0x1001bd25
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_start__E=0x1001b499
--define_symbol ti_sysbios_family_arm_m3_Hwi_Module__startupDone__S=0x1001bb85
--define_symbol ti_sysbios_knl_Swi_startup__E=0x1001c239
--define_symbol ti_sysbios_knl_Task_schedule__I=0x1001b175
--define_symbol ti_sysbios_gates_GateMutex_leave__E=0x1001bf99
--define_symbol ti_sysbios_heaps_HeapMem_Object__delete__S=0x1001bca5
--define_symbol ti_sysbios_knl_Clock_TimerProxy_setNextTick__E=0x1001c0b1
--define_symbol ti_sysbios_knl_Swi_Object__get__S=0x1001b975
--define_symbol ti_sysbios_knl_Task_restore__E=0x1001ba45
--define_symbol xdc_runtime_Memory_HeapProxy_alloc__E=0x1001c885
--define_symbol ti_sysbios_gates_GateHwi_Object__create__S=0x1001ba91
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_disableInterrupt__E=0x1001c27d
--define_symbol ti_sysbios_BIOS_start__E=0x1001c051
--define_symbol ti_sysbios_BIOS_exit__E=0x1001c031
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_getStackAlignment__E=0x1001c15d
--define_symbol xdc_runtime_SysCallback_abort__E=0x1001c80d
--define_symbol ti_sysbios_knl_Queue_destruct=0x1001c2cd
--define_symbol ti_sysbios_family_arm_m3_Hwi_postInit__I=0x1001a315
--define_symbol ti_sysbios_gates_GateMutex_Instance_init__E=0x1001be9d
--define_symbol ti_sysbios_knl_Task_Instance_finalize__E=0x1001a4f9
--define_symbol ti_sysbios_knl_Clock_TimerProxy_getCurrentTick__E=0x1001c091
--define_symbol ti_sysbios_family_arm_m3_Hwi_disableFxn__E=0x1001c145
--define_symbol xdc_runtime_Memory_HeapProxy_free__E=0x1001c889
--define_symbol ti_sysbios_knl_Mailbox_Module_startup__E=0x1001b2d9
--define_symbol ti_sysbios_knl_Task_Object__delete__S=0x1001bdc5
--define_symbol ti_sysbios_gates_GateHwi_Handle__label__S=0x1001bbe5
--define_symbol xdc_runtime_Text_ropeText__E=0x1001c7e5
--define_symbol ti_sysbios_knl_Clock_destruct=0x1001c2b5
--define_symbol ti_sysbios_knl_Queue_construct=0x1001b8f1
--define_symbol ti_sysbios_family_arm_m3_Hwi_switchFromBootStack__E=0x1001c009
--define_symbol ti_sysbios_heaps_HeapMem_Object__get__S=0x1001b86d
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_create=0x1001be45
--define_symbol ti_sysbios_gates_GateMutex_query__E=0x1001c271
--define_symbol ti_sysbios_knl_Swi_schedule__I=0x1001b031
--define_symbol ti_sysbios_knl_Task_Params__init__S=0x1001c1d5
--define_symbol ti_sysbios_family_arm_m3_Hwi_Params__init__S=0x1001c139
--define_symbol ti_sysbios_family_arm_m3_Hwi_plug__E=0x1001c061
--define_symbol xdc_runtime_System_exitSpin__E=0x1001c897
--define_symbol ti_sysbios_gates_GateMutex_construct=0x1001b7e9
--define_symbol xdc_runtime_System_Module_GateProxy_leave__E=0x1001c891
--define_symbol ti_sysbios_knl_Mailbox_pend__E=0x1001a969
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_Module__startupDone__S=0x1001c261
--define_symbol xdc_runtime_Core_assignLabel__I=0x1001c6e9
--define_symbol xdc_runtime_System_Module_GateProxy_enter__E=0x1001c88d
--define_symbol xdc_runtime_System_exit__E=0x1001c7cd
--define_symbol ti_sysbios_knl_Swi_Params__init__S=0x1001c1c9
--define_symbol ti_sysbios_knl_Clock_workFunc__E=0x1001a881
--define_symbol ti_sysbios_family_arm_m3_Hwi_restoreFxn__E=0x1001c215
--define_symbol ti_sysbios_family_arm_cc26xx_TimestampProvider_Module_startup__E=0x1001bb65
--define_symbol ti_sysbios_knl_Semaphore_delete=0x1001c2d9
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_getPeriod__E=0x1001c251
--define_symbol ti_sysbios_family_arm_m3_Hwi_initNVIC__E=0x1001a695
--define_symbol ti_sysbios_knl_Clock_addI__E=0x1001ba1d
--define_symbol ti_sysbios_family_arm_m3_Hwi_Instance_finalize__E=0x1001aed1
--define_symbol ti_sysbios_heaps_HeapMem_alloc__E=0x1001a195
--define_symbol ti_sysbios_knl_Task_unblockI__E=0x1001b9a1
--define_symbol ti_sysbios_knl_Swi_Instance_finalize__E=0x1001bf97
--define_symbol ti_sysbios_family_arm_m3_Hwi_disableInterrupt__E=0x1001b251
--define_symbol ti_sysbios_family_arm_m3_Hwi_enableFxn__E=0x1001c151
--define_symbol xdc_runtime_Gate_enterSystem__E=0x1001c881
--define_symbol ti_sysbios_gates_GateMutex_Object__delete__S=0x1001bab5
--define_symbol ti_sysbios_family_arm_cc26xx_TimestampProvider_get64__E=0x1001b9cd
--define_symbol ti_sysbios_knl_Mailbox_Instance_init__E=0x1001a0cd
--define_symbol xdc_runtime_Text_cordText__E=0x1001c719
--define_symbol xdc_runtime_Startup_exec__E=0x1001c555
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_Module__startupDone__S=0x1001c275
--define_symbol ti_sysbios_heaps_HeapMem_getStats__E=0x1001b209
--define_symbol ti_sysbios_knl_Task_SupportProxy_swap__E=0x1001c2f9
--define_symbol xdc_runtime_Memory_getMaxDefaultTypeAlign__E=0x1001c855
--define_symbol ti_sysbios_knl_Task_Object__get__S=0x1001bde5
--define_symbol ti_sysbios_family_arm_m3_Hwi_construct=0x1001af29
--define_symbol ti_sysbios_knl_Clock_TimerProxy_Module__startupDone__S=0x1001c2a5
--define_symbol ti_sysbios_knl_Clock_Module_startup__E=0x1001bcc5
--define_symbol ti_sysbios_knl_Mailbox_construct=0x1001af81
--define_symbol ti_sysbios_knl_Task_construct=0x1001b125
--define_symbol xdc_runtime_Core_constructObject__I=0x1001c635
--define_symbol ti_sysbios_knl_Queue_dequeue__E=0x1001c1a5
--define_symbol ti_sysbios_knl_Task_Module_startup__E=0x1001adbd
--define_symbol ti_sysbios_family_arm_cc26xx_Timer_getExpiredTicks__E=0x1001c24d
--define_symbol ti_sysbios_family_arm_m3_Hwi_Object__get__S=0x1001b7bd
--define_symbol ti_sysbios_knl_Mailbox_destruct=0x1001c2c5
--define_symbol xdc_runtime_System_Module_startup__E=0x1001c87b
--define_symbol ti_sysbios_knl_Swi_postInit__I=0x1001c2e9
--define_symbol ti_sysbios_family_arm_m3_Hwi_Module_startup__E=0x1001b1c1
--define_symbol ti_sysbios_gates_GateMutex_enter__E=0x1001b6a5
--define_symbol ti_sysbios_family_arm_m3_Hwi_setPriority__E=0x1001be81
--define_symbol ti_sysbios_knl_Queue_Object__get__S=0x1001bd65
--define_symbol ti_sysbios_knl_Clock_setTimeout__E=0x1001c2bd
--define_symbol ti_sysbios_knl_Task_swapReturn=0x1001ca6d
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_glue=0x1001ca55
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_buildTaskStack=0x1001c969
--define_symbol ti_sysbios_family_arm_m3_TaskSupport_swap__E=0x1001ca65
--define_symbol ti_sysbios_family_arm_m3_Hwi_excHandlerAsm__I=0x1001c9fd
--define_symbol ti_sysbios_family_arm_m3_Hwi_return=0x1001ca53
--define_symbol ti_sysbios_family_arm_m3_Hwi_pendSV__I=0x1001ca3b
--define_symbol ti_sysbios_family_arm_m3_Hwi_dispatch__I=0x1001c899
--define_symbol ti_sysbios_family_xxx_Hwi_switchAndRunFunc=0x1001ca1d
--define_symbol ti_sysbios_family_arm_m3_Hwi_initStacks__E=0x1001c9b5
--define_symbol ti_sysbios_BIOS_RtsGateProxy_Object__delete__S=0x1001bab5
--define_symbol ti_sysbios_BIOS_RtsGateProxy_Params__init__S=0x1001c175
--define_symbol ti_sysbios_BIOS_RtsGateProxy_Handle__label__S=0x1001bc25
--define_symbol ti_sysbios_BIOS_RtsGateProxy_query__E=0x1001c271
--define_symbol ti_sysbios_knl_Clock_TimerProxy_startup__E=0x1001b791
--define_symbol ti_sysbios_hal_Hwi_disableInterrupt__E=0x1001c27d
--define_symbol ti_sysbios_hal_Hwi_enableInterrupt__E=0x1001c281
--define_symbol ti_sysbios_hal_Hwi_getStackInfo__E=0x1001c285
--define_symbol ti_sysbios_hal_Hwi_startup__E=0x1001c289
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_Object__delete__S=0x1001bba5
--define_symbol ti_sysbios_hal_Hwi_HwiProxy_Params__init__S=0x1001c139
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_query__E=0x1001c271
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_Object__delete__S=0x1001bab5
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_Params__init__S=0x1001c175
--define_symbol ti_sysbios_heaps_HeapMem_Module_GateProxy_Handle__label__S=0x1001bc25
--define_symbol xdc_runtime_Timestamp_SupportProxy_get64__E=0x1001b9cd
--define_symbol xdc_runtime_Timestamp_SupportProxy_getFreq__E=0x1001c12b
--define_symbol xdc_runtime_Timestamp_get32__E=0x1001c82d
--define_symbol xdc_runtime_Timestamp_get64__E=0x1001b9cd
--define_symbol xdc_runtime_Timestamp_getFreq__E=0x1001c12b
--define_symbol xdc_runtime_Memory_HeapProxy_Object__delete__S=0x1001bca5
--define_symbol xdc_runtime_Memory_HeapProxy_Handle__label__S=0x1001bc85
--define_symbol xdc_runtime_System_Module_GateProxy_Object__delete__S=0x1001bc05
--define_symbol xdc_runtime_System_Module_GateProxy_Handle__label__S=0x1001bbe5
--define_symbol xdc_runtime_System_Module_GateProxy_query__E=0x1001c265

/* Content from ti.sysbios.utils (null): */

/* Content from ti.drivers (null): */

/* Content from ti.drivers.pdm (null): */

/* Content from ti.mw.grlib (null): */

/* Content from ti.mw.lcd (null): */

/* Content from ti.mw.display (null): */

/* Content from ti.mw (null): */

/* Content from configPkg (null): */

/* Content from xdc.services.io (null): */

/* Content from ti.targets (null): */


--define_symbol xdc_runtime_Startup__EXECFXN__C=1
--define_symbol xdc_runtime_Startup__RESETFXN__C=1


--config_search C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/rom/cortexm/cc13xx/golden/CC13xx/
--config_def USE_TIRTOS_ROM=1

--keep __ASM__
--keep __PLAT__
--keep __ISA__
--keep __TARG__
--keep __TRDR__