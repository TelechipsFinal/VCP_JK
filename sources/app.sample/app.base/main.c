// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>

#include <adc.h>

/* ========================================================================== */
/* ADC & DMA 구조체 정의                            */
/* ========================================================================== */

// --- ADC 관련 구조체 ---
typedef struct ADCCmd { uint32 acCmd:16; uint32 acNofAin:4; uint32 acCnvCnt:5; uint32 :6; uint32 acCnvDone:1; } ADCCmd_t;
typedef union { uint32 cuNreg; ADCCmd_t cuBreg; } ADCCmdUnion_t;
typedef struct { uint32 clDiv:8; uint32 clIrqEn:1; uint32 clReqEn:1; uint32 :22; } ADCTestAdcclk_t;
typedef union { uint32 nReg; ADCTestAdcclk_t bReg; } ADCTestAdcclkU_t;

typedef struct ADCTest {
    ADCCmdUnion_t atAdcCmd; uint32 atAdcClr; ADCTestAdcclkU_t atAdcClk; uint32 atReserved[13];
    uint32 atAdcTime; uint32 atReserved2[15];
    uint32 atAdcAin00; uint32 atAdcAin01; uint32 atAdcAin02; uint32 atAdcAin03;
    uint32 atAdcAin04; uint32 atAdcAin05; uint32 atAdcAin06; uint32 atAdcAin07;
    uint32 atAdcAin08; uint32 atAdcAin09; uint32 atAdcAin10; uint32 atAdcAin11;
    uint32 atAdcAin12; uint32 atAdcAin13; uint32 atAdcAin14; uint32 atAdcAin15;
} ADCTest_t;

// --- DMA 채널 제어 구조체 ---
typedef struct { 
    uint32 scTransferSize:12; uint32 scSrcBurstSize:3; uint32 scDestBurstSize:3;
    uint32 scSrcWidth:3; uint32 scDestWidth:3; uint32 scSrcBus:1; uint32 scDestBus:1;
    uint32 scSrcIncrement:1; uint32 scDestIncrement:1; uint32 scProtection:3; uint32 scInterruptEnable:1;
} ADCDmaSchCtrl_t;

typedef union { uint32 rlNreg; ADCDmaSchCtrl_t rlBreg; } ADCUchCtrol_t;

typedef struct { 
    uint32 cfChEnable:1; uint32 cfSrcPeri:5; uint32 cfDstPeri:5; uint32 cfFlowCtrl:3;
    uint32 cfErrIntMask:1; uint32 cfIntMask:1; uint32 cfBusLock:1; uint32 cfActive:1;
    uint32 cfHalt:1; uint32 cfReserved:13;
} ADCDmaSchCfg_t;

typedef union { uint32 cfNreg; ADCDmaSchCfg_t cfBreg; } ADCUchCfg_t;

typedef struct { uint32 slNextLliBus:1; uint32 slreserved:1; uint32 slNextLli:30; } ADCDmaSchLli_t;
typedef union { uint32 dlNreg; ADCDmaSchLli_t dlBreg; } ADCDmaUlli_t;

typedef volatile struct {
    uint32          xcSrcAddrR;
    uint32          xcDestAddr;
    ADCDmaUlli_t    xcLli;
    ADCUchCtrol_t   xcCtrl;
    ADCUchCfg_t     xcCfg;
    uint32          xcReserved[3];
} ADCDmaxChannel_t;

// --- DMA 포트(컨트롤러) 구조체 (인터럽트 레지스터 포함) ---
typedef union { uint32 duNreg; uint32 duBreg; } ADCDmaUch_t; // 간소화된 Union

typedef volatile struct {
    ADCDmaUch_t     poIrqStatus;        // 0x000
    ADCDmaUch_t     poIrqItcStatus;     // 0x004 (오류 해결 핵심)
    ADCDmaUch_t     poIrqItcClear;      // 0x008 (오류 해결 핵심)
    ADCDmaUch_t     poIrqErrStatus;     // 0x00C
    ADCDmaUch_t     poIrqErrClear;      // 0x010
    ADCDmaUch_t     poRawIrqItcStatus;  // 0x014
    ADCDmaUch_t     poRawIrqErrStatus;  // 0x018
    ADCDmaUch_t     poEnabledChannel;   // 0x01C
    uint32          poSwReq[4];         // 0x020 ~ 0x02C
    union { uint32 dcNreg; uint32 dcBreg; } poConfig; // 0x030
    uint32          poSync;             // 0x034
    
    uint32          reserved[50];       // 0x038 ~ 0x0FC (Padding)
    
    ADCDmaxChannel_t poDmaChannel[2];   // 0x100 ~
} ADCDmaxPort_t;

#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    #include <spi_eccp.h>
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    #include <power_app.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_KEY == 1)
    #include <key.h>
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    #include <console.h>
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    #include <can_demo.h>
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    #include <idle.h>
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )
    #include <spi_led.h>
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    #include "fwupdate.h"
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    #include "fwupdate.h"
#endif

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;

/* ========================================================================== */
/* 매크로 및 전역 변수                              */
/* ========================================================================== */
#define ADC_DMA_TEST_REG(N)    ((volatile ADCDmaxPort_t  *)(0xA0800000UL + (0x10000 * (N))))
#define ADC_TEST_REG(N)        ((volatile ADCTest_t      *)((0xA0080000UL) + ((N) * 0x10000UL)))
#define ADC_DMA_BUF_SIZE       16

static uint32 gMicAdcTaskID = 0;
static uint32 gMicAdcTaskStk[ACFG_TASK_MEDIUM_STK_SIZE];
// 캐시 라인 정렬 (32바이트)
static uint32 gAdcDmaBuffer[ADC_DMA_BUF_SIZE] __attribute__((aligned(32)));

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask
(
    void *                              pArg
);

static void AppTaskCreate
(
    void
);

static void DisplayAliveLog
(
    void
);

static void DisplayOTPInfo
(
    void
);

static void Mic_ADC_Task(void * pArg);
static void Configure_ADC_DMA(void);
static void Local_CleanInvalidateDCache(uint32 addr, uint32 size);

/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/
void cmain (void)
{
    static uint32           AppTaskStartID = 0;
    static uint32           AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t            err;
    SALMcuVersionInfo_t     versionInfo = {0,0,0,0};

    (void)SAL_Init();

    BSP_PreInit(); /* Initialize basic BSP functions */

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    (void)CAN_DemoInitialize();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

    BSP_Init(); /* Initialize BSP functions */

    (void)SAL_GetVersion(&versionInfo);
    mcu_printf("\n===============================\n");
    mcu_printf("    MCU BSP Version: V%d.%d.%d\n",
           versionInfo.viMajorVersion,
           versionInfo.viMinorVersion,
           versionInfo.viPatchVersion);
    mcu_printf("-------------------------------\n");
    DisplayOTPInfo();
    mcu_printf("===============================\n\n");

    // create the first app task...
    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                         (const uint8 *)"App Task Start",
                         (SALTaskFunc) &Main_StartTask,
                         &AppTaskStartStk[0],
                         ACFG_TASK_MEDIUM_STK_SIZE,
                         SAL_PRIO_APP_CFG,
                         NULL);

    if (err == SAL_RET_SUCCESS)
    {
        // start woring os.... never return from this function
        (void)SAL_OsStart();
    }
}

/*
***************************************************************************************************
*                                          Main_StartTask
*
* This is an example of a startup task.
*
* Notes
*   As mentioned in the book's text, you MUST initialize the ticker only once multitasking has
*   started.
*
*   1) The first line of code is used to prevent a compiler warning because 'pArg' is not used.
*      The compiler should not generate any code for this statement.
*
***************************************************************************************************
*/
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    /* Service Init*/

    /* Create application tasks */
    AppTaskCreate();

    while (1)
    {  /* Task body, always written as an infinite loop.       */
        DisplayAliveLog();
        //mcu_printf("\n MCU Idle !!!");
        (void)SAL_TaskSleep(5000);
    }
}

static void AppTaskCreate(void)
{
#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    ECCP_InitSPIManager();
#endif  
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    POWER_APP_StartDemo();
#endif

  
#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    CreateConsoleTask();
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_KEY == 1 )
    KEY_AppCreate();
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    CAN_DemoCreateApp();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    CreateFWUDTask();
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    CreateFWUDTask();
#endif

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    IDLE_CreateTask();
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1)
    SPILED_CreateAppTask();
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

/* ADC 마이크 태스크 생성 */
(void)SAL_TaskCreate(&gMicAdcTaskID,
                         (const uint8 *)"ADC Mic Task",
                         (SALTaskFunc) &Mic_ADC_Task,
                         &gMicAdcTaskStk[0],
                         ACFG_TASK_MEDIUM_STK_SIZE,
                         SAL_PRIO_APP_CFG + 1,
                         NULL);

}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U)
    {
        mcu_printf("\n %d", gALiveCount);

        gALiveCount++;

        if(gALiveCount >= MAIN_UINT_MAX_NUM)
        {
            gALiveCount = 0;
        }
    }
    else
    {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR  0xA1011800U
#define PMU_REG_ADDR    0xA0F28000U

static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32          chipName = 0;
    uint32          dualBankVal = 0;
    uint32          dual_bank = 0;
    uint32          expandFlashVal = 0;
    uint32          expand_flash = 0;
    uint32          remap_mode = 0;
    uint32          hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[ 0];
    expandFlashVal = ldt1Addr[ 1];

    dualBankVal &= ldt1Addr[ 4];
    expandFlashVal &= ldt1Addr[ 5];

    dualBankVal &= ldt1Addr[ 8];
    expandFlashVal &= ldt1Addr[ 9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal  = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;            // (single_bank : dual_bank)
    expand_flash  = (expandFlashVal  == 0x0000) ? 0 : 1;    // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[ 0];

    mcu_printf("    CHIP   NAME  : %x\n",    chipName);
    mcu_printf("    DUAL   BANK  : %d\n",    dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n",    expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n",    (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[ 0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n",    hsm_ready);
#endif
}

/* -------------------------------------------------------------------------- */
/* [핵심 1] 캐시 제어 함수 (ASM)                       */
/* -------------------------------------------------------------------------- */
static void Local_CleanInvalidateDCache(uint32 addr, uint32 size)
{
    uint32 start_addr = addr & ~(32UL - 1UL);
    uint32 end_addr = addr + size;

    while (start_addr < end_addr) {
        // Clean & Invalidate Data Cache by MVA
        __asm volatile ("mcr p15, 0, %0, c7, c14, 1" :: "r"(start_addr));
        start_addr += 32;
    }
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

/* -------------------------------------------------------------------------- */
/* [핵심 2] DMA 설정 함수 (Bus 설정 포함)              */
/* -------------------------------------------------------------------------- */
static void Configure_ADC_DMA_Struct(uint32 dest_addr) {
    // 0. 동기화
    *(volatile uint32 *)(0xA0800020UL) = 0xFFFFFFFF; // poSync 주소 직접 쓰기
    ADC_DMA_TEST_REG(0)->poConfig.dcNreg = 0x1; 
    
    // 1. 주소 설정
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcSrcAddrR = (uint32)(&ADC_TEST_REG(0)->atAdcAin00);
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr = dest_addr;

    // 2. Control Register 설정 (버스 설정이 0이던 문제 해결)
    ADCDmaSchCtrl_t dma_ctrl = {0};
    dma_ctrl.scTransferSize = 16;
    dma_ctrl.scSrcBurstSize = 4;
    dma_ctrl.scDestBurstSize = 4;
    dma_ctrl.scSrcWidth = 2;            // 32-bit
    dma_ctrl.scDestWidth = 2;           // 32-bit
    
    dma_ctrl.scSrcIncrement = 1;        // 소스 주소 증가
    dma_ctrl.scDestIncrement = 1;       // 목적지 주소 증가
    
    // [중요] 버스 인터페이스 설정 (adc_test.c 참조)
    dma_ctrl.scSrcBus = 0;              // ADC는 0번 버스
    dma_ctrl.scDestBus = 1;             // RAM은 1번 버스 [해결 포인트]

    dma_ctrl.scInterruptEnable = 1;     // 인터럽트 비트 셋
    
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCtrl.rlBreg = dma_ctrl;

    // 3. Config Register 설정
    ADCDmaSchCfg_t dma_cfg = {0};
    dma_cfg.cfChEnable = 1;
    dma_cfg.cfSrcPeri = 8;              // ADC0
    dma_cfg.cfFlowCtrl = 2;             // Peri-to-Mem
    dma_cfg.cfIntMask = 1;              
    
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg = dma_cfg;

    // 4. ADC 설정
    ADC_TEST_REG(0)->atAdcClk.bReg.clDiv   = 19; 
    ADC_TEST_REG(0)->atAdcClk.bReg.clReqEn = 1;
    ADC_TEST_REG(0)->atAdcClk.bReg.clIrqEn = 1;
}

/* -------------------------------------------------------------------------- */
/* 메인 태스크                                    */
/* -------------------------------------------------------------------------- */
static void Mic_ADC_Task(void * pArg) {
    (void)pArg;
    uint32 buffer_addr = (uint32)(&gAdcDmaBuffer[0]);

    ADC_Init(ADC_MODE_NORMAL, ADC_MODULE_0);
    
    mcu_printf("\r\n[DEBUG] DMA with Correct BUS Config & Structs...\r\n");

    while(1) {
        // [추가] 인터럽트 클리어 (다음 전송을 위해 필수)
        // 정의된 구조체 멤버 poIrqItcStatus를 사용하여 상태 확인 및 클리어
        if (ADC_DMA_TEST_REG(0)->poIrqItcStatus.duNreg) {
            ADC_DMA_TEST_REG(0)->poIrqItcClear.duNreg = ADC_DMA_TEST_REG(0)->poIrqItcStatus.duNreg;
        }

        // DMA 설정 및 시작
        Configure_ADC_DMA_Struct(buffer_addr);

        // ADC 트리거
        ADC_TEST_REG(0)->atAdcCmd.cuNreg = 0xFFFF; 

        SAL_TaskSleep(100); 

        // 캐시 비우기
        Local_CleanInvalidateDCache(buffer_addr, ADC_DMA_BUF_SIZE * 4);

        mcu_printf("DMA_BUF: ");
        for(int i = 0; i < 4; i++) {
            mcu_printf("[%d]:%d ", i, (int)(gAdcDmaBuffer[i] & 0xFFF));
        }

        uint32 hw_val = ADC_TEST_REG(0)->atAdcAin03 & 0xFFF;
        mcu_printf("| HW_REG[3]:%d\r\n", (int)hw_val);
    }
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

