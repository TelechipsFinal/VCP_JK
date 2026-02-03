// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
* FileName : main.c
* Description : ADC + DMA Debugging Mode (Register Dump)
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <adc.h>
#include <gic.h> 

#define ADC_DMA_BUF_SIZE       16      
#define MIC_EVENT_FLAG         0x00000001 

#define ADC_DMA_TEST_REG(N)    ((volatile ADCDmaxPort_t  *)(0xA0800000UL + (0x10000 * (N))))
#define ADC_TEST_REG(N)        ((volatile ADCTest_t      *)((0xA0080000UL) + ((N) * 0x10000UL)))

// 디버깅용 전역 변수
static volatile uint32 gAdcDmaBuffer[ADC_DMA_BUF_SIZE] __attribute__((aligned(32)));
static uint32 gMicReadyEventID = 0; 
static uint32 gMicAdcTaskID = 0;
static uint32 gMicAdcTaskStk[ACFG_TASK_MEDIUM_STK_SIZE];
uint32 gALiveMsgOnOff;
static uint32 gALiveCount;

/* --- 구조체 정의 (생략 없이 포함) --- */
typedef struct ADCCmd { uint32 acCmd:16; uint32 acNofAin:4; uint32 acCnvCnt:5; uint32 :6; uint32 acCnvDone:1; } ADCCmd_t;
typedef union { uint32 cuNreg; ADCCmd_t cuBreg; } ADCCmdUnion_t;
typedef struct { uint32 clDiv:8; uint32 clIrqEn:1; uint32 clReqEn:1; uint32 :22; } ADCTestAdcclk_t;
typedef union { uint32 nReg; ADCTestAdcclk_t bReg; } ADCTestAdcclkU_t;
typedef struct ADCTest {
    ADCCmdUnion_t atAdcCmd; uint32 atAdcClr; ADCTestAdcclkU_t atAdcClk; uint32 atReserved[13];
    uint32 atAdcTime; uint32 atReserved2[15];
    uint32 atAdcAin00; uint32 atAdcAin01; uint32 atAdcAin02; uint32 atAdcAin03; 
} ADCTest_t;
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
typedef union { uint32 duNreg; uint32 duBreg; } ADCDmaUch_t;
typedef volatile struct {
    ADCDmaUch_t     poIrqStatus;        
    ADCDmaUch_t     poIrqItcStatus;     
    ADCDmaUch_t     poIrqItcClear;      
    ADCDmaUch_t     poIrqErrStatus;     
    ADCDmaUch_t     poIrqErrClear;      
    ADCDmaUch_t     poRawIrqItcStatus;  
    ADCDmaUch_t     poRawIrqErrStatus;  
    ADCDmaUch_t     poEnabledChannel;   
    uint32          poSwReq[4];         
    union { uint32 dcNreg; uint32 dcBreg; } poConfig; 
    uint32          poSync;             
    uint32          reserved[50];       
    ADCDmaxChannel_t poDmaChannel[2];   
} ADCDmaxPort_t;

/* --- 함수 선언 --- */
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);
static void Mic_ANC_Task(void * pArg);
static void ADC0_ISR(void *pArg);
static void Configure_ADC_DMA_Simple(uint32 dest_addr);
static void Local_CleanInvalidateDCache(uint32 addr, uint32 size);
static void Dump_DMA_Status(char* tag); // [신규] 디버깅 함수

/* ========================================================================== */
/* SYSTEM FUNCTIONS                                                           */
/* ========================================================================== */
void cmain (void) {
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init(); BSP_PreInit(); BSP_Init();
    SALMcuVersionInfo_t vInfo = {0}; SAL_GetVersion(&vInfo);
    mcu_printf("\n=== MCU BSP V%d.%d.%d ===\n", vInfo.viMajorVersion, vInfo.viMinorVersion, vInfo.viPatchVersion);
    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"Start", (SALTaskFunc) &Main_StartTask, &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}
static void Main_StartTask(void * pArg) {
    (void)pArg; (void)SAL_OsInitFuncs(); AppTaskCreate();
    while (1) { DisplayAliveLog(); (void)SAL_TaskSleep(5000); }
}
static void AppTaskCreate(void) {
#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    CreateConsoleTask();
#endif 
    (void)SAL_TaskCreate(&gMicAdcTaskID, (const uint8 *)"ANC Task", (SALTaskFunc) &Mic_ANC_Task, &gMicAdcTaskStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG + 1, NULL);
}
static void DisplayAliveLog(void) {
    if (gALiveMsgOnOff != 0U) { mcu_printf("\n %d", gALiveCount++); if(gALiveCount >= MAIN_UINT_MAX_NUM) gALiveCount = 0; }
    else { gALiveCount = 0; }
}
static void DisplayOTPInfo(void) {}
static void Local_CleanInvalidateDCache(uint32 addr, uint32 size) {
    uint32 start = addr & ~(32UL - 1UL);
    uint32 end = addr + size;
    while (start < end) {
        __asm volatile ("mcr p15, 0, %0, c7, c14, 1" :: "r"(start));
        start += 32;
    }
    __asm volatile ("dsb"); __asm volatile ("isb");
}

/* -------------------------------------------------------------------------- */
/* [신규] DMA 상태 덤프 함수 (핵심 디버깅 도구)                                  */
/* -------------------------------------------------------------------------- */
static void Dump_DMA_Status(char* tag) {
    uint32 cfg_reg  = ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfNreg;
    uint32 dest_addr= ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr;
    uint32 raw_itc  = ADC_DMA_TEST_REG(0)->poRawIrqItcStatus.duNreg;
    uint32 raw_err  = ADC_DMA_TEST_REG(0)->poRawIrqErrStatus.duNreg;
    uint32 enabled  = ADC_DMA_TEST_REG(0)->poEnabledChannel.duNreg;

    // 현재 얼마나 이동했는지 계산 (현재 Dest - 시작 Dest)
    uint32 transferred_bytes = dest_addr - (uint32)&gAdcDmaBuffer[0];
    
    mcu_printf("[%s] En:%d | Err:0x%X | ITC:0x%X | Moved:%d Bytes\n", 
               tag, 
               (enabled & 0x1),  // 1이면 켜져있음, 0이면 꺼짐(완료/에러)
               raw_err,          // 에러 상태
               raw_itc,          // 완료 상태
               transferred_bytes // 이동한 바이트 수 (16이어야 정상인데 4에서 멈추는지 확인)
               );
}

/* -------------------------------------------------------------------------- */
/* ISR                                                                        */
/* -------------------------------------------------------------------------- */
static void ADC0_ISR(void *pArg)
{
    (void)pArg;
    ADC_TEST_REG(0)->atAdcClr = 1; 

    // ISR 진입 확인용 (너무 많이 뜨면 주석 처리)
    // mcu_printf("I"); 

    if (ADC_DMA_TEST_REG(0)->poRawIrqItcStatus.duNreg & 0x01) {
        ADC_DMA_TEST_REG(0)->poIrqItcClear.duNreg = 0x01;
        (void)SAL_EventSet(gMicReadyEventID, MIC_EVENT_FLAG, 0);
    }
}

/* -------------------------------------------------------------------------- */
/* DMA Setup                                                                  */
/* -------------------------------------------------------------------------- */
static void Configure_ADC_DMA_Simple(uint32 dest_addr) {
    *(volatile uint32 *)(0xA0800020UL) = 0xFFFFFFFF; 
    ADC_DMA_TEST_REG(0)->poSync = 0; // Sync 끔
    ADC_DMA_TEST_REG(0)->poConfig.dcNreg = 0x1; 
    
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcSrcAddrR = (uint32)(&ADC_TEST_REG(0)->atAdcAin00);
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr = dest_addr;

    ADCDmaSchCtrl_t dma_ctrl = {0};
    dma_ctrl.scTransferSize = 16; // 16 items
    
    // [설정] Burst=1 (4 items) 유지
    // 이전 로그에서 4개씩은 잘 들어갔음
    dma_ctrl.scSrcBurstSize = 1; 
    dma_ctrl.scDestBurstSize = 1;
    
    dma_ctrl.scSrcWidth = 2; dma_ctrl.scDestWidth = 2; 
    dma_ctrl.scSrcIncrement = 0; dma_ctrl.scDestIncrement = 1; 
    dma_ctrl.scSrcBus = 0; dma_ctrl.scDestBus = 1; 
    dma_ctrl.scInterruptEnable = 1; 
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCtrl.rlBreg = dma_ctrl;

    ADCDmaSchCfg_t dma_cfg = {0};
    dma_cfg.cfChEnable = 1;
    dma_cfg.cfSrcPeri = 8; // ID 8
    dma_cfg.cfFlowCtrl = 2;
    dma_cfg.cfIntMask = 1;
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg = dma_cfg;

    ADC_TEST_REG(0)->atAdcClk.bReg.clDiv = 19; 
    ADC_TEST_REG(0)->atAdcClk.bReg.clReqEn = 1; 
    ADC_TEST_REG(0)->atAdcClk.bReg.clIrqEn = 1; 
}

/* -------------------------------------------------------------------------- */
/* MAIN TASK (Debugging Enabled)                                              */
/* -------------------------------------------------------------------------- */
static void Mic_ANC_Task(void * pArg) {
    (void)pArg;
    SALRetCode_t err;
    uint32 received_flags = 0;

    err = SAL_EventCreate(&gMicReadyEventID, (const uint8 *)"MicEvt", 0);
    GIC_IntVectSet(GIC_ADC0, GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_LEVEL_HIGH, (GICIsrFunc)&ADC0_ISR, NULL);
    GIC_IntSrcEn(GIC_ADC0);

    mcu_printf("\r\n[ANC] Debugging Mode Started...\r\n");

    while(1) {
        if (ADC_DMA_TEST_REG(0)->poRawIrqItcStatus.duNreg) {
            ADC_DMA_TEST_REG(0)->poIrqItcClear.duNreg = 0xFFFFFFFF;
        }
        
        // 버퍼 초기화
        for(int i=0; i<ADC_DMA_BUF_SIZE; i++) gAdcDmaBuffer[i] = 0xDEADBEEF;
        Local_CleanInvalidateDCache((uint32)&gAdcDmaBuffer[0], ADC_DMA_BUF_SIZE * 4);

        Configure_ADC_DMA_Simple((uint32)&gAdcDmaBuffer[0]);

        // [디버그 1] 시작 직후 상태
        Dump_DMA_Status("START");

        // ADC 트리거 (16개 요청)
        // [수정] 16개 딱 맞춤 (15)
        ADC_TEST_REG(0)->atAdcCmd.cuNreg = (1 << 3) | (15 << 20);

        // [디버그 2] 10ms 대기 후 상태 (진행 중인지 확인)
        SAL_TaskSleep(10); 
        Dump_DMA_Status("CHK_10ms");

        err = SAL_EventGet(gMicReadyEventID, MIC_EVENT_FLAG, SAL_EVENT_OPT_CLR_ALL, 100, &received_flags);

        if (err == SAL_RET_SUCCESS) {
            // [디버그 3] 성공 시 상태
            Dump_DMA_Status("DONE");
            
            Local_CleanInvalidateDCache((uint32)&gAdcDmaBuffer[0], ADC_DMA_BUF_SIZE * 4);
            mcu_printf("RAW: %d %d %d %d [End:%d]\n", 
                       (int)(gAdcDmaBuffer[0] & 0xFFF),
                       (int)(gAdcDmaBuffer[1] & 0xFFF),
                       (int)(gAdcDmaBuffer[2] & 0xFFF),
                       (int)(gAdcDmaBuffer[3] & 0xFFF),
                       (int)(gAdcDmaBuffer[15] & 0xFFF)); 
            
            ADC_TEST_REG(0)->atAdcClk.nReg = 0;
            SAL_TaskSleep(500); // 로그 홍수 방지
        } 
        else {
            // [디버그 4] 타임아웃(실패) 시 상태 (여기가 제일 중요!)
            Dump_DMA_Status("FAIL");
            
            Local_CleanInvalidateDCache((uint32)&gAdcDmaBuffer[0], ADC_DMA_BUF_SIZE * 4);
            // 어디까지 썼는지 확인
             mcu_printf("Buf Dump: %x %x %x %x ... %x\n", 
                       gAdcDmaBuffer[0], gAdcDmaBuffer[1], gAdcDmaBuffer[2], gAdcDmaBuffer[3], gAdcDmaBuffer[15]);

            ADC_TEST_REG(0)->atAdcClk.nReg = 0;
            SAL_TaskSleep(500); 
        }
    }
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )