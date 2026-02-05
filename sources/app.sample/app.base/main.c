// SPDX-License-Identifier: Apache-2.0

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <adc.h>
#include <gic.h> 
#include <timer.h>

/* ========================================================================== */
/* 1. 설정 및 정의                                                            */
/* ========================================================================== */
static void Local_Delay(uint32 count) {
    for(volatile uint32 k=0; k<count; k++);
}

#define MIC_EVENT_FLAG      0x00000001 
static volatile uint32 gTimerTriggerFlag = 0;

// [설정] 4개 중 1개만 전송 (Skipping)
#define SKIP_STEP           2     
static int g_SkipCounter = 0; 

// 버퍼 및 변수
static volatile uint16 gAdcDmaBuffer[16] __attribute__((aligned(32)));
static uint32 gMicAdcTaskID    = 0;
static uint32 gMicAdcTaskStk[ACFG_TASK_MEDIUM_STK_SIZE];

// [에러 해결] 콘솔 라이브러리에서 참조하는 전역 변수
uint32 gALiveMsgOnOff = 0; 

// 레지스터 주소 및 정의
#define ADC_DMA_TEST_REG(N)    ((volatile ADCDmaxPort_t  *)(0xA0800000UL + (0x10000 * (N))))
#define ADC_TEST_REG(N)        ((volatile ADCTest_t      *)((0xA0080000UL) + ((N) * 0x10000UL)))
#define ADC_TEST_REG_SM(N)     ((volatile ADCTestAdcSM_t *)((0xA00A0000UL) + (N * 0x10000UL)))
#define HwHCLK_MASK1           (0xA0F20004UL) 
#define HwSW_RESET1            (0xA0F20010UL) 

/* --- 구조체 정의 (필수) --- */
typedef struct ADCSmCmd { uint32 scCmd:16; uint32 :15; uint32 scCnvDone:1; } ADCSmCmd_t;
typedef union { uint32 nReg; ADCSmCmd_t bReg; } ADCSmCmdU_t;
typedef struct ADCSmCtrl { uint32 smAmmEn:1; uint32 :3; uint32 smErrClr:1; uint32 :27; } ADCSmCtrl_t;
typedef union { uint32 nReg; ADCSmCtrl_t bReg; } ADCSmCtrlU_t;
typedef struct ADCSmToVal { uint32 tvToVal:32; } ADCSmToVal_t;
typedef union { uint32 nReg; ADCSmToVal_t bReg; } ADCSmToValU_t;
typedef struct ADCSmAckToVal { uint32 atAckToVal:16; uint32 :16; } ADCSmAckToVal_t;
typedef union { uint32 nReg; ADCSmAckToVal_t bReg; } ADCSmAckToValU_t;
typedef struct ADCTestAdcSM { ADCSmCmdU_t smAdcSmCmd; uint32 smReserved0x004[3]; ADCSmCtrlU_t smAdcSmCtrl; ADCSmToValU_t smAdcSmToVal; ADCSmAckToValU_t smAdcSmAckToVal; } ADCTestAdcSM_t;
typedef struct ADCCmd { uint32 acCmd:16; uint32 acNofAin:4; uint32 acCnvCnt:5; uint32 :6; uint32 acCnvDone:1; } ADCCmd_t;
typedef union { uint32 cuNreg; ADCCmd_t cuBreg; } ADCCmdUnion_t;
typedef struct { uint32 clDiv:8; uint32 clIrqEn:1; uint32 clReqEn:1; uint32 :22; } ADCTestAdcclk_t;
typedef union { uint32 nReg; ADCTestAdcclk_t bReg; } ADCTestAdcclkU_t;
typedef struct ADCTest { ADCCmdUnion_t atAdcCmd; uint32 atAdcClr; ADCTestAdcclkU_t atAdcClk; uint32 atReserved[13]; uint32 atAdcTime; uint32 atReserved2[15]; uint32 atAdcAin00; uint32 atAdcAin01; uint32 atAdcAin02; uint32 atAdcAin03; } ADCTest_t;
typedef struct { uint32 scTransferSize:12; uint32 scSrcBurstSize:3; uint32 scDestBurstSize:3; uint32 scSrcWidth:3; uint32 scDestWidth:3; uint32 scSrcBus:1; uint32 scDestBus:1; uint32 scSrcIncrement:1; uint32 scDestIncrement:1; uint32 scProtection:3; uint32 scInterruptEnable:1; } ADCDmaSchCtrl_t;
typedef union { uint32 rlNreg; ADCDmaSchCtrl_t rlBreg; } ADCUchCtrol_t;
typedef struct { uint32 cfChEnable:1; uint32 cfSrcPeri:5; uint32 cfDstPeri:5; uint32 cfFlowCtrl:3; uint32 cfErrIntMask:1; uint32 cfIntMask:1; uint32 cfBusLock:1; uint32 cfActive:1; uint32 cfHalt:1; uint32 cfReserved:13; } ADCDmaSchCfg_t;
typedef union { uint32 cfNreg; ADCDmaSchCfg_t cfBreg; } ADCUchCfg_t;
typedef struct { uint32 slNextLliBus:1; uint32 slreserved:1; uint32 slNextLli:30; } ADCDmaSchLli_t;
typedef union { uint32 dlNreg; ADCDmaSchLli_t dlBreg; } ADCDmaUlli_t;
typedef volatile struct { uint32 xcSrcAddrR; uint32 xcDestAddr; ADCDmaUlli_t xcLli; ADCUchCtrol_t xcCtrl; ADCUchCfg_t xcCfg; uint32 xcReserved[3]; } ADCDmaxChannel_t;
typedef union { uint32 duNreg; uint32 duBreg; } ADCDmaUch_t;
typedef volatile struct { ADCDmaUch_t poIrqStatus; ADCDmaUch_t poIrqItcStatus; ADCDmaUch_t poIrqItcClear; ADCDmaUch_t poIrqErrStatus; ADCDmaUch_t poIrqErrClear; ADCDmaUch_t poRawIrqItcStatus; ADCDmaUch_t poRawIrqErrStatus; ADCDmaUch_t poEnabledChannel; uint32 poSwReq[4]; union { uint32 dcNreg; uint32 dcBreg; } poConfig; uint32 poSync; uint32 reserved[50]; ADCDmaxChannel_t poDmaChannel[2]; } ADCDmaxPort_t;

/* --- 함수 선언 --- */
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Mic_ANC_Task(void * pArg);
static sint32 Mic_Timer_Handler(TIMERChannel_t iChannel, void * pArgs);
static void Local_CleanInvalidateDCache(uint32 addr, uint32 size);
static void Configure_ADC_DMA_Stable(uint32 dest_addr);

/* --- 유틸리티 및 타이머 핸들러 --- */
static void Configure_ADC_DMA_Stable(uint32 dest_addr) {
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg.cfChEnable = 0;
    *(volatile uint32 *)(0xA0800020UL) = 0xFFFFFFFF; 
    ADC_DMA_TEST_REG(0)->poSync = 0xFFFF; 
    ADC_DMA_TEST_REG(0)->poConfig.dcNreg = 0x1; 
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcSrcAddrR = (uint32)(&ADC_TEST_REG(0)->atAdcAin00);
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr = dest_addr;

    ADCDmaSchCtrl_t dma_ctrl = {0};
    dma_ctrl.scTransferSize = 1; 
    dma_ctrl.scSrcWidth = 1; dma_ctrl.scDestWidth = 1; 
    dma_ctrl.scSrcBus = 0; dma_ctrl.scDestBus = 1; 
    dma_ctrl.scInterruptEnable = 0; 
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCtrl.rlBreg = dma_ctrl;

    ADCDmaSchCfg_t dma_cfg = {0};
    dma_cfg.cfChEnable = 1; 
    dma_cfg.cfSrcPeri = 8; 
    dma_cfg.cfFlowCtrl = 2; 
    dma_cfg.cfIntMask = 1; 
    ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg = dma_cfg;

    ADC_TEST_REG(0)->atAdcClk.bReg.clDiv = 19; 
    ADC_TEST_REG(0)->atAdcClk.bReg.clReqEn = 1; 
}

static void Local_CleanInvalidateDCache(uint32 addr, uint32 size) {
    uint32 start = addr & ~(32UL - 1UL);
    uint32 end = addr + size;
    while (start < end) {
        __asm volatile ("mcr p15, 0, %0, c7, c14, 1" :: "r"(start));
        start += 32;
    }
    __asm volatile ("dsb"); __asm volatile ("isb");
}

static sint32 Mic_Timer_Handler(TIMERChannel_t iChannel, void * pArgs) {
    (void)iChannel; (void)pArgs;
    gTimerTriggerFlag = 1; 
    return (sint32)SAL_RET_SUCCESS;
}

/* --- 메인 엔진 --- */
void cmain (void) {
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init(); BSP_PreInit(); BSP_Init();
    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"Start", (SALTaskFunc) &Main_StartTask, &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg) {
    (void)pArg; (void)SAL_OsInitFuncs(); 
    AppTaskCreate();
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void AppTaskCreate(void) {
    (void)SAL_TaskCreate(&gMicAdcTaskID, (const uint8 *)"ANC Task", (SALTaskFunc) &Mic_ANC_Task, &gMicAdcTaskStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG + 1, NULL);
}

/* [핵심] 완전 폴링 방식 태스크 (인터럽트 없음) */
static void Mic_ANC_Task(void * pArg) {
    (void)pArg;
    uint32 buffer_addr = (uint32)(&gAdcDmaBuffer[0]);
    
    static int stage1_val = 1550; // stage2 변수 삭제
    
    // [핵심 수정 1] Alpha를 높여서 반응 속도를 올림 (20 -> 50)
    // 값이 클수록 지연이 줄어들지만 고주파가 덜 깎입니다. 타협점이 필요합니다.
    const int alpha = 50; 
    const int bias = 1550;

    mcu_printf("\n[Polling Mode] Initializing HW...\n");
    
    uint32 clk_mask = SAL_ReadReg(HwHCLK_MASK1);
    SAL_WriteReg(clk_mask | ((uint32)1 << 10), HwHCLK_MASK1);
    
    uint32 temp_reset = SAL_ReadReg(HwSW_RESET1); 
    SAL_WriteReg(temp_reset & ~(1UL << 10), HwSW_RESET1); 
    Local_Delay(1000);
    SAL_WriteReg(temp_reset | (1UL << 10), HwSW_RESET1);  

    ADC_TEST_REG_SM(0)->smAdcSmCtrl.bReg.smAmmEn = (uint32)0x1;
    Configure_ADC_DMA_Stable(buffer_addr);

    (void)TIMER_EnableWithMode(TIMER_CH_1, 62, TIMER_OP_FREERUN, &Mic_Timer_Handler, NULL);
    GIC_IntSrcEn((uint32)GIC_TIMER_0 + (uint32)TIMER_CH_1);

    mcu_printf("[Polling Mode] Loop Started (SKIP=4).\n");

    while(1) {
        if (gTimerTriggerFlag == 1) {
            gTimerTriggerFlag = 0; 

            // ADC 및 DMA 트리거 (기존 동일)
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg.cfChEnable = 0;
            ADC_DMA_TEST_REG(0)->poIrqItcClear.duNreg = 0x01;
            ADC_TEST_REG(0)->atAdcClr = 0xFFFFFFFF;
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr = buffer_addr; 
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCtrl.rlBreg.scTransferSize = 1; 
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg.cfChEnable = 1;
            ADC_TEST_REG(0)->atAdcCmd.cuNreg = (1 << 3) | (0 << 20); 

            volatile uint32 timeout = 10000;
            while(!(ADC_TEST_REG(0)->atAdcCmd.cuBreg.acCnvDone) && --timeout);

            Local_CleanInvalidateDCache(buffer_addr, 32);
            int raw_val = (int)(gAdcDmaBuffer[0] & 0xFFFF);
            
            /* --- [수정된 ANC 로직: 1단계 필터] --- */
            // [핵심 수정 2] 2단계(stage2)를 제거하고 1단계만 수행하여 위상 지연을 최소화
            stage1_val = (alpha * raw_val + (100 - alpha) * stage1_val) / 100;
            
            // 2. 위상 반전 (Anti-Noise)
            // stage2가 아닌 stage1을 바로 사용하여 즉각적인 반대 파형 생성
            int anti_noise = bias - (stage1_val - bias);
            
            // 3. 신호 중첩
            int cancelled_val = raw_val + (anti_noise - bias);
            
            if (++g_SkipCounter >= SKIP_STEP) {
                // 데이터 전송
                mcu_printf("%d,%d,%d\n", raw_val, stage1_val, cancelled_val);
                g_SkipCounter = 0;
            }
        }
        (void)SAL_TaskSleep(0);
    }
}

#endif

