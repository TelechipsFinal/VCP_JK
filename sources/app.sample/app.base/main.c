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

// [추가] 오디오 코덱 제어용 헤더
#include <wm8904.h> 

/* ========================================================================== */
/* 1. 하드웨어 설정 (I2S 및 ANC 파라미터)                                     */
/* ========================================================================== */

// [중요] I2S 데이터 전송 레지스터 (TX FIFO)
// TCC80xx 매뉴얼 기준 I2S 포트의 데이터 레지스터 주소를 매핑합니다.
// 보통 Base Address + 0x00 또는 0x04입니다. (확인 필요, 여기선 0x00 가정)
#define I2S_TX_FIFO_REG       (*(volatile uint32 *)(MCU_BSP_I2S_BASE + 0x00))

// I2C 설정 (보드 회로도에 따라 채널/포트 변경 필요)
#define CODEC_I2C_CH          (0UL)
#define CODEC_I2C_PORT        (1UL)

// ANC 파라미터
#define ADC_ZERO_POINT        1550    // ADC의 0점 (Bias)
#define DAC_SCALE_FACTOR      16      // 12bit(ADC) -> 16bit(I2S) 증폭 배수

static void Local_Delay(uint32 count) {
    for(volatile uint32 k=0; k<count; k++);
}

#define MIC_EVENT_FLAG      0x00000001 
static volatile uint32 gTimerTriggerFlag = 0;

#define SKIP_STEP           2     
static int g_SkipCounter = 0; 

// 버퍼 및 변수
static volatile uint16 gAdcDmaBuffer[16] __attribute__((aligned(32)));
static uint32 gMicAdcTaskID    = 0;
static uint32 gMicAdcTaskStk[ACFG_TASK_MEDIUM_STK_SIZE];

uint32 gALiveMsgOnOff = 0; 

// 레지스터 주소 및 정의
#define ADC_DMA_TEST_REG(N)    ((volatile ADCDmaxPort_t  *)(0xA0800000UL + (0x10000 * (N))))
#define ADC_TEST_REG(N)        ((volatile ADCTest_t      *)((0xA0080000UL) + ((N) * 0x10000UL)))
#define ADC_TEST_REG_SM(N)     ((volatile ADCTestAdcSM_t *)((0xA00A0000UL) + (N * 0x10000UL)))
#define HwHCLK_MASK1           (0xA0F20004UL) 
#define HwSW_RESET1            (0xA0F20010UL) 

/* --- 구조체 정의 (기존 유지) --- */
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
    dma_ctrl.scTransferSize = 1;  // Latency 최소화: 1개씩 처리
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

/* ========================================================================== */
/* [핵심] ANC Task: ADC 입력 -> 알고리즘 -> I2S(WM8904) 출력                  */
/* ========================================================================== */
static void Mic_ANC_Task(void * pArg) {
    (void)pArg;
    uint32 buffer_addr = (uint32)(&gAdcDmaBuffer[0]);
    
    // 필터 및 예측 변수
    static int stage1_val = 1550; 
    static int prev_stage1_val = 1550;
    
    const int alpha = 50; 
    const int bias = 1550;
    const int pred_gain = 120;

    mcu_printf("\n[ANC] Starting System with WM8904 & I2S...\n");
    
    /* 1. 하드웨어 초기화 (Clock & Reset) */
    uint32 clk_mask = SAL_ReadReg(HwHCLK_MASK1);
    SAL_WriteReg(clk_mask | ((uint32)1 << 10), HwHCLK_MASK1);
    uint32 temp_reset = SAL_ReadReg(HwSW_RESET1); 
    SAL_WriteReg(temp_reset & ~(1UL << 10), HwSW_RESET1); 
    Local_Delay(1000);
    SAL_WriteReg(temp_reset | (1UL << 10), HwSW_RESET1);  

    /* 2. WM8904 (외부 DAC) 초기화 */
    I2SConfig_t i2s_conf = {0}; 
    // audio_test.c를 참고하여 기본 I2S 설정 (필요시 수정)
    i2s_conf.i2sMode = I2S_MASTER_MODE;
    i2s_conf.i2sBitPerSample = I2S_BIT_DEPTH_16; 
    
    // 코덱 초기화 (I2C 통신 사용)
    if (WM8904_Initial(CODEC_I2C_CH, CODEC_I2C_PORT, &i2s_conf) != SAL_RET_SUCCESS) {
        mcu_printf("[Error] WM8904 Init Failed.\n");
    } else {
        // 소리 크기 설정 (0~100)
        WM8904_SetVolume(CODEC_I2C_CH, 85); 
        mcu_printf("[OK] WM8904 Initialized. Volume: 85\n");
    }

    /* 3. ADC 및 타이머 시작 */
    ADC_TEST_REG_SM(0)->smAdcSmCtrl.bReg.smAmmEn = (uint32)0x1;
    Configure_ADC_DMA_Stable(buffer_addr);

    (void)TIMER_EnableWithMode(TIMER_CH_1, 62, TIMER_OP_FREERUN, &Mic_Timer_Handler, NULL);
    GIC_IntSrcEn((uint32)GIC_TIMER_0 + (uint32)TIMER_CH_1);

    mcu_printf("[ANC] Loop Running (Sample-by-Sample)\n");

    while(1) {
        if (gTimerTriggerFlag == 1) {
            gTimerTriggerFlag = 0; 

            // [ADC] 데이터 수집 시작 (Single Sample)
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg.cfChEnable = 0;
            ADC_DMA_TEST_REG(0)->poIrqItcClear.duNreg = 0x01;
            ADC_TEST_REG(0)->atAdcClr = 0xFFFFFFFF;
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcDestAddr = buffer_addr; 
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCtrl.rlBreg.scTransferSize = 1; 
            ADC_DMA_TEST_REG(0)->poDmaChannel[0].xcCfg.cfBreg.cfChEnable = 1;
            ADC_TEST_REG(0)->atAdcCmd.cuNreg = (1 << 3) | (0 << 20); 

            // 변환 대기 (Busy Wait)
            volatile uint32 timeout = 10000;
            while(!(ADC_TEST_REG(0)->atAdcCmd.cuBreg.acCnvDone) && --timeout);

            Local_CleanInvalidateDCache(buffer_addr, 32);
            int raw_val = (int)(gAdcDmaBuffer[0] & 0xFFFF);
            
            /* --- [ANC] 알고리즘 처리 --- */
            
            // 1. LPF (Noise Filter)
            stage1_val = (alpha * raw_val + (100 - alpha) * stage1_val) / 100;
            
            // 2. Prediction (Phase Compensation)
            int slope = stage1_val - prev_stage1_val;
            int predicted_val = stage1_val + (slope * pred_gain / 100);
            prev_stage1_val = stage1_val;
            
            // 3. Phase Invert (Generate Anti-Noise)
            // ADC 중앙값(Bias)을 기준으로 뒤집기
            int anti_noise = bias - (predicted_val - bias);
            
            // 4. Output Scaling (ADC 12bit -> I2S 16bit)
            // (값 - Bias) * Scale
            int i2s_out = (anti_noise - ADC_ZERO_POINT) * DAC_SCALE_FACTOR;

            /* --- [DAC] I2S 출력 (WM8904 -> PAM8403) --- */
            // 계산된 값을 I2S TX FIFO에 바로 씁니다.
            // 스테레오(L/R) 출력을 위해 두 번 쓸 수도 있습니다.
            I2S_TX_FIFO_REG = (uint32)(i2s_out & 0xFFFF); 
            // I2S_TX_FIFO_REG = (uint32)(i2s_out & 0xFFFF); // 필요시 Right 채널 추가
            
            // 데이터 전송 (디버깅용)
            if (++g_SkipCounter >= SKIP_STEP) {
                // mcu_printf는 실전에서 주석 처리 권장
                mcu_printf("%d,%d,%d\n", raw_val, stage1_val, anti_noise);
                g_SkipCounter = 0;
            }
        }
        // Real-time 성능을 위해 Sleep 최소화
        (void)SAL_TaskSleep(0);
    }
}

#endif