// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c
* Description : Final Step - Official Audio Test Sequence
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <i2s.h>
#include <clock.h>
#include <clock_dev.h>
#include <gpio.h>
#include <mpu.h> // MPU_GetDMABaseAddress 사용을 위해 추가

uint32 gALiveMsgOnOff = 1;

/* 예제와 동일한 버퍼 크기 설정 */
#define AUDIO_PERIOD_SIZE  0x400U
#define AUDIO_BUFFER_SIZE  0x2000U

static I2SConfig_t g_I2sConfig;
extern uint32 I2S_GetRxDaCdar(void);

static void Main_StartTask(void * pArg);
static void Audio_OfficialTask(void * pArg);

void cmain (void)
{
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init();
    BSP_PreInit(); 
    BSP_Init(); 
    
    mcu_printf("\n[System] Starting Official Audio Test Sequence...\n");
    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"Start", (SALTaskFunc) &Main_StartTask, 
                         &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg)
{
    (void)pArg; (void)SAL_OsInitFuncs();
    static uint32 AudioTaskID;
    static uint32 AudioTaskStk[ACFG_TASK_MEDIUM_STK_SIZE]; 
    (void)SAL_TaskCreate(&AudioTaskID, (const uint8 *)"AudioOfficial", (SALTaskFunc) &Audio_OfficialTask,
                         &AudioTaskStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void Audio_OfficialTask(void * pArg)
{
    (void)pArg;
    uint32 tick = 0;
    
    /* 예제 방식의 버퍼 주소 획득 (Non-cacheable area) */
    uint32 * AUDIO_RxBuffer = (uint32 *)MPU_GetDMABaseAddress();
    
    SAL_TaskSleep(3000);
    mcu_printf("\n========== OFFICIAL SEQUENCE START ==========\n");

    /* 1. I2S 기본 설정 (audio_test.c 방식) */
    g_I2sConfig.i2sHwCh         = I2S_CH0;
    g_I2sConfig.i2sMode         = I2S_MASTER_MODE;
    g_I2sConfig.i2sFormat       = I2S_FORMAT_I2S;
    g_I2sConfig.i2sNumCh        = I2S_STEREO;
    g_I2sConfig.i2sLRmode       = I2S_LRMODE_OFF;
    g_I2sConfig.i2sBitPerSample = I2S_BIT_DEPTH_16;
    g_I2sConfig.i2sSampleRate   = I2S_SAMPLE_RATE_32000;
    g_I2sConfig.i2sBclkDiv      = I2S_BCLK_DIV_64;
    g_I2sConfig.i2sMclkDiv      = I2S_MCLK_DIV_6;

    /* 수신 스트림 정보 설정 */
    g_I2sConfig.i2sStreamInfo.i2sIn.i2sDmaAddr = AUDIO_RxBuffer;
    g_I2sConfig.i2sStreamInfo.i2sIn.i2sPeriodBytes = AUDIO_PERIOD_SIZE;
    g_I2sConfig.i2sStreamInfo.i2sIn.i2sBufferBytes = AUDIO_BUFFER_SIZE;
    g_I2sConfig.i2sStreamInfo.i2sIn.i2sThresholdBytes = AUDIO_BUFFER_SIZE - AUDIO_PERIOD_SIZE;

    /* 2. 하드웨어 초기화 시퀀스 (예제 핵심 로직) */
    I2S_SWReset(SALEnabled);  // 리셋 시작
    I2S_SWReset(SALDisabled); // 리셋 해제

    I2S_SetGpiofunction(&g_I2sConfig); // 공식 GPIO 설정 함수
    I2S_SetClock(&g_I2sConfig);        // 공식 클럭 설정 함수
    I2S_DaifSetting(&g_I2sConfig);     // 인터페이스 설정

    I2S_RxAdmaSetting(&g_I2sConfig);   // RX DMA 설정
    I2S_SetTransferSize(&g_I2sConfig, I2S_DIN); // 전송 크기 설정

    I2S_DAMREnable();                  // ★ DMA 모드 활성화 ★
    I2S_FifoClear(I2S_DIN);            // FIFO 정리

    /* 3. 활성화 */
    I2S_Enable(I2S_DIN);
    mcu_printf("[Audio] Official Setup Done. Buffer: 0x%08X\n", (uint32)AUDIO_RxBuffer);

    while(1) {
        uint32 current_ptr = I2S_GetRxDaCdar();
        if (tick % 100 == 0) {
            mcu_printf("[Monitor] DMA_PTR: 0x%08X", current_ptr);
            if (current_ptr != 0) {
                mcu_printf(" -> SUCCESS! Data Flowing.\n");
            } else {
                mcu_printf(" -> Still Zero. Checking B21 voltage...\n");
            }
        }
        tick++;
        SAL_TaskSleep(10);
    }
}
#endif