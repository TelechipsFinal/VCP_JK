// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : app_cfg.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#ifndef MCU_BSP_APP_CFG_HEADER
#define MCU_BSP_APP_CFG_HEADER

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
/* TASK STACK SIZES : Size of the task stacks (# of WARD entries)                   */
#define ACFG_TASK_USER_STK_SIZE         (128U)
/* normal measn that task has no deep fucnction call or large local variable/array  */
#define ACFG_TASK_NORMAL_STK_SIZE       (128U)
/* medium measn that task has some fucnction call or small local variables/arrays   */
#define ACFG_TASK_MEDIUM_STK_SIZE       (256U)

/* ========================================================================== */
/* [필수] I2S 드라이버 활성화                                                  */
/* ========================================================================== */
#undef MCU_BSP_SUPPORT_DRIVER_I2S
#define MCU_BSP_SUPPORT_DRIVER_I2S      1

#undef AUDIO_RX_ENABLE
#define AUDIO_RX_ENABLE                 // 마이크 수신 활성화

/* ========================================================================== */
/* [수정] Makefile 설정 강제 덮어쓰기 (undef 후 define)                        */
/* ========================================================================== */

// 1. 콘솔 기능 끄기
#ifdef MCU_BSP_SUPPORT_APP_CONSOLE
    #undef MCU_BSP_SUPPORT_APP_CONSOLE
#endif
#define MCU_BSP_SUPPORT_APP_CONSOLE     0

// 2. CAN 기능 끄기
#ifdef MCU_BSP_SUPPORT_CAN_DEMO
    #undef MCU_BSP_SUPPORT_CAN_DEMO
#endif
#define MCU_BSP_SUPPORT_CAN_DEMO        0

// 3. 이더넷 기능 끄기
#ifdef MCU_BSP_SUPPORT_DRIVER_ETH
    #undef MCU_BSP_SUPPORT_DRIVER_ETH
#endif
#define MCU_BSP_SUPPORT_DRIVER_ETH      0

#ifdef MCU_BSP_SUPPORT_DRIVER_GMAC
    #undef MCU_BSP_SUPPORT_DRIVER_GMAC
#endif
#define MCU_BSP_SUPPORT_DRIVER_GMAC     0

// 4. 펌웨어 업데이트 기능 끄기
#ifdef MCU_BSP_SUPPORT_APP_FW_UPDATE
    #undef MCU_BSP_SUPPORT_APP_FW_UPDATE
#endif
#define MCU_BSP_SUPPORT_APP_FW_UPDATE   0

#ifdef MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP
    #undef MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP
#endif
#define MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP 0

// 5. 기타 기능 끄기 (Key, Idle, SPI LED)
#ifdef MCU_BSP_SUPPORT_APP_KEY
    #undef MCU_BSP_SUPPORT_APP_KEY
#endif
#define MCU_BSP_SUPPORT_APP_KEY         0

#ifdef MCU_BSP_SUPPORT_APP_IDLE
    #undef MCU_BSP_SUPPORT_APP_IDLE
#endif
#define MCU_BSP_SUPPORT_APP_IDLE        0

#ifdef MCU_BSP_SUPPORT_APP_SPI_LED
    #undef MCU_BSP_SUPPORT_APP_SPI_LED
#endif
#define MCU_BSP_SUPPORT_APP_SPI_LED     0

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#endif  // MCU_BSP_APP_CFG_HEADER

