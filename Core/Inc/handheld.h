/**
  ******************************************************************************
  * @file    handheld.h
  * @brief   手持控制器业务逻辑头文件
  ******************************************************************************
  */
#ifndef __HANDHELD_H
#define __HANDHELD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nrf24l01.h"
#include "jy901s.h"

/* 接口函数 -------------------------------------------------------------------*/
void Handheld_Init(void);
void Handheld_Process(void);
uint8_t Handheld_GetDisplayData(float *pRoll, float *pPitch, float *pYaw, uint8_t *pGripClosed);
uint8_t Handheld_GetRfStatus(void);   /* NRF24L01 模块自检结果: 1=正常 0=异常 */

#ifdef __cplusplus
}
#endif

#endif /* __HANDHELD_H */
