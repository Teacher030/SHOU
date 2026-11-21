/**
  ******************************************************************************
  * @file    jy901s.h
  * @brief   JY901S 姿态传感器 I2C 读取头文件
  * @note    通过软件 I2C(PB6=SCL, PB7=SDA) 读取 Roll/Pitch/Yaw 角度
  ******************************************************************************
  */
#ifndef __JY901S_H
#define __JY901S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 接口函数 -------------------------------------------------------------------*/
void    Jy901s_Init(void);
void    Jy901s_Poll(void);                    /* 主循环中反复调用, I2C 读取并解析 */
uint8_t Jy901s_IsAngleUpdated(void);          /* 是否有新的有效角度 */
void    Jy901s_GetAngle(float *pRoll, float *pPitch, float *pYaw);  /* 读取角度(度) */

#ifdef __cplusplus
}
#endif

#endif /* __JY901S_H */
