/**
  ******************************************************************************
  * @file    nrf24l01.h
  * @brief   NRF24L01+ 2.4GHz 无线模块驱动(手持端, 发送模式)
  * @note    基于 HAL 库, 使用 SPI1; CE=PB0, CSN=PA4(SPI1_NSS 软件模式复用)
  ******************************************************************************
  */
#ifndef __NRF24L01_H
#define __NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* 引脚定义 -------------------------------------------------------------------*/
/* CE  —— PB0 (CubeMX 已配置为推挽输出, 定义见 main.h 中 CE_Pin/CE_GPIO_Port) */
/* CSN —— PA4 (CubeMX 中为 SPI1_NSS 软件模式, 驱动内将其复用为片选输出) */
#define NRF24_CE_PORT     CE_GPIO_Port
#define NRF24_CE_PIN      CE_Pin
#define NRF24_CSN_PORT    GPIOA
#define NRF24_CSN_PIN     GPIO_PIN_4

/* 发送负载宽度(字节), 必须与机械臂端一致 */
#define NRF24_PAYLOAD_WIDTH   32

/* 寄存器地址 -----------------------------------------------------------------*/
#define NRF24_REG_CONFIG       0x00
#define NRF24_REG_EN_AA        0x01
#define NRF24_REG_EN_RXADDR    0x02
#define NRF24_REG_SETUP_AW     0x03
#define NRF24_REG_SETUP_RETR   0x04
#define NRF24_REG_RF_CH        0x05
#define NRF24_REG_RF_SETUP     0x06
#define NRF24_REG_STATUS       0x07
#define NRF24_REG_RX_ADDR_P0   0x0A
#define NRF24_REG_TX_ADDR      0x10
#define NRF24_REG_RX_PW_P0     0x11
#define NRF24_REG_FIFO_STATUS  0x17

/* SPI 命令 -------------------------------------------------------------------*/
#define NRF24_CMD_R_REGISTER      0x00
#define NRF24_CMD_W_REGISTER      0x20
#define NRF24_CMD_R_RX_PAYLOAD    0x61
#define NRF24_CMD_W_TX_PAYLOAD    0xA0
#define NRF24_CMD_FLUSH_TX        0xE1
#define NRF24_CMD_FLUSH_RX        0xE2
#define NRF24_CMD_NOP             0xFF

/* STATUS 寄存器位 -------------------------------------------------------------*/
#define NRF24_STATUS_TX_DS   0x20    /* 数据发送完成(收到ACK) */
#define NRF24_STATUS_MAX_RT  0x10    /* 达到最大重发次数 */

/* 接口函数 -------------------------------------------------------------------*/
void    NRF24_Init(void);
uint8_t NRF24_Check(void);
uint8_t NRF24_TxPacket(const uint8_t *pBuf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_H */
