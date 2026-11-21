/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @brief   NRF24L01+ 发送模式驱动实现
  ******************************************************************************
  */
#include "nrf24l01.h"

/* 射频参数(必须与机械臂端完全一致) --------------------------------------------*/
#define NRF24_RF_CHANNEL      40      /* 载波频率 2400+40 = 2440MHz */
#define NRF24_RF_SETUP_VAL    0x06    /* 1Mbps, 0dBm */
#define NRF24_SETUP_RETR_VAL  0x13    /* 自动重发: 500us 间隔, 最多 3 次 */

/* 5 字节发送地址(必须与机械臂端接收地址一致) */
static const uint8_t s_txAddress[5] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E};

/* 微秒级近似延时(72MHz 主频) */
static void NRF24_DelayUs(uint32_t us)
{
    volatile uint32_t cnt;
    for (cnt = 0; cnt < us * 6u; cnt++)
    {
        __NOP();
    }
}

/* 引脚控制 */
static void NRF24_CSN_Low(void)
{
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

static void NRF24_CSN_High(void)
{
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

static void NRF24_CE_Low(void)
{
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

static void NRF24_CE_High(void)
{
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

/* SPI 单字节收发 */
static uint8_t NRF24_SPI_RW(uint8_t data)
{
    uint8_t rx = 0x00;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

/* 写单个寄存器 */
static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_W_REGISTER | reg);
    NRF24_SPI_RW(value);
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 读单个寄存器 */
static uint8_t NRF24_ReadReg(uint8_t reg)
{
    uint8_t value;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_R_REGISTER | reg);
    value = NRF24_SPI_RW(NRF24_CMD_NOP);
    NRF24_CSN_High();
    NRF24_DelayUs(1);
    return value;
}

/* 连续写多个寄存器 */
static void NRF24_WriteBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_W_REGISTER | reg);
    for (i = 0; i < len; i++)
    {
        NRF24_SPI_RW(pBuf[i]);
    }
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 连续读多个寄存器 */
static void NRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_R_REGISTER | reg);
    for (i = 0; i < len; i++)
    {
        pBuf[i] = NRF24_SPI_RW(NRF24_CMD_NOP);
    }
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 初始化 NRF24L01 为发送模式 */
void NRF24_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 将 CSN 引脚(PA4, CubeMX 中为 SPI1_NSS 软件模式)复用为普通推挽输出 */
    gpio.Pin   = NRF24_CSN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24_CSN_PORT, &gpio);

    /* 初始引脚状态: CE=0, CSN=1 */
    NRF24_CE_Low();
    NRF24_CSN_High();
    HAL_Delay(20);                           /* 等待模块上电稳定 */

    /* 配置寄存器(先保持 PWR_UP=0 以便写入) */
    NRF24_WriteReg(NRF24_REG_CONFIG,     0x0C);
    NRF24_WriteReg(NRF24_REG_EN_AA,      0x01);   /* 使能管道0自动应答 */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR,  0x01);   /* 使能管道0接收(用于收ACK) */
    NRF24_WriteReg(NRF24_REG_SETUP_AW,   0x03);   /* 地址宽度 5 字节 */
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, NRF24_SETUP_RETR_VAL);
    NRF24_WriteReg(NRF24_REG_RF_CH,      NRF24_RF_CHANNEL);
    NRF24_WriteReg(NRF24_REG_RF_SETUP,   NRF24_RF_SETUP_VAL);
    NRF24_WriteBuf(NRF24_REG_TX_ADDR,    s_txAddress, 5);   /* 发送地址 */
    NRF24_WriteBuf(NRF24_REG_RX_ADDR_P0, s_txAddress, 5);   /* ACK 接收地址 */
    NRF24_WriteReg(NRF24_REG_RX_PW_P0,   NRF24_PAYLOAD_WIDTH);

    /* 清除中断标志并清空收发 FIFO */
    NRF24_WriteReg(NRF24_REG_STATUS, 0x70);
    NRF24_CSN_Low();  NRF24_SPI_RW(NRF24_CMD_FLUSH_TX); NRF24_CSN_High();
    NRF24_CSN_Low();  NRF24_SPI_RW(NRF24_CMD_FLUSH_RX); NRF24_CSN_High();

    /* 进入发送模式, CONFIG = 0x7A:
     *   bit0 PRIM_RX=0 发送模式
     *   bit1 PWR_UP=1  上电
     *   bit2 CRCO=0    1 字节 CRC
     *   bit3 EN_CRC=1  使能 CRC
     *   bit4~6 = 1     屏蔽全部中断, 由软件轮询 STATUS */
    NRF24_WriteReg(NRF24_REG_CONFIG, 0x7A);

    NRF24_CE_Low();                          /* 保持待机, 发送时再拉高 */
    HAL_Delay(1);
}

/* 检测模块通信是否正常(写读回 TX_ADDR 校验) */
uint8_t NRF24_Check(void)
{
    uint8_t wr[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t rd[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t i;

    NRF24_WriteBuf(NRF24_REG_TX_ADDR, wr, 5);
    NRF24_ReadBuf(NRF24_REG_TX_ADDR, rd, 5);

    for (i = 0; i < 5; i++)
    {
        if (rd[i] != 0xA5)
        {
            return 0;                        /* 通信异常 */
        }
    }

    /* 自检后必须恢复真实发送地址, 否则会以 0xA5 作为发送地址导致对端收不到 */
    NRF24_WriteBuf(NRF24_REG_TX_ADDR, s_txAddress, 5);
    return 1;                                /* 通信正常 */
}

/* 发送一个数据包, 发送成功(收到ACK)返回 1, 否则返回 0 */
uint8_t NRF24_TxPacket(const uint8_t *pBuf, uint8_t len)
{
    uint8_t  status;
    uint8_t  i;
    uint32_t t0;

    /* 若发送 FIFO 已满(上次未发完), 先清空避免阻塞 */
    if (NRF24_ReadReg(NRF24_REG_FIFO_STATUS) & 0x10)   /* TX_FULL */
    {
        NRF24_CSN_Low(); NRF24_SPI_RW(NRF24_CMD_FLUSH_TX); NRF24_CSN_High();
    }

    /* 写入负载 */
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_W_TX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
        NRF24_SPI_RW(pBuf[i]);
    }
    NRF24_CSN_High();

    /* 拉高 CE 触发发送(>10us) */
    NRF24_CE_High();
    NRF24_DelayUs(15);

    /* 等待发送完成(TX_DS)或达到最大重发(MAX_RT), 最多 10ms */
    t0 = HAL_GetTick();
    do
    {
        status = NRF24_ReadReg(NRF24_REG_STATUS);
        if (status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT))
        {
            break;
        }
    } while ((HAL_GetTick() - t0) < 10);

    NRF24_CE_Low();

    /* 清除中断标志(写 1 清除) */
    NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    return (status & NRF24_STATUS_TX_DS) ? 1 : 0;
}
